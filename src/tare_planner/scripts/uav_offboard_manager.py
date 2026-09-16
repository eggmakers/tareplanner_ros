#!/usr/bin/env python3
import math

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import ExtendedState, State
from mavros_msgs.srv import CommandBool, SetMode
from nav_msgs.msg import Odometry


class UavOffboardManager:
    def __init__(self):
        self.state_topic = rospy.get_param("~state_topic", "/mavros/state")
        self.setpoint_topic = rospy.get_param(
            "~setpoint_topic", "/mavros/setpoint_position/local"
        )
        self.odometry_topic = rospy.get_param(
            "~odometry_topic", "/mavros/local_position/odom"
        )
        self.extended_state_topic = rospy.get_param(
            "~extended_state_topic", "/mavros/extended_state"
        )
        self.arming_service = rospy.get_param("~arming_service", "/mavros/cmd/arming")
        self.set_mode_service = rospy.get_param("~set_mode_service", "/mavros/set_mode")
        self.offboard_mode = rospy.get_param("~offboard_mode", "OFFBOARD")
        self.auto_arm = rospy.get_param("~auto_arm", False)
        self.warmup_seconds = float(rospy.get_param("~warmup_seconds", 2.0))
        self.required_setpoints = int(rospy.get_param("~required_setpoints", 20))
        self.retry_interval = float(rospy.get_param("~retry_interval", 2.0))
        self.max_setpoint_age = float(rospy.get_param("~max_setpoint_age", 0.5))
        self.max_odometry_age = float(rospy.get_param("~max_odometry_age", 0.5))
        self.max_auto_arm_speed = float(rospy.get_param("~max_auto_arm_speed", 0.2))
        self.max_auto_arm_position_error = float(
            rospy.get_param("~max_auto_arm_position_error", 0.35)
        )
        self.require_landed_state = rospy.get_param("~require_landed_state", True)
        self.reassert_offboard_after_mode_exit = rospy.get_param(
            "~reassert_offboard_after_mode_exit", False
        )

        self.state = State()
        self.extended_state = ExtendedState()
        self.extended_state_received = False
        self.odometry = None
        self.last_odometry_time = None
        self.last_setpoint = None
        self.last_setpoint_time = None
        self.first_setpoint_time = None
        self.setpoint_count = 0
        self.last_request_time = rospy.Time(0)
        self.ready_logged = False
        self.offboard_achieved = False
        self.suspended = False

        self.set_mode_client = rospy.ServiceProxy(self.set_mode_service, SetMode)
        self.arming_client = rospy.ServiceProxy(self.arming_service, CommandBool)
        rospy.Subscriber(self.state_topic, State, self.state_callback, queue_size=5)
        rospy.Subscriber(self.setpoint_topic, PoseStamped, self.setpoint_callback, queue_size=20)
        rospy.Subscriber(
            self.odometry_topic, Odometry, self.odometry_callback, queue_size=5
        )
        rospy.Subscriber(
            self.extended_state_topic,
            ExtendedState,
            self.extended_state_callback,
            queue_size=5,
        )

        rospy.logwarn(
            "uav_offboard_manager enabled: mode=%s auto_arm=%s; use only on an intended vehicle",
            self.offboard_mode,
            self.auto_arm,
        )

    def state_callback(self, msg):
        self.state = msg
        if msg.mode == self.offboard_mode:
            self.offboard_achieved = True
        elif (
            self.offboard_achieved
            and not self.reassert_offboard_after_mode_exit
            and not self.suspended
        ):
            self.suspended = True
            rospy.logwarn(
                "uav_offboard_manager: mode changed from OFFBOARD to %s; "
                "automatic OFFBOARD requests are suspended until this node restarts",
                msg.mode,
            )

    def extended_state_callback(self, msg):
        self.extended_state = msg
        self.extended_state_received = True

    def odometry_callback(self, msg):
        self.odometry = msg
        self.last_odometry_time = rospy.Time.now()

    def setpoint_callback(self, msg):
        if self.first_setpoint_time is None:
            self.first_setpoint_time = rospy.Time.now()
        self.last_setpoint = msg
        self.last_setpoint_time = rospy.Time.now()
        self.setpoint_count += 1

    def setpoints_ready(self, now):
        if self.first_setpoint_time is None:
            return False
        return (
            self.setpoint_count >= self.required_setpoints
            and (now - self.first_setpoint_time).to_sec() >= self.warmup_seconds
            and self.last_setpoint_time is not None
            and (now - self.last_setpoint_time).to_sec() <= self.max_setpoint_age
        )

    def auto_arm_checks(self, now):
        if self.odometry is None or self.last_odometry_time is None:
            return False, "waiting for odometry"
        if (now - self.last_odometry_time).to_sec() > self.max_odometry_age:
            return False, "odometry is stale"
        if self.last_setpoint is None:
            return False, "waiting for setpoint"
        if self.require_landed_state and (
            not self.extended_state_received
            or self.extended_state.landed_state != ExtendedState.LANDED_STATE_ON_GROUND
        ):
            return False, "vehicle is not confirmed on ground"

        velocity = self.odometry.twist.twist.linear
        speed = math.sqrt(
            velocity.x * velocity.x + velocity.y * velocity.y + velocity.z * velocity.z
        )
        if not math.isfinite(speed) or speed > self.max_auto_arm_speed:
            return False, "vehicle is moving"

        position = self.odometry.pose.pose.position
        target = self.last_setpoint.pose.position
        values = (position.x, position.y, position.z, target.x, target.y, target.z)
        if not all(math.isfinite(value) for value in values):
            return False, "position or setpoint is non-finite"
        error = math.sqrt(
            (target.x - position.x) ** 2
            + (target.y - position.y) ** 2
            + (target.z - position.z) ** 2
        )
        if error > self.max_auto_arm_position_error:
            return False, "setpoint is too far from current position"
        return True, "ready"

    def request_mode(self):
        try:
            response = self.set_mode_client(base_mode=0, custom_mode=self.offboard_mode)
            if response.mode_sent:
                rospy.loginfo("uav_offboard_manager: requested mode %s", self.offboard_mode)
            else:
                rospy.logwarn("uav_offboard_manager: mode request was rejected")
        except rospy.ServiceException as exc:
            rospy.logwarn("uav_offboard_manager: set_mode service failed: %s", exc)

    def request_arm(self):
        try:
            response = self.arming_client(value=True)
            if response.success:
                rospy.loginfo("uav_offboard_manager: arm requested")
            else:
                rospy.logwarn("uav_offboard_manager: arm request was rejected")
        except rospy.ServiceException as exc:
            rospy.logwarn("uav_offboard_manager: arming service failed: %s", exc)

    def spin(self):
        rate = rospy.Rate(10.0)
        while not rospy.is_shutdown():
            now = rospy.Time.now()
            if not self.state.connected:
                rospy.loginfo_throttle(10.0, "uav_offboard_manager: waiting for MAVROS FCU connection")
                rate.sleep()
                continue
            if self.suspended:
                rospy.loginfo_throttle(
                    10.0,
                    "uav_offboard_manager: suspended after external/failsafe mode takeover (%s)",
                    self.state.mode,
                )
                rate.sleep()
                continue
            if not self.setpoints_ready(now):
                rospy.loginfo_throttle(
                    10.0,
                    "uav_offboard_manager: warming setpoint stream (%d/%d)",
                    self.setpoint_count,
                    self.required_setpoints,
                )
                rate.sleep()
                continue

            complete = self.state.mode == self.offboard_mode and (
                self.state.armed or not self.auto_arm
            )
            if complete:
                if not self.ready_logged:
                    rospy.loginfo(
                        "uav_offboard_manager: vehicle ready, mode=%s armed=%s",
                        self.state.mode,
                        self.state.armed,
                    )
                    self.ready_logged = True
                rate.sleep()
                continue

            self.ready_logged = False
            if (now - self.last_request_time).to_sec() < self.retry_interval:
                rate.sleep()
                continue
            self.last_request_time = now
            if self.state.mode != self.offboard_mode:
                self.request_mode()
            elif self.auto_arm and not self.state.armed:
                ready, reason = self.auto_arm_checks(now)
                if ready:
                    self.request_arm()
                else:
                    rospy.logwarn_throttle(
                        5.0, "uav_offboard_manager: auto-arm blocked: %s", reason
                    )
            rate.sleep()


if __name__ == "__main__":
    rospy.init_node("uav_offboard_manager")
    UavOffboardManager().spin()
