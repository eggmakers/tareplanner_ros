#!/usr/bin/env python3
import json
import math
import os
import threading

import rospy
from geometry_msgs.msg import PointStamped, PoseStamped, Quaternion, TwistStamped
from mavros_msgs.msg import RCIn, State
from mavros_msgs.srv import SetMode
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool


class UavWaypointBridge:
    def __init__(self):
        self.input_topic = rospy.get_param("~input_waypoint_topic", "/way_point")
        self.output_topic = rospy.get_param(
            "~output_pose_topic", "/mavros/setpoint_position/local"
        )
        self.odometry_topic = rospy.get_param(
            "~odometry_topic", "/mavros/local_position/odom"
        )
        self.state_topic = rospy.get_param("~state_topic", "/mavros/state")
        self.command_velocity_topic = rospy.get_param(
            "~command_velocity_topic", "/tare_uav/command_velocity"
        )
        self.frame_id = rospy.get_param("~frame_id", "map")
        self.yaw_from_motion = rospy.get_param("~yaw_from_motion", True)
        self.default_yaw = float(rospy.get_param("~default_yaw", 0.0))
        self.publish_rate = float(rospy.get_param("~publish_rate", 20.0))
        self.override_z = rospy.get_param("~override_z", False)
        self.fixed_z = float(rospy.get_param("~fixed_z", 1.5))
        self.fixed_z_relative_to_start = rospy.get_param(
            "~fixed_z_relative_to_start", False
        )

        self.smoothing_enabled = rospy.get_param("~smoothing_enabled", True)
        self.max_xy_speed = float(rospy.get_param("~max_xy_speed", 0.35))
        self.max_z_speed = float(rospy.get_param("~max_z_speed", 0.35))
        self.max_xy_accel = float(rospy.get_param("~max_xy_accel", 0.20))
        self.max_z_accel = float(rospy.get_param("~max_z_accel", 0.25))
        self.max_xy_jerk = float(rospy.get_param("~max_xy_jerk", 0.6))
        self.max_z_jerk = float(rospy.get_param("~max_z_jerk", 0.5))
        self.max_yaw_rate = float(rospy.get_param("~max_yaw_rate", 0.35))
        self.heading_alignment_enabled = rospy.get_param(
            "~heading_alignment_enabled", True
        )
        self.heading_alignment_tolerance = float(
            rospy.get_param("~heading_alignment_tolerance", 0.26)
        )
        self.motion_yaw_min_distance = float(
            rospy.get_param("~motion_yaw_min_distance", 0.25)
        )
        self.xy_arrival_radius = float(rospy.get_param("~xy_arrival_radius", 0.08))
        self.z_arrival_radius = float(rospy.get_param("~z_arrival_radius", 0.04))
        self.hold_until_armed = rospy.get_param("~hold_until_armed", True)
        self.takeoff_stabilization_enabled = rospy.get_param(
            "~takeoff_stabilization_enabled", True
        )
        self.takeoff_altitude_tolerance = float(
            rospy.get_param("~takeoff_altitude_tolerance", 0.15)
        )
        self.takeoff_stabilization_seconds = float(
            rospy.get_param("~takeoff_stabilization_seconds", 3.0)
        )
        self.start_exploration_after_takeoff = rospy.get_param(
            "~start_exploration_after_takeoff", True
        )
        self.start_exploration_topic = rospy.get_param(
            "~start_exploration_topic", "/start_exploration"
        )
        self.rc_override_enabled = rospy.get_param("~rc_override_enabled", False)
        self.rc_input_topic = rospy.get_param("~rc_input_topic", "/mavros/rc/in")
        self.rc_override_channel = int(rospy.get_param("~rc_override_channel", 5))
        self.rc_override_pwm = int(rospy.get_param("~rc_override_pwm", 1900))
        self.rc_override_debounce_seconds = float(
            rospy.get_param("~rc_override_debounce_seconds", 0.10)
        )
        self.rc_override_mode = rospy.get_param("~rc_override_mode", "POSCTL")
        self.rc_override_retry_interval = float(
            rospy.get_param("~rc_override_retry_interval", 0.50)
        )
        self.rc_override_status_topic = rospy.get_param(
            "~rc_override_status_topic", "/tare_uav/manual_override"
        )
        self.set_mode_service = rospy.get_param(
            "~set_mode_service", "/mavros/set_mode"
        )

        # This final command-side fence is intentionally independent of TARE's
        # planning boundary. It prevents a stale or unsafe waypoint from ever
        # becoming a PX4 position target.
        self.geofence_enabled = rospy.get_param("~geofence_enabled", False)
        self.geofence_file = rospy.get_param("~geofence_file", "")
        self.geofence_min_x = float(rospy.get_param("~geofence_min_x", -1000.0))
        self.geofence_max_x = float(rospy.get_param("~geofence_max_x", 1000.0))
        self.geofence_min_y = float(rospy.get_param("~geofence_min_y", -1000.0))
        self.geofence_max_y = float(rospy.get_param("~geofence_max_y", 1000.0))
        self._load_geofence_file()

        self._validate_parameters()

        self.lock = threading.Lock()
        self.current_position = None
        self.initial_z = None
        self.current_yaw = None
        self.target_position = None
        self.command_position = None
        self.command_velocity = [0.0, 0.0, 0.0]
        self.command_acceleration = [0.0, 0.0, 0.0]
        self.target_yaw = self.default_yaw
        self.command_yaw = self.default_yaw
        self.last_update = None
        self.vehicle_armed = False
        self.vehicle_mode = ""
        self.takeoff_hold_xy = None
        self.takeoff_hold_yaw = self.default_yaw
        self.takeoff_stable_since = None
        self.takeoff_stabilized = not self.takeoff_stabilization_enabled
        self.exploration_start_sent = False
        self.manual_override_latched = False
        self.rc_override_high_since = None
        self.last_rc_override_request = None

        self.pose_pub = rospy.Publisher(self.output_topic, PoseStamped, queue_size=2)
        self.velocity_pub = rospy.Publisher(
            self.command_velocity_topic, TwistStamped, queue_size=2
        )
        self.exploration_start_pub = rospy.Publisher(
            self.start_exploration_topic, Bool, queue_size=1, latch=True
        )
        self.manual_override_pub = rospy.Publisher(
            self.rc_override_status_topic, Bool, queue_size=1, latch=True
        )
        self.manual_override_pub.publish(Bool(data=False))
        self.waypoint_sub = rospy.Subscriber(
            self.input_topic, PointStamped, self.waypoint_callback, queue_size=2
        )
        self.odom_sub = rospy.Subscriber(
            self.odometry_topic, Odometry, self.odometry_callback, queue_size=5
        )
        self.state_sub = rospy.Subscriber(
            self.state_topic, State, self.state_callback, queue_size=5
        )
        self.rc_sub = None
        self.set_mode_client = None
        if self.rc_override_enabled:
            self.rc_sub = rospy.Subscriber(
                self.rc_input_topic, RCIn, self.rc_input_callback, queue_size=5
            )
            self.set_mode_client = rospy.ServiceProxy(self.set_mode_service, SetMode)
        self.timer = rospy.Timer(
            rospy.Duration(1.0 / self.publish_rate), self.publish_setpoint
        )

        rospy.logwarn(
            "uav_waypoint_bridge safety profile: speed_xy=%.2f speed_z=%.2f "
            "accel_xy=%.2f accel_z=%.2f jerk_xy=%.2f yaw_from_motion=%s "
            "heading_alignment=%s/%.1fdeg "
            "geofence=%s hold_until_armed=%s takeoff_stabilization=%.1fs "
            "delayed_exploration=%s rc_override=%s/ch%d>=%d",
            self.max_xy_speed,
            self.max_z_speed,
            self.max_xy_accel,
            self.max_z_accel,
            self.max_xy_jerk,
            self.yaw_from_motion,
            self.heading_alignment_enabled,
            math.degrees(self.heading_alignment_tolerance),
            self.geofence_enabled,
            self.hold_until_armed,
            self.takeoff_stabilization_seconds,
            self.start_exploration_after_takeoff,
            self.rc_override_enabled,
            self.rc_override_channel,
            self.rc_override_pwm,
        )

        if self.geofence_enabled:
            rospy.logwarn(
                "uav_waypoint_bridge command geofence: x=[%.2f, %.2f], "
                "y=[%.2f, %.2f] map ENU",
                self.geofence_min_x,
                self.geofence_max_x,
                self.geofence_min_y,
                self.geofence_max_y,
            )

    def _load_geofence_file(self):
        if not self.geofence_enabled or not self.geofence_file:
            return
        if not os.path.isfile(self.geofence_file):
            raise ValueError("geofence file does not exist: " + self.geofence_file)
        with open(self.geofence_file, "r", encoding="utf-8") as stream:
            report = json.load(stream)
        bounds = report.get("safety_geofence_enu")
        required = ("min_x", "max_x", "min_y", "max_y")
        if not isinstance(bounds, dict) or not all(key in bounds for key in required):
            raise ValueError(
                "geofence file has no safety_geofence_enu bounds: "
                + self.geofence_file
            )
        self.geofence_min_x = float(bounds["min_x"])
        self.geofence_max_x = float(bounds["max_x"])
        self.geofence_min_y = float(bounds["min_y"])
        self.geofence_max_y = float(bounds["max_y"])

    def _validate_parameters(self):
        if self.publish_rate <= 0.0:
            rospy.logwarn("publish_rate is invalid; using 20 Hz")
            self.publish_rate = 20.0
        positive = {
            "max_xy_speed": self.max_xy_speed,
            "max_z_speed": self.max_z_speed,
            "max_xy_accel": self.max_xy_accel,
            "max_z_accel": self.max_z_accel,
            "max_xy_jerk": self.max_xy_jerk,
            "max_z_jerk": self.max_z_jerk,
            "max_yaw_rate": self.max_yaw_rate,
        }
        invalid = [name for name, value in positive.items() if value <= 0.0]
        if invalid:
            raise ValueError("positive safety limits required: " + ", ".join(invalid))
        if self.takeoff_altitude_tolerance <= 0.0:
            raise ValueError("takeoff_altitude_tolerance must be positive")
        if self.takeoff_stabilization_seconds < 0.0:
            raise ValueError("takeoff_stabilization_seconds cannot be negative")
        if not 0.0 < self.heading_alignment_tolerance <= math.pi:
            raise ValueError("heading_alignment_tolerance must be in (0, pi]")
        if self.motion_yaw_min_distance < 0.0:
            raise ValueError("motion_yaw_min_distance cannot be negative")
        if self.rc_override_channel < 1:
            raise ValueError("rc_override_channel is one-based and must be positive")
        if not 800 <= self.rc_override_pwm <= 2200:
            raise ValueError("rc_override_pwm must be between 800 and 2200")
        if self.rc_override_debounce_seconds < 0.0:
            raise ValueError("rc_override_debounce_seconds cannot be negative")
        if self.rc_override_retry_interval <= 0.0:
            raise ValueError("rc_override_retry_interval must be positive")
        if not self.rc_override_mode:
            raise ValueError("rc_override_mode cannot be empty")
        if self.geofence_enabled and (
            self.geofence_min_x >= self.geofence_max_x
            or self.geofence_min_y >= self.geofence_max_y
        ):
            raise ValueError("geofence minimums must be smaller than maximums")

    @staticmethod
    def yaw_to_quaternion(yaw):
        half_yaw = yaw * 0.5
        return Quaternion(0.0, 0.0, math.sin(half_yaw), math.cos(half_yaw))

    @staticmethod
    def quaternion_to_yaw(quaternion):
        sin_yaw = 2.0 * (
            quaternion.w * quaternion.z + quaternion.x * quaternion.y
        )
        cos_yaw = 1.0 - 2.0 * (
            quaternion.y * quaternion.y + quaternion.z * quaternion.z
        )
        return math.atan2(sin_yaw, cos_yaw)

    @staticmethod
    def wrap_angle(angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    @staticmethod
    def limit_vector(x_value, y_value, maximum):
        magnitude = math.hypot(x_value, y_value)
        if magnitude <= maximum or magnitude < 1e-9:
            return x_value, y_value
        scale = maximum / magnitude
        return x_value * scale, y_value * scale

    @staticmethod
    def limit_scalar(value, maximum):
        return max(-maximum, min(maximum, value))

    def odometry_callback(self, msg):
        position = msg.pose.pose.position
        with self.lock:
            self.current_position = [position.x, position.y, position.z]
            if self.initial_z is None:
                self.initial_z = position.z
            self.current_yaw = self.quaternion_to_yaw(msg.pose.pose.orientation)
            if self.command_position is None:
                self.command_position = list(self.current_position)
                self.target_position = list(self.current_position)
                initial_yaw = self.current_yaw
                self.command_yaw = initial_yaw
                if self.yaw_from_motion:
                    self.target_yaw = initial_yaw

    def state_callback(self, msg):
        with self.lock:
            self.vehicle_mode = msg.mode
            if (
                msg.armed
                and not self.vehicle_armed
                and self.current_position is not None
                and not self.manual_override_latched
            ):
                self.command_position = list(self.current_position)
                self.command_velocity = [0.0, 0.0, 0.0]
                self.command_acceleration = [0.0, 0.0, 0.0]
                self.last_update = None
                self.takeoff_hold_xy = list(self.current_position[:2])
                self.takeoff_hold_yaw = self.command_yaw
                self.takeoff_stable_since = None
                self.takeoff_stabilized = not self.takeoff_stabilization_enabled
                self.target_position = [
                    self.current_position[0],
                    self.current_position[1],
                    self.resolved_fixed_z(),
                ]
            elif not msg.armed:
                self.takeoff_hold_xy = None
                self.takeoff_stable_since = None
                self.takeoff_stabilized = not self.takeoff_stabilization_enabled
            self.vehicle_armed = bool(msg.armed)

    def rc_override_triggered(self, channels, now_seconds):
        if self.manual_override_latched:
            return False
        channel_index = self.rc_override_channel - 1
        if channel_index >= len(channels):
            self.rc_override_high_since = None
            return False
        if channels[channel_index] < self.rc_override_pwm:
            self.rc_override_high_since = None
            return False
        if self.rc_override_high_since is None:
            self.rc_override_high_since = now_seconds
        return (
            now_seconds - self.rc_override_high_since
            >= self.rc_override_debounce_seconds
        )

    def activate_manual_override(self):
        with self.lock:
            if self.manual_override_latched:
                return False
            self.manual_override_latched = True
            hold_position = self.current_position or self.command_position
            if hold_position is not None:
                self.command_position = list(hold_position)
                self.target_position = list(hold_position)
            self.command_velocity = [0.0, 0.0, 0.0]
            self.command_acceleration = [0.0, 0.0, 0.0]
            if self.current_yaw is not None:
                self.command_yaw = self.current_yaw
                self.target_yaw = self.current_yaw

        self.manual_override_pub.publish(Bool(data=True))
        rospy.logerr(
            "uav_waypoint_bridge: RC manual override latched; holding position "
            "and requesting %s. Restart only after landing.",
            self.rc_override_mode,
        )
        return True

    def rc_input_callback(self, msg):
        now = rospy.Time.now()
        if self.rc_override_triggered(msg.channels, now.to_sec()):
            self.activate_manual_override()
            self.request_rc_override_mode(now)

    def request_rc_override_mode(self, now):
        if (
            not self.rc_override_enabled
            or not self.manual_override_latched
            or self.vehicle_mode != "OFFBOARD"
            or self.set_mode_client is None
        ):
            return
        if (
            self.last_rc_override_request is not None
            and (now - self.last_rc_override_request).to_sec()
            < self.rc_override_retry_interval
        ):
            return
        self.last_rc_override_request = now
        try:
            response = self.set_mode_client(
                base_mode=0, custom_mode=self.rc_override_mode
            )
            if not response.mode_sent:
                rospy.logerr_throttle(
                    1.0,
                    "uav_waypoint_bridge: PX4 rejected RC override mode %s",
                    self.rc_override_mode,
                )
        except rospy.ServiceException as exc:
            rospy.logerr_throttle(
                1.0, "uav_waypoint_bridge: RC override mode request failed: %s", exc
            )

    def clamp_to_geofence(self, target):
        if not self.geofence_enabled:
            return list(target), False
        clamped = list(target)
        clamped[0] = max(self.geofence_min_x, min(self.geofence_max_x, target[0]))
        clamped[1] = max(self.geofence_min_y, min(self.geofence_max_y, target[1]))
        changed = clamped[0] != target[0] or clamped[1] != target[1]
        return clamped, changed

    def waypoint_callback(self, msg):
        target = [msg.point.x, msg.point.y, msg.point.z]
        if self.override_z:
            target[2] = self.resolved_fixed_z()
        if not all(math.isfinite(value) for value in target):
            rospy.logwarn_throttle(2.0, "uav_waypoint_bridge rejected non-finite waypoint")
            return

        raw_target = list(target)
        target, was_clamped = self.clamp_to_geofence(target)
        if was_clamped:
            rospy.logwarn_throttle(
                2.0,
                "uav_waypoint_bridge clamped unsafe waypoint "
                "(%.2f, %.2f) -> (%.2f, %.2f)",
                raw_target[0],
                raw_target[1],
                target[0],
                target[1],
            )

        with self.lock:
            if self.manual_override_latched:
                return
            self.target_position = target
            if self.yaw_from_motion:
                self.update_motion_yaw_target()
            else:
                self.target_yaw = self.default_yaw

    def resolved_fixed_z(self):
        if not getattr(self, "fixed_z_relative_to_start", False):
            return self.fixed_z
        initial_z = getattr(self, "initial_z", None)
        return self.fixed_z if initial_z is None else initial_z + self.fixed_z

    def update_horizontal_profile(self, dt, target_xy=None):
        target_x, target_y = (
            self.target_position[:2] if target_xy is None else target_xy
        )
        error_x = target_x - self.command_position[0]
        error_y = target_y - self.command_position[1]
        distance = math.hypot(error_x, error_y)
        if distance <= self.xy_arrival_radius:
            self.command_position[0] = target_x
            self.command_position[1] = target_y
            self.command_velocity[0] = 0.0
            self.command_velocity[1] = 0.0
            self.command_acceleration[0] = 0.0
            self.command_acceleration[1] = 0.0
            return

        braking_speed = math.sqrt(2.0 * self.max_xy_accel * distance)
        desired_speed = min(self.max_xy_speed, braking_speed)
        desired_vx = desired_speed * error_x / distance
        desired_vy = desired_speed * error_y / distance

        requested_ax = (desired_vx - self.command_velocity[0]) / dt
        requested_ay = (desired_vy - self.command_velocity[1]) / dt
        requested_ax, requested_ay = self.limit_vector(
            requested_ax, requested_ay, self.max_xy_accel
        )

        delta_ax = requested_ax - self.command_acceleration[0]
        delta_ay = requested_ay - self.command_acceleration[1]
        delta_ax, delta_ay = self.limit_vector(
            delta_ax, delta_ay, self.max_xy_jerk * dt
        )
        self.command_acceleration[0] += delta_ax
        self.command_acceleration[1] += delta_ay

        self.command_velocity[0] += self.command_acceleration[0] * dt
        self.command_velocity[1] += self.command_acceleration[1] * dt
        self.command_velocity[0], self.command_velocity[1] = self.limit_vector(
            self.command_velocity[0], self.command_velocity[1], self.max_xy_speed
        )
        self.command_position[0] += self.command_velocity[0] * dt
        self.command_position[1] += self.command_velocity[1] * dt
        next_error_x = target_x - self.command_position[0]
        next_error_y = target_y - self.command_position[1]
        if error_x * next_error_x + error_y * next_error_y <= 0.0:
            self.command_position[0] = target_x
            self.command_position[1] = target_y
            self.command_velocity[0] = 0.0
            self.command_velocity[1] = 0.0
            self.command_acceleration[0] = 0.0
            self.command_acceleration[1] = 0.0

    def update_vertical_profile(self, dt):
        error = self.target_position[2] - self.command_position[2]
        if abs(error) <= self.z_arrival_radius:
            self.command_position[2] = self.target_position[2]
            self.command_velocity[2] = 0.0
            self.command_acceleration[2] = 0.0
            return

        braking_speed = math.sqrt(2.0 * self.max_z_accel * abs(error))
        desired_velocity = math.copysign(
            min(self.max_z_speed, braking_speed), error
        )
        requested_acceleration = self.limit_scalar(
            (desired_velocity - self.command_velocity[2]) / dt, self.max_z_accel
        )
        acceleration_delta = self.limit_scalar(
            requested_acceleration - self.command_acceleration[2],
            self.max_z_jerk * dt,
        )
        self.command_acceleration[2] += acceleration_delta
        self.command_velocity[2] = self.limit_scalar(
            self.command_velocity[2] + self.command_acceleration[2] * dt,
            self.max_z_speed,
        )
        self.command_position[2] += self.command_velocity[2] * dt
        next_error = self.target_position[2] - self.command_position[2]
        if error * next_error <= 0.0:
            self.command_position[2] = self.target_position[2]
            self.command_velocity[2] = 0.0
            self.command_acceleration[2] = 0.0

    def update_yaw_profile(self, dt):
        yaw_error = self.wrap_angle(self.target_yaw - self.command_yaw)
        yaw_step = self.limit_scalar(yaw_error, self.max_yaw_rate * dt)
        self.command_yaw = self.wrap_angle(self.command_yaw + yaw_step)

    def update_motion_yaw_target(self):
        if self.command_position is None or self.target_position is None:
            return False
        dx = self.target_position[0] - self.command_position[0]
        dy = self.target_position[1] - self.command_position[1]
        if math.hypot(dx, dy) <= self.motion_yaw_min_distance:
            return False
        self.target_yaw = math.atan2(dy, dx)
        return True

    def heading_is_aligned(self):
        measured_yaw = (
            self.current_yaw if self.current_yaw is not None else self.command_yaw
        )
        error = self.wrap_angle(self.target_yaw - measured_yaw)
        return abs(error) <= self.heading_alignment_tolerance

    def hold_horizontal_profile(self):
        self.command_velocity[0] = 0.0
        self.command_velocity[1] = 0.0
        self.command_acceleration[0] = 0.0
        self.command_acceleration[1] = 0.0

    def update_smoothed_exploration_profile(self, dt):
        motion_heading_available = False
        if self.yaw_from_motion:
            motion_heading_available = self.update_motion_yaw_target()
        self.update_yaw_profile(dt)
        if (
            self.yaw_from_motion
            and self.heading_alignment_enabled
            and motion_heading_available
            and not self.heading_is_aligned()
        ):
            self.hold_horizontal_profile()
        else:
            self.update_horizontal_profile(dt)
        self.update_vertical_profile(dt)

    def takeoff_hold_active(self, now):
        if (
            not self.takeoff_stabilization_enabled
            or not self.vehicle_armed
            or self.takeoff_stabilized
        ):
            return False
        if self.takeoff_hold_xy is None:
            self.takeoff_hold_xy = list(self.current_position[:2])
            self.takeoff_hold_yaw = self.command_yaw
        altitude_error = abs(self.current_position[2] - self.target_position[2])
        if altitude_error <= self.takeoff_altitude_tolerance:
            if self.takeoff_stable_since is None:
                self.takeoff_stable_since = now
            elif (
                now - self.takeoff_stable_since
            ).to_sec() >= self.takeoff_stabilization_seconds:
                self.takeoff_stabilized = True
                rospy.loginfo(
                    "uav_waypoint_bridge: takeoff height stable; releasing XY and yaw exploration"
                )
                return False
        else:
            self.takeoff_stable_since = None
        return True

    def publish_exploration_start_if_ready(self):
        if (
            not self.start_exploration_after_takeoff
            or self.exploration_start_sent
            or not self.vehicle_armed
            or not self.takeoff_stabilized
            or self.manual_override_latched
        ):
            return
        self.exploration_start_pub.publish(Bool(data=True))
        self.exploration_start_sent = True
        rospy.loginfo(
            "uav_waypoint_bridge: UAV stabilized; starting TARE scan processing"
        )

    def publish_setpoint(self, _event):
        now = rospy.Time.now()
        with self.lock:
            if self.command_position is None or self.target_position is None:
                self.last_update = now
                return
            if self.last_update is None:
                dt = 1.0 / self.publish_rate
            else:
                dt = max(0.001, min(0.2, (now - self.last_update).to_sec()))
            self.last_update = now
            takeoff_hold = (
                False
                if self.manual_override_latched
                else self.takeoff_hold_active(now)
            )
            self.publish_exploration_start_if_ready()

            if self.manual_override_latched:
                self.command_velocity = [0.0, 0.0, 0.0]
                self.command_acceleration = [0.0, 0.0, 0.0]
            elif self.hold_until_armed and not self.vehicle_armed:
                self.command_position = list(self.current_position)
                self.command_velocity = [0.0, 0.0, 0.0]
                self.command_acceleration = [0.0, 0.0, 0.0]
            elif takeoff_hold and self.smoothing_enabled:
                self.update_horizontal_profile(dt, self.takeoff_hold_xy)
                self.update_vertical_profile(dt)
                self.command_yaw = self.takeoff_hold_yaw
            elif takeoff_hold:
                self.command_position = [
                    self.takeoff_hold_xy[0],
                    self.takeoff_hold_xy[1],
                    self.target_position[2],
                ]
                self.command_velocity = [0.0, 0.0, 0.0]
                self.command_acceleration = [0.0, 0.0, 0.0]
                self.command_yaw = self.takeoff_hold_yaw
            elif self.smoothing_enabled:
                self.update_smoothed_exploration_profile(dt)
            else:
                self.command_position = list(self.target_position)
                self.command_velocity = [0.0, 0.0, 0.0]
                self.command_acceleration = [0.0, 0.0, 0.0]
                self.command_yaw = self.target_yaw

            command_position = list(self.command_position)
            command_velocity = list(self.command_velocity)
            command_yaw = self.command_yaw

        self.request_rc_override_mode(now)

        pose = PoseStamped()
        pose.header.stamp = now
        pose.header.frame_id = self.frame_id
        pose.pose.position.x = command_position[0]
        pose.pose.position.y = command_position[1]
        pose.pose.position.z = command_position[2]
        pose.pose.orientation = self.yaw_to_quaternion(command_yaw)
        self.pose_pub.publish(pose)

        velocity = TwistStamped()
        velocity.header = pose.header
        velocity.twist.linear.x = command_velocity[0]
        velocity.twist.linear.y = command_velocity[1]
        velocity.twist.linear.z = command_velocity[2]
        self.velocity_pub.publish(velocity)


if __name__ == "__main__":
    rospy.init_node("uav_waypoint_bridge")
    UavWaypointBridge()
    rospy.spin()
