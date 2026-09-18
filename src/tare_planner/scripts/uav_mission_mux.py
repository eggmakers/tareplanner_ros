#!/usr/bin/env python3

import threading

import rospy
from geometry_msgs.msg import Point, PointStamped, PoseStamped
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger, TriggerResponse


class UavMissionMux:
    MODES = ("tare", "fixed_start", "saved_map", "replay", "hold")

    def __init__(self):
        self.lock = threading.RLock()
        self.frame_id = rospy.get_param("~frame_id", "map")
        self.mode = rospy.get_param("~default_mode", "tare").lower()
        if self.mode not in self.MODES:
            self.mode = "hold"
        self.fixed_height = float(rospy.get_param("~fixed_flight_height", 1.5))
        self.relative_height = bool(rospy.get_param(
            "~fixed_flight_height_relative_to_start", False))
        self.robot_position = None
        self.start_z = None
        self.manual_override = False
        self.emergency_latched = False
        self.replay_active = False
        self.sources = {
            "tare": {"waypoint": None, "path": None},
            "saved_map": {"waypoint": None, "path": None},
            "replay": {"waypoint": None, "path": None},
        }

        self.output_waypoint_pub = rospy.Publisher(
            rospy.get_param("~output_waypoint_topic",
                            "/tare_uav/command/waypoint"),
            PointStamped, queue_size=2)
        self.output_path_pub = rospy.Publisher(
            rospy.get_param("~output_path_topic",
                            "/tare_uav/command/reference_path"),
            Path, queue_size=1, latch=True)
        self.mode_pub = rospy.Publisher(
            rospy.get_param("~selected_mode_topic",
                            "/tare_uav/mission/selected"),
            String, queue_size=1, latch=True)
        self.status_pub = rospy.Publisher(
            rospy.get_param("~status_topic", "/tare_uav/mission/status"),
            String, queue_size=1, latch=True)
        self.tare_pause_pub = rospy.Publisher(
            rospy.get_param("~tare_pause_topic", "/tare_uav/mission/pause"),
            Bool, queue_size=1, latch=True)
        self.tare_resume_pub = rospy.Publisher(
            rospy.get_param("~tare_resume_topic",
                            "/tare_uav/mission/resume_exploration"),
            Bool, queue_size=1)
        self.tare_goal_pub = rospy.Publisher(
            rospy.get_param("~tare_goal_topic", "/tare_uav/source/tare/goal"),
            PoseStamped, queue_size=1)
        self.fixed_start_goal_pub = rospy.Publisher(
            rospy.get_param("~fixed_start_goal_topic",
                            "/tare_uav/source/fixed_start/goal"),
            PoseStamped, queue_size=1)
        self.saved_map_goal_pub = rospy.Publisher(
            rospy.get_param("~saved_map_goal_topic",
                            "/tare_uav/source/saved_map/goal"),
            PoseStamped, queue_size=1)

        self.subscribe_source(
            "tare",
            rospy.get_param("~tare_waypoint_topic",
                            "/tare_uav/source/tare/waypoint"),
            rospy.get_param("~tare_path_topic",
                            "/sensor_coverage_planner/exploration_path"))
        self.subscribe_source(
            "saved_map",
            rospy.get_param("~saved_map_waypoint_topic",
                            "/tare_uav/source/saved_map/waypoint"),
            rospy.get_param("~saved_map_path_topic",
                            "/tare_uav/source/saved_map/path"))
        self.subscribe_source(
            "replay",
            rospy.get_param("~replay_waypoint_topic",
                            "/tare_uav/source/replay/waypoint"),
            rospy.get_param("~replay_path_topic",
                            "/tare_uav/source/replay/path"))

        rospy.Subscriber(
            rospy.get_param("~odometry_topic", "/state_estimation"),
            Odometry, self.odometry_callback, queue_size=10)
        rospy.Subscriber(
            rospy.get_param("~rviz_goal_topic", "/move_base_simple/goal"),
            PoseStamped, self.goal_callback, queue_size=1)
        rospy.Subscriber(
            rospy.get_param("~manual_override_topic",
                            "/tare_uav/manual_override"),
            Bool, self.manual_override_callback, queue_size=1)
        rospy.Subscriber(
            rospy.get_param("~replay_active_topic", "/tare_uav/replay/active"),
            Bool, self.replay_active_callback, queue_size=1)
        rospy.Subscriber(
            rospy.get_param("~select_topic", "/tare_uav/mission/select"),
            String, self.select_topic_callback, queue_size=1)

        rospy.Service("~select_tare", Trigger,
                      lambda request: self.select_service("tare"))
        rospy.Service("~select_fixed_start", Trigger,
                      lambda request: self.select_service("fixed_start"))
        rospy.Service("~select_saved_map", Trigger,
                      lambda request: self.select_service("saved_map"))
        rospy.Service("~select_replay", Trigger,
                      lambda request: self.select_service("replay"))
        rospy.Service("~hold", Trigger,
                      lambda request: self.select_service("hold"))
        rospy.Service("~clear_emergency", Trigger, self.clear_emergency_service)

        rate = float(rospy.get_param("~publish_rate", 10.0))
        rospy.Timer(rospy.Duration(1.0 / max(1.0, rate)), self.timer_callback)
        rospy.Timer(rospy.Duration(0.5), self.initial_mode_timer, oneshot=True)

    def subscribe_source(self, name, waypoint_topic, path_topic):
        rospy.Subscriber(
            waypoint_topic, PointStamped,
            lambda message, source=name: self.waypoint_callback(source, message),
            queue_size=2)
        rospy.Subscriber(
            path_topic, Path,
            lambda message, source=name: self.path_callback(source, message),
            queue_size=1)

    def waypoint_callback(self, source, message):
        with self.lock:
            self.sources[source]["waypoint"] = message

    def path_callback(self, source, message):
        with self.lock:
            self.sources[source]["path"] = message

    def odometry_callback(self, message):
        with self.lock:
            self.robot_position = message.pose.pose.position
            if self.start_z is None:
                self.start_z = self.robot_position.z

    def replay_active_callback(self, message):
        with self.lock:
            self.replay_active = message.data

    def manual_override_callback(self, message):
        with self.lock:
            self.manual_override = message.data
            if message.data:
                self.emergency_latched = True
                self.set_status("EMERGENCY_HOLD: RC/manual override latched")

    def flight_z(self):
        if self.relative_height and self.start_z is not None:
            return self.start_z + self.fixed_height
        return self.fixed_height

    def hold_waypoint(self):
        if self.robot_position is None:
            return None
        message = PointStamped()
        message.header.frame_id = self.frame_id
        message.header.stamp = rospy.Time.now()
        message.point = Point(x=self.robot_position.x,
                              y=self.robot_position.y,
                              z=self.flight_z())
        return message

    def hold_path(self, waypoint):
        message = Path()
        message.header = waypoint.header
        pose = PoseStamped()
        pose.header = waypoint.header
        pose.pose.position = waypoint.point
        pose.pose.orientation.w = 1.0
        message.poses.append(pose)
        return message

    def set_status(self, text):
        rospy.logwarn("[uav_mission_mux] %s", text)
        self.status_pub.publish(String(data=text))

    def initial_mode_timer(self, _event):
        with self.lock:
            self.apply_mode_side_effects()

    def apply_mode_side_effects(self):
        self.mode_pub.publish(String(data=self.mode))
        if self.mode == "tare":
            self.tare_pause_pub.publish(Bool(data=False))
            self.tare_resume_pub.publish(Bool(data=True))
        else:
            self.tare_pause_pub.publish(Bool(data=True))
        self.set_status("Selected mission mode: %s" % self.mode.upper())

    def select(self, mode):
        if mode not in self.MODES:
            return False, "Unknown mode '%s'. Valid modes: %s" % (
                mode, ", ".join(self.MODES))
        if self.emergency_latched and mode != "hold":
            return False, "Emergency hold is latched; clear it before selecting a mission."
        self.mode = mode
        self.apply_mode_side_effects()
        return True, "Selected UAV mission mode: %s" % mode

    def select_service(self, mode):
        with self.lock:
            success, message = self.select(mode)
            return TriggerResponse(success, message)

    def select_topic_callback(self, message):
        with self.lock:
            success, text = self.select(message.data.strip().lower())
            if not success:
                self.set_status("MODE_REJECTED: " + text)

    def clear_emergency_service(self, _request):
        with self.lock:
            if self.manual_override:
                return TriggerResponse(
                    False, "RC/manual override is still active; reset the switch first.")
            self.emergency_latched = False
            self.mode = "hold"
            self.apply_mode_side_effects()
            return TriggerResponse(True, "Emergency latch cleared; UAV remains in HOLD.")

    def goal_callback(self, message):
        with self.lock:
            if self.emergency_latched:
                self.set_status("Goal ignored while emergency hold is latched.")
                return
            if self.mode == "tare":
                self.tare_goal_pub.publish(message)
                self.set_status("RViz goal routed to live TARE graph navigation.")
            elif self.mode == "fixed_start":
                self.fixed_start_goal_pub.publish(message)
                self.set_status("RViz goal routed to fixed-start planning (no flight command).")
            elif self.mode == "saved_map":
                self.saved_map_goal_pub.publish(message)
                self.set_status("RViz goal routed to saved-map A* planning.")
            else:
                self.set_status("RViz goal ignored in %s mode." % self.mode.upper())

    def timer_callback(self, _event):
        with self.lock:
            if self.robot_position is None:
                return
            if self.emergency_latched or self.mode in ("hold", "fixed_start"):
                waypoint = self.hold_waypoint()
                self.output_waypoint_pub.publish(waypoint)
                self.output_path_pub.publish(self.hold_path(waypoint))
                return

            if self.mode == "replay" and not self.replay_active:
                waypoint = self.hold_waypoint()
                self.output_waypoint_pub.publish(waypoint)
                self.output_path_pub.publish(self.hold_path(waypoint))
                return

            source = self.sources[self.mode]
            if source["waypoint"] is None:
                waypoint = self.hold_waypoint()
                self.output_waypoint_pub.publish(waypoint)
                self.output_path_pub.publish(self.hold_path(waypoint))
                return

            waypoint = source["waypoint"]
            waypoint.header.stamp = rospy.Time.now()
            waypoint.header.frame_id = waypoint.header.frame_id or self.frame_id
            waypoint.point.z = self.flight_z()
            self.output_waypoint_pub.publish(waypoint)
            if source["path"] is not None:
                source["path"].header.stamp = waypoint.header.stamp
                for pose in source["path"].poses:
                    pose.header = source["path"].header
                    pose.pose.position.z = self.flight_z()
                self.output_path_pub.publish(source["path"])


if __name__ == "__main__":
    rospy.init_node("uav_mission_mux")
    UavMissionMux()
    rospy.spin()
