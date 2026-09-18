#!/usr/bin/env python3

import hashlib
import json
import math
import os
import threading

import rospy
from geometry_msgs.msg import Point, PointStamped, PoseStamped
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger, TriggerResponse


def yaw_from_quaternion(quaternion):
    sin_yaw = 2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y)
    cos_yaw = 1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z)
    return math.atan2(sin_yaw, cos_yaw)


def canonical_sha256(value):
    payload = json.dumps(value, sort_keys=True, separators=(",", ":"),
                         ensure_ascii=True).encode("utf-8")
    return hashlib.sha256(payload).hexdigest()


class UavPathReplayer:
    def __init__(self):
        self.lock = threading.RLock()
        bundle_dir = os.path.abspath(os.path.expanduser(
            rospy.get_param("~bundle_dir", "~/tare_uav_handoff")))
        self.route_file = os.path.abspath(os.path.expanduser(
            rospy.get_param("~route_file",
                            os.path.join(bundle_dir, "uav_route.json"))))
        self.frame_id = rospy.get_param("~frame_id", "map").lstrip("/")
        self.fixed_height = float(rospy.get_param("~fixed_flight_height", 1.5))
        self.relative_height = bool(rospy.get_param(
            "~fixed_flight_height_relative_to_start", False))
        self.strict_height_match = bool(rospy.get_param(
            "~strict_height_match", True))
        self.height_match_tolerance = float(rospy.get_param(
            "~height_match_tolerance", 0.01))
        self.lookahead_distance = float(rospy.get_param(
            "~lookahead_distance", 1.5))
        self.arrival_distance = float(rospy.get_param(
            "~arrival_distance", 0.35))
        self.arrival_stable_seconds = float(rospy.get_param(
            "~arrival_stable_seconds", 2.0))
        self.start_delay = float(rospy.get_param("~start_delay", 2.0))
        self.progress_timeout = float(rospy.get_param(
            "~progress_timeout", 20.0))
        self.min_progress_distance = float(rospy.get_param(
            "~min_progress_distance", 0.15))
        self.odometry_timeout = float(rospy.get_param(
            "~odometry_timeout", 1.0))

        self.relative_points = []
        self.world_points = []
        self.robot_position = None
        self.robot_yaw = None
        self.mission_start_pose = None
        self.last_odometry_time = rospy.Time(0)
        self.active = False
        self.path_index = 0
        self.motion_enable_time = rospy.Time(0)
        self.last_progress_position = None
        self.last_progress_time = rospy.Time(0)
        self.arrival_candidate_time = None

        self.waypoint_pub = rospy.Publisher(
            rospy.get_param("~waypoint_topic",
                            "/tare_uav/source/replay/waypoint"),
            PointStamped, queue_size=2)
        self.path_pub = rospy.Publisher(
            rospy.get_param("~path_topic", "/tare_uav/source/replay/path"),
            Path, queue_size=1, latch=True)
        self.status_pub = rospy.Publisher(
            rospy.get_param("~status_topic", "/tare_uav/replay/status"),
            String, queue_size=1, latch=True)
        self.active_pub = rospy.Publisher(
            rospy.get_param("~active_topic", "/tare_uav/replay/active"),
            Bool, queue_size=1, latch=True)

        rospy.Subscriber(
            rospy.get_param("~odometry_topic", "/state_estimation"),
            Odometry, self.odometry_callback, queue_size=10)
        rospy.Subscriber(
            rospy.get_param("~start_pose_topic",
                            "/tare_uav/exploration_start_pose"),
            PoseStamped, self.start_pose_callback, queue_size=1)
        rospy.Subscriber(
            rospy.get_param("~manual_override_topic",
                            "/tare_uav/manual_override"),
            Bool, self.manual_override_callback, queue_size=1)

        rospy.Service("~start", Trigger, self.start_callback)
        rospy.Service("~stop", Trigger, self.stop_callback)
        rospy.Service("~reload", Trigger, self.reload_callback)
        self.load_route()
        control_rate = float(rospy.get_param("~control_rate", 10.0))
        rospy.Timer(rospy.Duration(1.0 / max(1.0, control_rate)),
                    self.control_timer_callback)

    @staticmethod
    def copy_point(point):
        return Point(x=point.x, y=point.y, z=point.z)

    @staticmethod
    def distance_xy(point_a, point_b):
        return math.hypot(point_a.x - point_b.x, point_a.y - point_b.y)

    def set_status(self, text):
        rospy.loginfo("[uav_path_replayer] %s", text)
        self.status_pub.publish(String(data=text))
        self.active_pub.publish(Bool(data=self.active))

    def validate_height_policy(self, document):
        policy = document.get("height_policy", {})
        route_relative = bool(policy.get("relative_to_start", False))
        route_height = float(policy.get("fixed_flight_height_m", float("nan")))
        if not math.isfinite(route_height):
            raise ValueError("route height policy is missing")
        if self.strict_height_match:
            if route_relative != self.relative_height:
                raise ValueError("route and launch use different relative-height policies")
            if abs(route_height - self.fixed_height) > self.height_match_tolerance:
                raise ValueError(
                    "route height %.3f m differs from fixed_flight_height %.3f m" %
                    (route_height, self.fixed_height))

    def load_route(self):
        with self.lock:
            self.active = False
            self.relative_points = []
            self.world_points = []
            try:
                with open(self.route_file, "r", encoding="utf-8") as route_stream:
                    document = json.load(route_stream)
                if document.get("schema_version") != 2:
                    raise ValueError("unsupported schema_version")
                if document.get("coordinate_system") != "start_relative_xy_yaw":
                    raise ValueError("unsupported coordinate_system")
                self.validate_height_policy(document)
                points = document.get("points", [])
                if len(points) < 2:
                    raise ValueError("route contains fewer than two points")
                route_payload = {
                    "coordinate_system": document["coordinate_system"],
                    "height_policy": document["height_policy"],
                    "points": points,
                }
                if canonical_sha256(route_payload) != document.get("payload_sha256"):
                    raise ValueError("route checksum mismatch")
                for item in points:
                    values = (float(item["x"]), float(item["y"]),
                              float(item.get("z", 0.0)))
                    if not all(math.isfinite(value) for value in values):
                        raise ValueError("route contains a non-finite coordinate")
                    self.relative_points.append(
                        Point(x=values[0], y=values[1], z=values[2]))
            except (OSError, ValueError, KeyError, TypeError) as error:
                self.set_status("Route load failed: %s" % error)
                return False

            self.set_status(
                "Loaded %d route poses; call /uav_path_replayer/start after selecting replay mode." %
                len(self.relative_points))
            return True

    def odometry_callback(self, message):
        with self.lock:
            self.robot_position = self.copy_point(message.pose.pose.position)
            self.robot_yaw = yaw_from_quaternion(message.pose.pose.orientation)
            self.last_odometry_time = rospy.Time.now()
            if self.mission_start_pose is None:
                self.mission_start_pose = message.pose.pose

    def start_pose_callback(self, message):
        frame = message.header.frame_id.lstrip("/")
        if frame and frame != self.frame_id:
            return
        with self.lock:
            self.mission_start_pose = message.pose

    def manual_override_callback(self, message):
        if not message.data:
            return
        with self.lock:
            if self.active:
                self.stop("Replay stopped: RC/manual override is active.")

    def flight_z(self):
        if self.relative_height:
            return self.mission_start_pose.position.z + self.fixed_height
        return self.fixed_height

    def transform_route(self):
        start = self.mission_start_pose
        start_yaw = yaw_from_quaternion(start.orientation)
        cos_yaw = math.cos(start_yaw)
        sin_yaw = math.sin(start_yaw)
        target_z = self.flight_z()
        self.world_points = []
        for relative in self.relative_points:
            self.world_points.append(Point(
                x=start.position.x + cos_yaw * relative.x - sin_yaw * relative.y,
                y=start.position.y + sin_yaw * relative.x + cos_yaw * relative.y,
                z=target_z))

    def publish_path(self):
        message = Path()
        message.header.frame_id = self.frame_id
        message.header.stamp = rospy.Time.now()
        for point in self.world_points:
            pose = PoseStamped()
            pose.header = message.header
            pose.pose.position = self.copy_point(point)
            pose.pose.orientation.w = 1.0
            message.poses.append(pose)
        self.path_pub.publish(message)

    def publish_waypoint(self, point):
        message = PointStamped()
        message.header.frame_id = self.frame_id
        message.header.stamp = rospy.Time.now()
        message.point = self.copy_point(point)
        self.waypoint_pub.publish(message)

    def start_callback(self, _request):
        with self.lock:
            if len(self.relative_points) < 2:
                return TriggerResponse(False, "No valid route is loaded.")
            if self.robot_position is None or self.mission_start_pose is None:
                return TriggerResponse(False, "Odometry/start pose is unavailable.")
            if (rospy.Time.now() - self.last_odometry_time).to_sec() > self.odometry_timeout:
                return TriggerResponse(False, "Odometry is stale.")

            self.transform_route()
            self.path_index = 0
            self.last_progress_position = self.copy_point(self.robot_position)
            self.last_progress_time = rospy.Time.now()
            self.arrival_candidate_time = None
            self.motion_enable_time = rospy.Time.now() + rospy.Duration(self.start_delay)
            self.active = True
            self.publish_path()
            self.set_status("Replay active; waypoint release starts after %.1f s." %
                            self.start_delay)
            return TriggerResponse(True, "UAV route replay started.")

    def stop_callback(self, _request):
        with self.lock:
            self.stop("Replay stopped by operator; holding current position.")
            return TriggerResponse(True, "UAV route replay stopped.")

    def reload_callback(self, _request):
        success = self.load_route()
        return TriggerResponse(success, "Route reloaded." if success
                               else "Route reload failed.")

    def stop(self, status):
        self.active = False
        self.path_index = 0
        self.arrival_candidate_time = None
        if self.robot_position is not None:
            hold = self.copy_point(self.robot_position)
            hold.z = self.flight_z() if self.mission_start_pose is not None else hold.z
            self.publish_waypoint(hold)
        self.set_status(status)

    def select_lookahead(self):
        search_end = min(len(self.world_points), self.path_index + 50)
        nearest_index = min(
            range(self.path_index, search_end),
            key=lambda index: self.distance_xy(
                self.robot_position, self.world_points[index]))
        self.path_index = nearest_index
        distance = 0.0
        for index in range(nearest_index + 1, len(self.world_points)):
            distance += self.distance_xy(self.world_points[index - 1],
                                         self.world_points[index])
            if distance >= self.lookahead_distance:
                return self.world_points[index]
        return self.world_points[-1]

    def control_timer_callback(self, _event):
        with self.lock:
            if not self.active:
                return
            now = rospy.Time.now()
            if self.robot_position is None or (
                    now - self.last_odometry_time).to_sec() > self.odometry_timeout:
                self.stop("Replay stopped: odometry timeout.")
                return

            goal_distance = self.distance_xy(
                self.robot_position, self.world_points[-1])
            if goal_distance <= self.arrival_distance:
                if self.arrival_candidate_time is None:
                    self.arrival_candidate_time = now
                if (now - self.arrival_candidate_time).to_sec() >= self.arrival_stable_seconds:
                    self.stop("Replay completed: destination reached and stable.")
                    return
            else:
                self.arrival_candidate_time = None

            if self.distance_xy(self.last_progress_position,
                                self.robot_position) >= self.min_progress_distance:
                self.last_progress_position = self.copy_point(self.robot_position)
                self.last_progress_time = now
            elif (now - self.last_progress_time).to_sec() >= self.progress_timeout:
                self.stop("Replay stopped: no translational progress.")
                return

            self.publish_path()
            if now < self.motion_enable_time:
                hold = self.copy_point(self.robot_position)
                hold.z = self.flight_z()
                self.publish_waypoint(hold)
            else:
                self.publish_waypoint(self.select_lookahead())


if __name__ == "__main__":
    rospy.init_node("uav_path_replayer")
    UavPathReplayer()
    rospy.spin()
