#!/usr/bin/env python3

import hashlib
import json
import math
import os
import shutil
import tempfile

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerResponse


def yaw_from_quaternion(quaternion):
    sin_yaw = 2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y)
    cos_yaw = 1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z)
    return math.atan2(sin_yaw, cos_yaw)


def canonical_sha256(value):
    payload = json.dumps(value, sort_keys=True, separators=(",", ":"),
                         ensure_ascii=True).encode("utf-8")
    return hashlib.sha256(payload).hexdigest()


def file_sha256(path):
    digest = hashlib.sha256()
    with open(path, "rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def atomic_json_write(path, document):
    directory = os.path.dirname(path)
    os.makedirs(directory, exist_ok=True)
    descriptor, temporary_path = tempfile.mkstemp(
        prefix=".uav_route_", suffix=".tmp", dir=directory)
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as output:
            json.dump(document, output, indent=2, sort_keys=True,
                      ensure_ascii=True)
            output.write("\n")
            output.flush()
            os.fsync(output.fileno())
        os.replace(temporary_path, path)
    except Exception:
        if os.path.exists(temporary_path):
            os.unlink(temporary_path)
        raise


class UavPathExporter:
    def __init__(self):
        self.frame_id = rospy.get_param("~frame_id", "map").lstrip("/")
        self.bundle_dir = os.path.abspath(os.path.expanduser(
            rospy.get_param("~bundle_dir", "~/tare_uav_handoff")))
        self.route_file = os.path.abspath(os.path.expanduser(
            rospy.get_param("~route_file",
                            os.path.join(self.bundle_dir, "uav_route.json"))))
        self.manifest_file = os.path.abspath(os.path.expanduser(
            rospy.get_param("~manifest_file",
                            os.path.join(self.bundle_dir, "manifest.json"))))
        self.map_file = os.path.abspath(os.path.expanduser(
            rospy.get_param("~map_file", ""))) if rospy.get_param(
                "~map_file", "") else ""
        self.copy_map = bool(rospy.get_param("~copy_map", True))
        self.fixed_height = float(rospy.get_param("~fixed_flight_height", 1.5))
        self.relative_height = bool(rospy.get_param(
            "~fixed_flight_height_relative_to_start", False))
        self.start_match_tolerance = float(rospy.get_param(
            "~start_match_tolerance", 0.75))
        self.height_tolerance = float(rospy.get_param("~height_tolerance", 0.10))
        self.vehicle = {
            "length_m": float(rospy.get_param("~vehicle_length", 0.45298)),
            "width_m": float(rospy.get_param("~vehicle_width", 0.45298)),
            "height_m": float(rospy.get_param("~vehicle_height", 0.17139)),
            "mass_kg": float(rospy.get_param("~vehicle_mass", 3.0)),
        }
        self.clearance = {
            "horizontal_m": float(rospy.get_param(
                "~horizontal_clearance", 0.75)),
            "vertical_m": float(rospy.get_param("~vertical_clearance", 0.35)),
        }

        self.start_pose = None
        self.path = None
        self.status_pub = rospy.Publisher(
            rospy.get_param("~status_topic", "/tare_uav/handoff/export_status"),
            String, queue_size=1, latch=True)
        rospy.Subscriber(
            rospy.get_param("~start_pose_topic",
                            "/tare_uav/exploration_start_pose"),
            PoseStamped, self.start_pose_callback, queue_size=1)
        rospy.Subscriber(
            rospy.get_param("~path_topic", "/tare_uav/fixed_start_path"),
            Path, self.path_callback, queue_size=1)
        rospy.Service("~save", Trigger, self.save_callback)
        rospy.Service("~clear", Trigger, self.clear_callback)
        self.set_status("Waiting for exploration start pose and fixed-start path.")

    def set_status(self, text):
        rospy.loginfo("[uav_path_exporter] %s", text)
        self.status_pub.publish(String(data=text))

    def start_pose_callback(self, message):
        frame = message.header.frame_id.lstrip("/")
        if frame and frame != self.frame_id:
            self.set_status("Rejected start pose in frame %s; expected %s." %
                            (frame, self.frame_id))
            return
        self.start_pose = message
        self.set_status("Exploration start pose captured; waiting for fixed-start path.")

    def expected_flight_z(self):
        if self.start_pose is None:
            return None
        if self.relative_height:
            return self.start_pose.pose.position.z + self.fixed_height
        return self.fixed_height

    def path_callback(self, message):
        if len(message.poses) < 2:
            return
        frame = (message.header.frame_id or
                 message.poses[0].header.frame_id).lstrip("/")
        if frame != self.frame_id:
            self.set_status("Rejected path in frame %s; expected %s." %
                            (frame, self.frame_id))
            return
        if self.start_pose is None:
            self.set_status("Path received before exploration start pose.")
            return

        path_start = message.poses[0].pose.position
        recorded_start = self.start_pose.pose.position
        start_error = math.hypot(path_start.x - recorded_start.x,
                                 path_start.y - recorded_start.y)
        if start_error > self.start_match_tolerance:
            self.set_status(
                "Rejected path: first point is %.2f m from exploration start." %
                start_error)
            return

        expected_z = self.expected_flight_z()
        max_height_error = max(abs(pose.pose.position.z - expected_z)
                               for pose in message.poses)
        if max_height_error > self.height_tolerance:
            self.set_status(
                "Rejected path: z differs from fixed_flight_height by %.2f m." %
                max_height_error)
            return

        self.path = message
        self.set_status(
            "Fixed-start path captured (%d poses). Call /uav_path_exporter/save." %
            len(message.poses))

    @staticmethod
    def path_length(points):
        return sum(math.hypot(points[index]["x"] - points[index - 1]["x"],
                              points[index]["y"] - points[index - 1]["y"])
                   for index in range(1, len(points)))

    def build_route(self):
        start = self.start_pose.pose
        start_yaw = yaw_from_quaternion(start.orientation)
        cos_yaw = math.cos(start_yaw)
        sin_yaw = math.sin(start_yaw)
        expected_z = self.expected_flight_z()
        stored_z = (expected_z - start.position.z
                    if self.relative_height else expected_z)

        points = []
        for pose_stamped in self.path.poses:
            point = pose_stamped.pose.position
            dx = point.x - start.position.x
            dy = point.y - start.position.y
            points.append({
                "x": cos_yaw * dx + sin_yaw * dy,
                "y": -sin_yaw * dx + cos_yaw * dy,
                "z": stored_z,
            })

        route_payload = {
            "coordinate_system": "start_relative_xy_yaw",
            "height_policy": {
                "fixed_flight_height_m": self.fixed_height,
                "relative_to_start": self.relative_height,
            },
            "points": points,
        }
        return {
            "schema_version": 2,
            "created_at_ros_time": rospy.Time.now().to_sec(),
            "source_frame": self.frame_id,
            "coordinate_system": route_payload["coordinate_system"],
            "height_policy": route_payload["height_policy"],
            "source_start_pose": {
                "x": start.position.x,
                "y": start.position.y,
                "z": start.position.z,
                "yaw": start_yaw,
            },
            "vehicle": self.vehicle,
            "clearance": self.clearance,
            "point_count": len(points),
            "path_length_m": self.path_length(points),
            "payload_sha256": canonical_sha256(route_payload),
            "points": points,
        }

    def copy_map_to_bundle(self):
        if not self.map_file:
            return None
        if not os.path.isfile(self.map_file):
            raise OSError("map file does not exist: %s" % self.map_file)
        os.makedirs(self.bundle_dir, exist_ok=True)
        destination = os.path.join(self.bundle_dir,
                                   os.path.basename(self.map_file))
        if self.copy_map and os.path.abspath(destination) != self.map_file:
            temporary = destination + ".tmp"
            shutil.copy2(self.map_file, temporary)
            os.replace(temporary, destination)
        else:
            destination = self.map_file
        return {
            "file": os.path.basename(destination),
            "sha256": file_sha256(destination),
            "size_bytes": os.path.getsize(destination),
        }

    def save_callback(self, _request):
        if self.start_pose is None or self.path is None:
            return TriggerResponse(False, "No valid fixed-start path is ready.")
        try:
            route = self.build_route()
            atomic_json_write(self.route_file, route)
            map_entry = self.copy_map_to_bundle()
            route_hash = file_sha256(self.route_file)
            manifest = {
                "schema_version": 1,
                "bundle_type": "tare_uav_map_path_handoff",
                "frame_id": self.frame_id,
                "route": {
                    "file": os.path.basename(self.route_file),
                    "sha256": route_hash,
                    "size_bytes": os.path.getsize(self.route_file),
                },
                "map": map_entry,
                "height_policy": route["height_policy"],
                "vehicle": self.vehicle,
                "clearance": self.clearance,
            }
            atomic_json_write(self.manifest_file, manifest)
        except (OSError, ValueError, TypeError) as error:
            message = "Export failed: %s" % error
            self.set_status(message)
            return TriggerResponse(False, message)

        map_text = " with PCD map" if map_entry else " (route only; no PCD configured)"
        message = "Saved %.1f m UAV route to %s%s" % (
            route["path_length_m"], self.bundle_dir, map_text)
        self.set_status(message)
        return TriggerResponse(True, message)

    def clear_callback(self, _request):
        self.path = None
        message = "Captured path cleared; files on disk were not deleted."
        self.set_status(message)
        return TriggerResponse(True, message)


if __name__ == "__main__":
    rospy.init_node("uav_path_exporter")
    UavPathExporter()
    rospy.spin()
