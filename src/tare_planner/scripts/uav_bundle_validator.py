#!/usr/bin/env python3

import hashlib
import json
import math
import os

import rospy
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger, TriggerResponse


def file_sha256(path):
    digest = hashlib.sha256()
    with open(path, "rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


class UavBundleValidator:
    def __init__(self):
        self.bundle_dir = os.path.abspath(os.path.expanduser(
            rospy.get_param("~bundle_dir", "~/tare_uav_handoff")))
        self.manifest_file = os.path.abspath(os.path.expanduser(
            rospy.get_param("~manifest_file",
                            os.path.join(self.bundle_dir, "manifest.json"))))
        self.fixed_height = float(rospy.get_param("~fixed_flight_height", 1.5))
        self.relative_height = bool(rospy.get_param(
            "~fixed_flight_height_relative_to_start", False))
        self.height_tolerance = float(rospy.get_param(
            "~height_match_tolerance", 0.01))
        self.require_map = bool(rospy.get_param("~require_map", False))
        self.status_pub = rospy.Publisher(
            rospy.get_param("~status_topic", "/tare_uav/handoff/validation_status"),
            String, queue_size=1, latch=True)
        self.valid_pub = rospy.Publisher(
            rospy.get_param("~valid_topic", "/tare_uav/handoff/valid"),
            Bool, queue_size=1, latch=True)
        rospy.Service("~validate", Trigger, self.validate_service)
        success, message = self.validate()
        self.publish(success, message)

    def publish(self, success, message):
        (rospy.loginfo if success else rospy.logerr)(
            "[uav_bundle_validator] %s", message)
        self.valid_pub.publish(Bool(data=success))
        self.status_pub.publish(String(data=message))

    def validate_entry(self, entry, label):
        if not isinstance(entry, dict):
            raise ValueError("manifest has no %s entry" % label)
        filename = entry.get("file", "")
        if not filename or os.path.basename(filename) != filename:
            raise ValueError("%s file must be a bundle-local filename" % label)
        path = os.path.join(self.bundle_dir, filename)
        if not os.path.isfile(path):
            raise ValueError("%s file is missing: %s" % (label, path))
        expected_size = int(entry.get("size_bytes", -1))
        if expected_size != os.path.getsize(path):
            raise ValueError("%s size does not match manifest" % label)
        if file_sha256(path) != entry.get("sha256"):
            raise ValueError("%s SHA256 does not match manifest" % label)
        return path

    def validate(self):
        try:
            with open(self.manifest_file, "r", encoding="utf-8") as stream:
                manifest = json.load(stream)
            if manifest.get("schema_version") != 1 or manifest.get(
                    "bundle_type") != "tare_uav_map_path_handoff":
                raise ValueError("unsupported handoff manifest")
            route_path = self.validate_entry(manifest.get("route"), "route")
            map_entry = manifest.get("map")
            map_path = None
            if map_entry is not None:
                map_path = self.validate_entry(map_entry, "map")
            elif self.require_map:
                raise ValueError("a PCD map is required but the bundle is route-only")

            policy = manifest.get("height_policy", {})
            route_height = float(policy.get("fixed_flight_height_m", float("nan")))
            route_relative = bool(policy.get("relative_to_start", False))
            if not math.isfinite(route_height):
                raise ValueError("height policy is missing")
            if route_relative != self.relative_height:
                raise ValueError("relative-height policy differs from launch")
            if abs(route_height - self.fixed_height) > self.height_tolerance:
                raise ValueError(
                    "bundle height %.3f m differs from fixed_flight_height %.3f m" %
                    (route_height, self.fixed_height))
            message = "Valid UAV handoff: route=%s" % route_path
            if map_path:
                message += ", map=%s" % map_path
            return True, message
        except (OSError, ValueError, TypeError, KeyError, json.JSONDecodeError) as error:
            return False, "UAV handoff validation failed: %s" % error

    def validate_service(self, _request):
        success, message = self.validate()
        self.publish(success, message)
        return TriggerResponse(success, message)


if __name__ == "__main__":
    rospy.init_node("uav_bundle_validator")
    UavBundleValidator()
    rospy.spin()
