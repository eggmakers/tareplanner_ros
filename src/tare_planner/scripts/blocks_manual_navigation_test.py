#!/usr/bin/env python3
import json
import math
import os
import sys
import time
from threading import Lock

import airsim
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Bool, String


class BlocksManualNavigationTest:
    def __init__(self):
        self.goal_x = float(rospy.get_param("~goal_x"))
        self.goal_y = float(rospy.get_param("~goal_y"))
        self.fixed_height = float(rospy.get_param("~fixed_height", 1.5))
        self.timeout = float(rospy.get_param("~timeout", 180.0))
        self.arrival_tolerance = float(rospy.get_param("~arrival_tolerance", 0.5))
        self.height_tolerance = float(rospy.get_param("~height_tolerance", 0.25))
        self.min_path_poses = int(rospy.get_param("~min_path_poses", 2))
        self.min_progress = float(rospy.get_param("~min_progress", 1.0))
        self.report_path = rospy.get_param(
            "~report_path", "/data/blocks_manual_navigation_report.json"
        )
        self.airsim_host = rospy.get_param(
            "~airsim_host", os.environ.get("AIRSIM_HOST", "host.docker.internal")
        )
        self.airsim_port = int(
            rospy.get_param(
                "~airsim_port", os.environ.get("AIRSIM_RPC_PORT", "41452")
            )
        )
        self.vehicle_name = rospy.get_param(
            "~vehicle_name", os.environ.get("AIRSIM_VEHICLE_NAME", "PX4")
        )

        self.lock = Lock()
        self.odom = None
        self.mode = None
        self.active = None
        self.reached = None
        self.path = None
        self.blocked = None

        rospy.Subscriber(
            "/mavros/local_position/odom", Odometry, self.odom_callback, queue_size=10
        )
        rospy.Subscriber(
            "/tare_uav/mission/mode", String, self.mode_callback, queue_size=10
        )
        rospy.Subscriber(
            "/tare_uav/navigation/active", Bool, self.active_callback, queue_size=10
        )
        rospy.Subscriber(
            "/tare_uav/navigation/reached", Bool, self.reached_callback, queue_size=10
        )
        rospy.Subscriber(
            "/tare_uav/navigation/path", Path, self.path_callback, queue_size=10
        )
        rospy.Subscriber(
            "/tare_uav/local_planner/blocked", Bool, self.blocked_callback, queue_size=10
        )
        self.pause_pub = rospy.Publisher(
            "/tare_uav/mission/pause", Bool, queue_size=1, latch=True
        )
        self.goal_pub = rospy.Publisher(
            "/move_base_simple/goal", PoseStamped, queue_size=1, latch=True
        )

        self.client = airsim.MultirotorClient(
            ip=self.airsim_host, port=self.airsim_port, timeout_value=3.0
        )
        collision = self.client.simGetCollisionInfo(vehicle_name=self.vehicle_name)
        self.collision_baseline_timestamp = collision.time_stamp

    def odom_callback(self, msg):
        with self.lock:
            self.odom = msg

    def mode_callback(self, msg):
        with self.lock:
            self.mode = msg.data

    def active_callback(self, msg):
        with self.lock:
            self.active = bool(msg.data)

    def reached_callback(self, msg):
        with self.lock:
            self.reached = bool(msg.data)

    def path_callback(self, msg):
        with self.lock:
            self.path = msg

    def blocked_callback(self, msg):
        with self.lock:
            self.blocked = bool(msg.data)

    def snapshot(self):
        with self.lock:
            return self.odom, self.mode, self.active, self.reached, self.path, self.blocked

    def wait_for(self, predicate, timeout, description):
        deadline = time.monotonic() + timeout
        rate = rospy.Rate(10.0)
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            if predicate():
                return True
            rate.sleep()
        rospy.logerr("Timed out waiting for %s", description)
        return False

    def distance_to_goal(self, odom):
        if odom is None:
            return None
        position = odom.pose.pose.position
        return math.hypot(position.x - self.goal_x, position.y - self.goal_y)

    def publish_goal(self):
        goal = PoseStamped()
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = "map"
        goal.pose.position.x = self.goal_x
        goal.pose.position.y = self.goal_y
        goal.pose.position.z = self.fixed_height
        goal.pose.orientation.w = 1.0
        self.goal_pub.publish(goal)

    def collision_details(self):
        collision = self.client.simGetCollisionInfo(vehicle_name=self.vehicle_name)
        collided = bool(
            collision.has_collided
            and collision.time_stamp > self.collision_baseline_timestamp
        )
        return collided, collision.object_name if collided else None

    def write_report(self, passed, checks, details):
        report = {"passed": passed, "checks": checks, "details": details}
        directory = os.path.dirname(self.report_path)
        if directory:
            os.makedirs(directory, exist_ok=True)
        with open(self.report_path, "w", encoding="utf-8") as stream:
            json.dump(report, stream, indent=2, sort_keys=True)
            stream.write("\n")
        for name, result in checks.items():
            print("[{}] {}".format("PASS" if result else "FAIL", name))
        print(json.dumps(details, indent=2, sort_keys=True))
        print("Report: {}".format(self.report_path))
        return 0 if passed else 1

    def run(self):
        checks = {
            "inputs_ready": False,
            "pause_hold": False,
            "goal_accepted": False,
            "path_generated": False,
            "distance_decreased": False,
            "goal_reached": False,
            "fixed_height_held": True,
            "local_planner_not_blocked_at_finish": False,
            "airsim_collision_free": True,
        }
        details = {
            "goal_enu": [self.goal_x, self.goal_y, self.fixed_height],
            "start_position_enu": None,
            "final_position_enu": None,
            "initial_distance_m": None,
            "minimum_distance_m": None,
            "final_distance_m": None,
            "path_pose_count": 0,
            "collision_object": None,
        }

        checks["inputs_ready"] = self.wait_for(
            lambda: self.snapshot()[0] is not None and self.snapshot()[1] is not None,
            10.0,
            "odometry and mission mode",
        )
        if not checks["inputs_ready"]:
            return self.write_report(False, checks, details)

        self.pause_pub.publish(Bool(data=True))
        checks["pause_hold"] = self.wait_for(
            lambda: self.snapshot()[1] == "HOLD", 10.0, "HOLD mode"
        )
        if not checks["pause_hold"]:
            return self.write_report(False, checks, details)

        odom, _, _, _, _, _ = self.snapshot()
        start = odom.pose.pose.position
        initial_distance = self.distance_to_goal(odom)
        details["start_position_enu"] = [start.x, start.y, start.z]
        details["initial_distance_m"] = round(initial_distance, 3)
        minimum_distance = initial_distance

        navigation_deadline = time.monotonic() + 15.0
        while not rospy.is_shutdown() and time.monotonic() < navigation_deadline:
            self.publish_goal()
            if self.snapshot()[1] == "NAVIGATION":
                checks["goal_accepted"] = True
                break
            rospy.sleep(0.5)
        if not checks["goal_accepted"]:
            return self.write_report(False, checks, details)

        started = time.monotonic()
        next_log = started
        rate = rospy.Rate(10.0)
        while not rospy.is_shutdown() and time.monotonic() - started < self.timeout:
            odom, mode, active, reached, path, blocked = self.snapshot()
            if path is not None:
                details["path_pose_count"] = max(
                    details["path_pose_count"], len(path.poses)
                )
                checks["path_generated"] = (
                    details["path_pose_count"] >= self.min_path_poses
                )
            distance = self.distance_to_goal(odom)
            if distance is not None:
                minimum_distance = min(minimum_distance, distance)
                checks["distance_decreased"] = (
                    initial_distance - minimum_distance >= self.min_progress
                )
                if abs(odom.pose.pose.position.z - self.fixed_height) > self.height_tolerance:
                    checks["fixed_height_held"] = False
            if time.monotonic() >= next_log:
                rospy.loginfo(
                    "Manual navigation: mode=%s active=%s reached=%s distance=%.2f m path=%d blocked=%s",
                    mode,
                    active,
                    reached,
                    distance if distance is not None else float("nan"),
                    details["path_pose_count"],
                    blocked,
                )
                next_log += 5.0
            if reached and distance is not None and distance <= self.arrival_tolerance:
                checks["goal_reached"] = True
                checks["local_planner_not_blocked_at_finish"] = not bool(blocked)
                break
            rate.sleep()

        odom, mode, active, reached, _, blocked = self.snapshot()
        final_distance = self.distance_to_goal(odom)
        if odom is not None:
            final = odom.pose.pose.position
            details["final_position_enu"] = [final.x, final.y, final.z]
        details["minimum_distance_m"] = round(minimum_distance, 3)
        details["final_distance_m"] = (
            None if final_distance is None else round(final_distance, 3)
        )
        details["final_mode"] = mode
        details["navigation_active"] = active
        details["navigation_reached"] = reached
        details["local_planner_blocked"] = blocked
        collided, collision_object = self.collision_details()
        checks["airsim_collision_free"] = not collided
        details["collision_object"] = collision_object
        return self.write_report(all(checks.values()), checks, details)


if __name__ == "__main__":
    rospy.init_node("blocks_manual_navigation_test", anonymous=True)
    sys.exit(BlocksManualNavigationTest().run())
