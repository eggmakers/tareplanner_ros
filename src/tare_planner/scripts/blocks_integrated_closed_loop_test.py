#!/usr/bin/env python3
import json
import math
import os
import sys
import time
from threading import Lock

import airsim
import rospy
import sensor_msgs.point_cloud2 as point_cloud2
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import ExtendedState, State
from mavros_msgs.srv import SetMode
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger


class BlocksIntegratedClosedLoopTest:
    def __init__(self):
        self.fixed_height = float(rospy.get_param("~fixed_height", 1.5))
        self.height_tolerance = float(rospy.get_param("~height_tolerance", 0.25))
        self.arrival_tolerance = float(rospy.get_param("~arrival_tolerance", 0.5))
        self.navigation_timeout = float(rospy.get_param("~navigation_timeout", 180.0))
        self.landing_timeout = float(rospy.get_param("~landing_timeout", 60.0))
        self.minimum_goal_distance = float(
            rospy.get_param("~minimum_goal_distance", 1.8)
        )
        self.preferred_goal_distance = float(
            rospy.get_param("~preferred_goal_distance", 2.8)
        )
        self.maximum_goal_distance = float(
            rospy.get_param("~maximum_goal_distance", 4.0)
        )
        self.minimum_progress = float(rospy.get_param("~minimum_progress", 1.0))
        self.hold_settle_seconds = float(rospy.get_param("~hold_settle_seconds", 4.0))
        self.hold_observation_seconds = float(
            rospy.get_param("~hold_observation_seconds", 3.0)
        )
        self.hold_radius = float(rospy.get_param("~hold_radius", 0.35))
        self.report_path = rospy.get_param(
            "~report_path", "/data/blocks_integrated_closed_loop_report.json"
        )
        self.route_file = rospy.get_param(
            "~route_file", os.path.expanduser("~/tare_uav_handoff/uav_route.json")
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
        self.state = None
        self.extended_state = None
        self.odom = None
        self.selected_mode = None
        self.tare_mode = None
        self.navigation_active = None
        self.navigation_reached = None
        self.navigation_path = None
        self.navigation_path_received = 0.0
        self.fixed_start_path = None
        self.fixed_start_path_received = 0.0
        self.local_blocked = None
        self.trajectory = []

        rospy.Subscriber("/mavros/state", State, self.state_callback, queue_size=10)
        rospy.Subscriber(
            "/mavros/extended_state",
            ExtendedState,
            self.extended_state_callback,
            queue_size=10,
        )
        rospy.Subscriber(
            "/mavros/local_position/odom", Odometry, self.odom_callback, queue_size=10
        )
        rospy.Subscriber(
            "/tare_uav/mission/selected",
            String,
            self.selected_mode_callback,
            queue_size=10,
        )
        rospy.Subscriber(
            "/tare_uav/mission/mode", String, self.tare_mode_callback, queue_size=10
        )
        rospy.Subscriber(
            "/tare_uav/navigation/active",
            Bool,
            self.navigation_active_callback,
            queue_size=10,
        )
        rospy.Subscriber(
            "/tare_uav/navigation/reached",
            Bool,
            self.navigation_reached_callback,
            queue_size=10,
        )
        rospy.Subscriber(
            "/tare_uav/navigation/path",
            Path,
            self.navigation_path_callback,
            queue_size=5,
        )
        rospy.Subscriber(
            "/tare_uav/fixed_start_path",
            Path,
            self.fixed_start_path_callback,
            queue_size=5,
        )
        rospy.Subscriber(
            "/tare_uav/local_planner/blocked",
            Bool,
            self.local_blocked_callback,
            queue_size=10,
        )
        rospy.Subscriber(
            "/trajectory", PointCloud2, self.trajectory_callback, queue_size=2
        )
        self.goal_pub = rospy.Publisher(
            "/move_base_simple/goal", PoseStamped, queue_size=1, latch=True
        )

        self.client = airsim.MultirotorClient(
            ip=self.airsim_host, port=self.airsim_port, timeout_value=3.0
        )
        collision = self.client.simGetCollisionInfo(vehicle_name=self.vehicle_name)
        self.collision_baseline_timestamp = collision.time_stamp

    def state_callback(self, message):
        with self.lock:
            self.state = message

    def extended_state_callback(self, message):
        with self.lock:
            self.extended_state = message

    def odom_callback(self, message):
        with self.lock:
            self.odom = message

    def selected_mode_callback(self, message):
        with self.lock:
            self.selected_mode = message.data.strip().lower()

    def tare_mode_callback(self, message):
        with self.lock:
            self.tare_mode = message.data.strip().upper()

    def navigation_active_callback(self, message):
        with self.lock:
            self.navigation_active = bool(message.data)

    def navigation_reached_callback(self, message):
        with self.lock:
            self.navigation_reached = bool(message.data)

    def navigation_path_callback(self, message):
        with self.lock:
            self.navigation_path = message
            self.navigation_path_received = time.monotonic()

    def fixed_start_path_callback(self, message):
        with self.lock:
            self.fixed_start_path = message
            self.fixed_start_path_received = time.monotonic()

    def local_blocked_callback(self, message):
        with self.lock:
            self.local_blocked = bool(message.data)

    def trajectory_callback(self, message):
        points = []
        try:
            for x, y, z in point_cloud2.read_points(
                message, field_names=("x", "y", "z"), skip_nans=True
            ):
                if math.isfinite(x) and math.isfinite(y) and math.isfinite(z):
                    points.append((float(x), float(y), float(z)))
        except (ValueError, TypeError):
            return
        with self.lock:
            self.trajectory = points

    def snapshot(self):
        with self.lock:
            return {
                "state": self.state,
                "extended_state": self.extended_state,
                "odom": self.odom,
                "selected_mode": self.selected_mode,
                "tare_mode": self.tare_mode,
                "navigation_active": self.navigation_active,
                "navigation_reached": self.navigation_reached,
                "navigation_path": self.navigation_path,
                "navigation_path_received": self.navigation_path_received,
                "fixed_start_path": self.fixed_start_path,
                "fixed_start_path_received": self.fixed_start_path_received,
                "local_blocked": self.local_blocked,
                "trajectory": list(self.trajectory),
            }

    @staticmethod
    def xy(position):
        return position.x, position.y

    @staticmethod
    def distance_xy(position, goal):
        return math.hypot(position.x - goal[0], position.y - goal[1])

    def wait_for(self, predicate, timeout, description):
        deadline = time.monotonic() + timeout
        rate = rospy.Rate(10.0)
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            if predicate():
                return True
            rate.sleep()
        rospy.logerr("Timed out waiting for %s", description)
        return False

    def call_trigger(self, service_name, timeout=10.0):
        rospy.wait_for_service(service_name, timeout=timeout)
        response = rospy.ServiceProxy(service_name, Trigger)()
        if not response.success:
            raise RuntimeError("{}: {}".format(service_name, response.message))
        return response.message

    def select_mode(self, mode):
        service = {
            "tare": "/uav_mission_mux/select_tare",
            "fixed_start": "/uav_mission_mux/select_fixed_start",
            "hold": "/uav_mission_mux/hold",
        }[mode]
        self.call_trigger(service)
        return self.wait_for(
            lambda: self.snapshot()["selected_mode"] == mode,
            10.0,
            "mission mux mode {}".format(mode),
        )

    def choose_known_goal(self, position, trajectory):
        candidates = []
        for point in trajectory:
            distance = math.hypot(position.x - point[0], position.y - point[1])
            if self.minimum_goal_distance <= distance <= self.maximum_goal_distance:
                candidates.append((abs(distance - self.preferred_goal_distance), point))
        if not candidates:
            return None
        candidates.sort(key=lambda item: item[0])
        return candidates[0][1]

    def publish_goal(self, goal):
        message = PoseStamped()
        message.header.stamp = rospy.Time.now()
        message.header.frame_id = "map"
        message.pose.position.x = goal[0]
        message.pose.position.y = goal[1]
        message.pose.position.z = self.fixed_height
        message.pose.orientation.w = 1.0
        self.goal_pub.publish(message)

    def hold_is_stable(self):
        rospy.sleep(self.hold_settle_seconds)
        snapshot = self.snapshot()
        if snapshot["odom"] is None:
            return False, None
        origin = snapshot["odom"].pose.pose.position
        origin_xy = self.xy(origin)
        maximum_displacement = 0.0
        deadline = time.monotonic() + self.hold_observation_seconds
        rate = rospy.Rate(10.0)
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            odom = self.snapshot()["odom"]
            if odom is not None:
                position = odom.pose.pose.position
                maximum_displacement = max(
                    maximum_displacement,
                    math.hypot(position.x - origin_xy[0], position.y - origin_xy[1]),
                )
            rate.sleep()
        return maximum_displacement <= self.hold_radius, maximum_displacement

    def collision_details(self):
        collision = self.client.simGetCollisionInfo(vehicle_name=self.vehicle_name)
        collided = bool(
            collision.has_collided
            and collision.time_stamp > self.collision_baseline_timestamp
        )
        return collided, collision.object_name if collided else None

    def write_report(self, checks, details):
        passed = all(checks.values())
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
            "integrated_tare_mode": False,
            "hold_selected": False,
            "hold_position_stable": False,
            "fixed_start_selected": False,
            "fixed_start_path_generated": False,
            "route_exported": False,
            "tare_reselected": False,
            "rviz_goal_routed": False,
            "astar_path_generated": False,
            "distance_decreased": False,
            "goal_reached": False,
            "fixed_height_held": True,
            "local_planner_clear_at_finish": False,
            "landing_mode_accepted": False,
            "landed_and_disarmed": False,
            "airsim_collision_free": True,
            "landing_contact_is_ground_only": True,
        }
        details = {
            "goal_enu": None,
            "start_position_enu": None,
            "final_navigation_position_enu": None,
            "initial_goal_distance_m": None,
            "minimum_goal_distance_m": None,
            "hold_maximum_displacement_m": None,
            "fixed_start_path_poses": 0,
            "navigation_path_poses": 0,
            "route_file": self.route_file,
            "collision_object": None,
            "flight_collision_object": None,
        }

        checks["inputs_ready"] = self.wait_for(
            lambda: (
                self.snapshot()["state"] is not None
                and self.snapshot()["state"].connected
                and self.snapshot()["state"].armed
                and self.snapshot()["state"].mode == "OFFBOARD"
                and self.snapshot()["odom"] is not None
                and len(self.snapshot()["trajectory"]) >= 10
            ),
            20.0,
            "armed OFFBOARD vehicle, odometry, and trajectory",
        )
        if not checks["inputs_ready"]:
            return self.write_report(checks, details)

        checks["integrated_tare_mode"] = (
            self.snapshot()["selected_mode"] == "tare"
        )
        current = self.snapshot()["odom"].pose.pose.position
        trajectory = self.snapshot()["trajectory"]
        goal = self.choose_known_goal(current, trajectory)
        if goal is None:
            details["goal_selection_error"] = (
                "No previously flown point was within the configured distance band"
            )
            return self.write_report(checks, details)
        goal = (goal[0], goal[1], self.fixed_height)
        details["goal_enu"] = list(goal)

        try:
            checks["hold_selected"] = self.select_mode("hold")
            if checks["hold_selected"]:
                stable, displacement = self.hold_is_stable()
                checks["hold_position_stable"] = stable
                details["hold_maximum_displacement_m"] = round(displacement, 3)

            checks["fixed_start_selected"] = self.select_mode("fixed_start")
            request_time = time.monotonic()
            fixed_deadline = request_time + 20.0
            while not rospy.is_shutdown() and time.monotonic() < fixed_deadline:
                self.publish_goal(goal)
                snapshot = self.snapshot()
                path = snapshot["fixed_start_path"]
                if (
                    snapshot["fixed_start_path_received"] >= request_time
                    and path is not None
                    and len(path.poses) >= 2
                ):
                    checks["fixed_start_path_generated"] = True
                    details["fixed_start_path_poses"] = len(path.poses)
                    break
                rospy.sleep(0.5)

            if checks["fixed_start_path_generated"]:
                self.call_trigger("/uav_path_exporter/save")
                checks["route_exported"] = os.path.isfile(self.route_file)

            checks["tare_reselected"] = self.select_mode("tare")
            navigation_request_time = time.monotonic()
            navigation_deadline = navigation_request_time + 20.0
            while not rospy.is_shutdown() and time.monotonic() < navigation_deadline:
                self.publish_goal(goal)
                snapshot = self.snapshot()
                if snapshot["tare_mode"] == "NAVIGATION":
                    checks["rviz_goal_routed"] = True
                path = snapshot["navigation_path"]
                if (
                    snapshot["navigation_path_received"] >= navigation_request_time
                    and path is not None
                    and len(path.poses) >= 2
                ):
                    checks["astar_path_generated"] = True
                    details["navigation_path_poses"] = len(path.poses)
                if checks["rviz_goal_routed"] and checks["astar_path_generated"]:
                    break
                rospy.sleep(0.5)

            start_odom = self.snapshot()["odom"]
            start_position = start_odom.pose.pose.position
            initial_distance = self.distance_xy(start_position, goal)
            minimum_distance = initial_distance
            details["start_position_enu"] = [
                start_position.x,
                start_position.y,
                start_position.z,
            ]
            details["initial_goal_distance_m"] = round(initial_distance, 3)

            started = time.monotonic()
            next_log = started
            rate = rospy.Rate(10.0)
            while (
                not rospy.is_shutdown()
                and time.monotonic() - started < self.navigation_timeout
            ):
                snapshot = self.snapshot()
                odom = snapshot["odom"]
                if odom is not None:
                    position = odom.pose.pose.position
                    distance = self.distance_xy(position, goal)
                    minimum_distance = min(minimum_distance, distance)
                    checks["distance_decreased"] = (
                        initial_distance - minimum_distance >= self.minimum_progress
                    )
                    if abs(position.z - self.fixed_height) > self.height_tolerance:
                        checks["fixed_height_held"] = False
                    if (
                        snapshot["navigation_reached"]
                        and distance <= self.arrival_tolerance
                    ):
                        checks["goal_reached"] = True
                        checks["local_planner_clear_at_finish"] = not bool(
                            snapshot["local_blocked"]
                        )
                        details["final_navigation_position_enu"] = [
                            position.x,
                            position.y,
                            position.z,
                        ]
                        break
                if time.monotonic() >= next_log:
                    rospy.loginfo(
                        "Integrated navigation: mode=%s tare=%s distance=%.2f blocked=%s",
                        snapshot["selected_mode"],
                        snapshot["tare_mode"],
                        distance if odom is not None else float("nan"),
                        snapshot["local_blocked"],
                    )
                    next_log += 5.0
                rate.sleep()
            details["minimum_goal_distance_m"] = round(minimum_distance, 3)

            flight_collision, flight_collision_object = self.collision_details()
            checks["airsim_collision_free"] = not flight_collision
            details["flight_collision_object"] = flight_collision_object

            self.select_mode("hold")
            rospy.wait_for_service("/mavros/set_mode", timeout=10.0)
            response = rospy.ServiceProxy("/mavros/set_mode", SetMode)(
                base_mode=0, custom_mode="AUTO.LAND"
            )
            checks["landing_mode_accepted"] = bool(response.mode_sent)
            checks["landed_and_disarmed"] = self.wait_for(
                lambda: (
                    self.snapshot()["state"] is not None
                    and not self.snapshot()["state"].armed
                    and self.snapshot()["extended_state"] is not None
                    and self.snapshot()["extended_state"].landed_state
                    == ExtendedState.LANDED_STATE_ON_GROUND
                ),
                self.landing_timeout,
                "AUTO.LAND completion and disarm",
            )
        except (rospy.ROSException, rospy.ServiceException, RuntimeError) as error:
            details["service_error"] = str(error)

        collision, collision_object = self.collision_details()
        if collision:
            checks["landing_contact_is_ground_only"] = bool(
                collision_object and collision_object.lower().startswith("ground")
            )
        details["collision_object"] = collision_object
        final = self.snapshot()
        if final["state"] is not None:
            details["final_px4_mode"] = final["state"].mode
            details["final_armed"] = final["state"].armed
        if final["extended_state"] is not None:
            details["final_landed_state"] = final["extended_state"].landed_state
        return self.write_report(checks, details)


if __name__ == "__main__":
    rospy.init_node("blocks_integrated_closed_loop_test", anonymous=True)
    sys.exit(BlocksIntegratedClosedLoopTest().run())
