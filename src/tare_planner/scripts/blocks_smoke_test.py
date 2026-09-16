#!/usr/bin/env python3
import json
import math
import os
import sys
import time
from collections import deque
from threading import Lock

import airsim
import rospy
from geometry_msgs.msg import PointStamped, PoseStamped, TwistStamped
from mavros_msgs.msg import State
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool, String


class BlocksSmokeTest:
    def __init__(self):
        self.timeout = float(rospy.get_param("~timeout", 300.0))
        self.fixed_height = float(rospy.get_param("~fixed_height", 1.5))
        self.height_tolerance = float(rospy.get_param("~height_tolerance", 0.6))
        self.coordinate_tolerance = float(
            rospy.get_param("~coordinate_tolerance", 2.0)
        )
        self.min_cloud_points = int(rospy.get_param("~min_cloud_points", 100))
        self.min_setpoint_rate = float(
            rospy.get_param("~min_setpoint_rate", 10.0)
        )
        self.max_command_xy_speed = float(
            rospy.get_param("~max_command_xy_speed", 0.65)
        )
        self.max_tilt_p95_deg = float(rospy.get_param("~max_tilt_p95_deg", 10.0))
        self.max_tilt_peak_deg = float(rospy.get_param("~max_tilt_peak_deg", 15.0))
        self.min_attitude_samples = int(rospy.get_param("~min_attitude_samples", 50))
        self.max_heading_error_p95_deg = float(
            rospy.get_param("~max_heading_error_p95_deg", 20.0)
        )
        self.max_heading_error_peak_deg = float(
            rospy.get_param("~max_heading_error_peak_deg", 30.0)
        )
        self.min_heading_samples = int(rospy.get_param("~min_heading_samples", 20))
        self.heading_sample_min_speed = float(
            rospy.get_param("~heading_sample_min_speed", 0.08)
        )
        self.height_hold_seconds = float(rospy.get_param("~height_hold_seconds", 2.0))
        self.min_observation_seconds = float(
            rospy.get_param("~min_observation_seconds", 60.0)
        )
        self.min_horizontal_travel = float(
            rospy.get_param("~min_horizontal_travel", 6.0)
        )
        self.max_takeoff_xy_drift = float(
            rospy.get_param("~max_takeoff_xy_drift", 0.35)
        )
        self.expect_offboard = bool(rospy.get_param("~expect_offboard", True))
        self.expect_armed = bool(rospy.get_param("~expect_armed", True))
        self.airsim_host = rospy.get_param(
            "~airsim_host", os.environ.get("AIRSIM_HOST", "host.docker.internal")
        )
        self.airsim_port = int(
            rospy.get_param(
                "~airsim_port", os.environ.get("AIRSIM_RPC_PORT", "41452")
            )
        )
        self.rpc_timeout = float(rospy.get_param("~rpc_timeout", 3.0))
        self.vehicle_name = rospy.get_param(
            "~vehicle_name", os.environ.get("AIRSIM_VEHICLE_NAME", "PX4")
        )
        self.world_to_map_z_offset = float(
            rospy.get_param(
                "~world_to_map_z_offset",
                os.environ.get("AIRSIM_WORLD_TO_MAP_Z_OFFSET", "0.7"),
            )
        )
        self.report_path = rospy.get_param(
            "~report_path", "/data/blocks_smoke_report.json"
        )
        self.geofence_file = rospy.get_param(
            "~geofence_file", "/data/tunnel_scene.json"
        )
        self.waypoint_topic = rospy.get_param("~waypoint_topic", "/way_point")
        self.geofence = self.load_geofence()

        self.lock = Lock()
        self.state = None
        self.odom = None
        self.cloud = None
        self.waypoint = None
        self.safe_waypoint = None
        self.local_planner_status = None
        self.local_planner_blocked = None
        self.setpoint = None
        self.setpoint_times = deque(maxlen=400)
        self.attitude_tilt_samples = deque(maxlen=2000)
        self.heading_error_samples = deque(maxlen=2000)
        self.command_velocity = None
        self.exploration_finished = False
        self.max_observed_command_xy_speed = 0.0
        self.height_reached_since = None
        self.armed_start_xy = None
        self.max_horizontal_travel = 0.0
        self.max_observed_takeoff_xy_drift = 0.0
        self.takeoff_phase_completed = False

        rospy.Subscriber("/mavros/state", State, self.state_callback, queue_size=10)
        rospy.Subscriber(
            "/mavros/local_position/odom",
            Odometry,
            self.odom_callback,
            queue_size=10,
        )
        rospy.Subscriber(
            "/airsim/registered_scan", PointCloud2, self.cloud_callback, queue_size=2
        )
        rospy.Subscriber(
            self.waypoint_topic,
            PointStamped,
            self.waypoint_callback,
            queue_size=5,
        )
        rospy.Subscriber(
            "/tare_uav/safe_waypoint",
            PointStamped,
            self.safe_waypoint_callback,
            queue_size=5,
        )
        rospy.Subscriber(
            "/tare_uav/local_planner/status",
            String,
            self.local_planner_status_callback,
            queue_size=5,
        )
        rospy.Subscriber(
            "/tare_uav/local_planner/blocked",
            Bool,
            self.local_planner_blocked_callback,
            queue_size=5,
        )
        rospy.Subscriber(
            "/mavros/setpoint_position/local",
            PoseStamped,
            self.setpoint_callback,
            queue_size=50,
        )
        rospy.Subscriber(
            "/tare_uav/command_velocity",
            TwistStamped,
            self.command_velocity_callback,
            queue_size=50,
        )
        rospy.Subscriber(
            "/sensor_coverage_planner/exploration_finish",
            Bool,
            self.exploration_finish_callback,
            queue_size=5,
        )

        self.client = airsim.MultirotorClient(
            ip=self.airsim_host,
            port=self.airsim_port,
            timeout_value=self.rpc_timeout,
        )
        collision = self.client.simGetCollisionInfo(vehicle_name=self.vehicle_name)
        self.collision_baseline_timestamp = collision.time_stamp
        self.collision_monitoring_started = False
        self.collision_free = True
        self.collision_object = None
        self.collision_timestamp = None
        self.last_collision_query = 0.0

    def load_geofence(self):
        try:
            with open(self.geofence_file, "r", encoding="utf-8") as stream:
                report = json.load(stream)
            bounds = report.get("safety_geofence_enu")
            required = ("min_x", "max_x", "min_y", "max_y")
            if isinstance(bounds, dict) and all(key in bounds for key in required):
                return {key: float(bounds[key]) for key in required}
        except (OSError, ValueError, TypeError):
            pass
        return None

    def point_inside_geofence(self, point):
        if self.geofence is None:
            return True
        if point is None:
            return False
        return (
            self.geofence["min_x"] <= point.x <= self.geofence["max_x"]
            and self.geofence["min_y"] <= point.y <= self.geofence["max_y"]
        )

    def state_callback(self, msg):
        with self.lock:
            self.state = msg

    def odom_callback(self, msg):
        now = time.monotonic()
        with self.lock:
            self.odom = msg
            if abs(msg.pose.pose.position.z - self.fixed_height) <= self.height_tolerance:
                if self.height_reached_since is None:
                    self.height_reached_since = now
            else:
                self.height_reached_since = None
            if self.state and self.state.armed:
                current_xy = (msg.pose.pose.position.x, msg.pose.pose.position.y)
                if self.armed_start_xy is None:
                    self.armed_start_xy = current_xy
                self.max_horizontal_travel = max(
                    self.max_horizontal_travel,
                    math.hypot(
                        current_xy[0] - self.armed_start_xy[0],
                        current_xy[1] - self.armed_start_xy[1],
                    ),
                )
                if not self.takeoff_phase_completed:
                    self.max_observed_takeoff_xy_drift = max(
                        self.max_observed_takeoff_xy_drift,
                        math.hypot(
                            current_xy[0] - self.armed_start_xy[0],
                            current_xy[1] - self.armed_start_xy[1],
                        ),
                    )
                    if (
                        self.height_reached_since is not None
                        and now - self.height_reached_since
                        >= self.height_hold_seconds
                    ):
                        self.takeoff_phase_completed = True
                quaternion = msg.pose.pose.orientation
                sin_roll = 2.0 * (
                    quaternion.w * quaternion.x + quaternion.y * quaternion.z
                )
                cos_roll = 1.0 - 2.0 * (
                    quaternion.x * quaternion.x + quaternion.y * quaternion.y
                )
                roll = math.atan2(sin_roll, cos_roll)
                sin_pitch = 2.0 * (
                    quaternion.w * quaternion.y - quaternion.z * quaternion.x
                )
                pitch = math.asin(max(-1.0, min(1.0, sin_pitch)))
                tilt = math.degrees(
                    math.acos(
                        max(-1.0, min(1.0, math.cos(roll) * math.cos(pitch)))
                    )
                )
                self.attitude_tilt_samples.append(tilt)
                if self.takeoff_phase_completed and self.command_velocity is not None:
                    velocity = self.command_velocity.twist.linear
                    if math.hypot(velocity.x, velocity.y) >= self.heading_sample_min_speed:
                        sin_yaw = 2.0 * (
                            quaternion.w * quaternion.z
                            + quaternion.x * quaternion.y
                        )
                        cos_yaw = 1.0 - 2.0 * (
                            quaternion.y * quaternion.y
                            + quaternion.z * quaternion.z
                        )
                        actual_yaw = math.atan2(sin_yaw, cos_yaw)
                        motion_yaw = math.atan2(velocity.y, velocity.x)
                        yaw_error = math.atan2(
                            math.sin(motion_yaw - actual_yaw),
                            math.cos(motion_yaw - actual_yaw),
                        )
                        self.heading_error_samples.append(abs(math.degrees(yaw_error)))

    def cloud_callback(self, msg):
        with self.lock:
            self.cloud = msg

    def waypoint_callback(self, msg):
        with self.lock:
            self.waypoint = msg

    def safe_waypoint_callback(self, msg):
        with self.lock:
            self.safe_waypoint = msg

    def local_planner_status_callback(self, msg):
        with self.lock:
            self.local_planner_status = msg.data

    def local_planner_blocked_callback(self, msg):
        with self.lock:
            self.local_planner_blocked = bool(msg.data)

    def setpoint_callback(self, msg):
        now = time.monotonic()
        with self.lock:
            self.setpoint = msg
            self.setpoint_times.append(now)

    def command_velocity_callback(self, msg):
        velocity = msg.twist.linear
        xy_speed = math.hypot(velocity.x, velocity.y)
        with self.lock:
            self.command_velocity = msg
            self.max_observed_command_xy_speed = max(
                self.max_observed_command_xy_speed, xy_speed
            )

    def exploration_finish_callback(self, msg):
        with self.lock:
            self.exploration_finished = bool(msg.data)

    def attitude_statistics(self):
        with self.lock:
            samples = list(self.attitude_tilt_samples)
        if not samples:
            return 0, None, None
        samples.sort()
        index = max(0, min(len(samples) - 1, math.ceil(0.95 * len(samples)) - 1))
        return len(samples), samples[index], samples[-1]

    def heading_statistics(self):
        with self.lock:
            samples = list(self.heading_error_samples)
        if not samples:
            return 0, None, None
        samples.sort()
        index = max(0, min(len(samples) - 1, math.ceil(0.95 * len(samples)) - 1))
        return len(samples), samples[index], samples[-1]

    def setpoint_rate(self):
        now = time.monotonic()
        with self.lock:
            recent = [stamp for stamp in self.setpoint_times if now - stamp <= 5.0]
        if len(recent) < 2:
            return 0.0
        return (len(recent) - 1) / (recent[-1] - recent[0])

    def airsim_position_enu(self):
        state = self.client.simGetGroundTruthKinematics(vehicle_name=self.vehicle_name)
        position = state.position
        values = (
            position.y_val,
            position.x_val,
            -position.z_val + self.world_to_map_z_offset,
        )
        if not all(math.isfinite(value) for value in values):
            raise RuntimeError("AirSim returned a non-finite ground-truth position")
        return values

    def update_collision_status(self):
        now = time.monotonic()
        if now - self.last_collision_query < 1.0:
            return
        self.last_collision_query = now
        try:
            collision = self.client.simGetCollisionInfo(vehicle_name=self.vehicle_name)
            with self.lock:
                vehicle_armed = bool(self.state and self.state.armed)

            # AirSim may preserve the previous collision timestamp across
            # reset(). Keep advancing the baseline until the first armed
            # sample, then only treat newer timestamps as flight collisions.
            if self.expect_armed and not vehicle_armed:
                self.collision_baseline_timestamp = max(
                    self.collision_baseline_timestamp,
                    collision.time_stamp,
                )
                return
            if not self.collision_monitoring_started:
                self.collision_baseline_timestamp = max(
                    self.collision_baseline_timestamp,
                    collision.time_stamp,
                )
                self.collision_monitoring_started = True
                return
            if (
                collision.has_collided
                and collision.time_stamp > self.collision_baseline_timestamp
            ):
                self.collision_free = False
                self.collision_object = collision.object_name
                self.collision_timestamp = collision.time_stamp
        except Exception as exc:  # AirSim RPC exceptions vary by msgpack version.
            self.collision_free = False
            self.collision_object = "RPC error: " + str(exc)

    def evaluate(self):
        self.update_collision_status()
        with self.lock:
            state = self.state
            odom = self.odom
            cloud = self.cloud
            waypoint = self.waypoint
            safe_waypoint = self.safe_waypoint
            local_planner_status = self.local_planner_status
            local_planner_blocked = self.local_planner_blocked
            setpoint = self.setpoint
            height_reached_since = self.height_reached_since
            command_velocity = self.command_velocity
            max_observed_command_xy_speed = self.max_observed_command_xy_speed
            max_horizontal_travel = self.max_horizontal_travel
            max_observed_takeoff_xy_drift = self.max_observed_takeoff_xy_drift
            takeoff_phase_completed = self.takeoff_phase_completed
            exploration_finished = self.exploration_finished

        attitude_count, tilt_p95, tilt_peak = self.attitude_statistics()
        heading_count, heading_p95, heading_peak = self.heading_statistics()
        height_hold_elapsed = (
            0.0
            if height_reached_since is None
            else time.monotonic() - height_reached_since
        )
        attitude_stable = not self.expect_armed or (
            attitude_count >= self.min_attitude_samples
            and tilt_p95 <= self.max_tilt_p95_deg
            and tilt_peak <= self.max_tilt_peak_deg
        )
        nose_forward = not self.expect_armed or (
            heading_count >= self.min_heading_samples
            and heading_p95 <= self.max_heading_error_p95_deg
            and heading_peak <= self.max_heading_error_peak_deg
        )

        checks = {
            "mavros_connected": bool(state and state.connected),
            "odom_received": odom is not None,
            "odom_frame_map": bool(odom and odom.header.frame_id == "map"),
            "cloud_received": cloud is not None,
            "cloud_frame_map": bool(cloud and cloud.header.frame_id == "map"),
            "cloud_has_points": bool(
                cloud and cloud.width * cloud.height >= self.min_cloud_points
            ),
            "waypoint_received": waypoint is not None,
            "waypoint_fixed_height": bool(
                waypoint
                and abs(waypoint.point.z - self.fixed_height)
                <= self.height_tolerance
            ),
            "waypoint_inside_geofence": self.point_inside_geofence(
                waypoint.point if waypoint else None
            ),
            "local_planner_status_received": local_planner_status is not None,
            "safe_waypoint_received": safe_waypoint is not None,
            "safe_waypoint_fixed_height": bool(
                safe_waypoint
                and abs(safe_waypoint.point.z - self.fixed_height)
                <= self.height_tolerance
            ),
            "safe_waypoint_inside_geofence": self.point_inside_geofence(
                safe_waypoint.point if safe_waypoint else None
            ),
            "setpoint_received": setpoint is not None,
            "setpoint_frame_map": bool(
                setpoint and setpoint.header.frame_id == "map"
            ),
            "setpoint_fixed_height": bool(
                setpoint
                and abs(setpoint.pose.position.z - self.fixed_height)
                <= self.height_tolerance
            ),
            "setpoint_inside_geofence": self.point_inside_geofence(
                setpoint.pose.position if setpoint else None
            ),
            "setpoint_rate": self.setpoint_rate() >= self.min_setpoint_rate,
            "command_velocity_received": command_velocity is not None,
            "command_speed_limited": bool(
                command_velocity
                and max_observed_command_xy_speed <= self.max_command_xy_speed
            ),
            "offboard_mode": bool(
                state and (state.mode == "OFFBOARD" or not self.expect_offboard)
            ),
            "vehicle_armed": bool(
                state and (state.armed or not self.expect_armed)
            ),
            "fixed_height_reached": bool(
                height_hold_elapsed >= self.height_hold_seconds or not self.expect_armed
            ),
            "attitude_stable": attitude_stable,
            "nose_forward": nose_forward,
            "exploration_travelled": bool(
                max_horizontal_travel >= self.min_horizontal_travel
                or not self.expect_armed
            ),
            "exploration_not_premature": bool(
                not exploration_finished
                or max_horizontal_travel >= self.min_horizontal_travel
                or not self.expect_armed
            ),
            "vertical_takeoff_before_exploration": bool(
                (
                    takeoff_phase_completed
                    and max_observed_takeoff_xy_drift <= self.max_takeoff_xy_drift
                )
                or not self.expect_armed
            ),
            "airsim_collision_free": self.collision_free,
        }

        details = {
            "mode": state.mode if state else None,
            "armed": state.armed if state else None,
            "odom_frame": odom.header.frame_id if odom else None,
            "odom_position_enu": (
                [
                    odom.pose.pose.position.x,
                    odom.pose.pose.position.y,
                    odom.pose.pose.position.z,
                ]
                if odom
                else None
            ),
            "cloud_frame": cloud.header.frame_id if cloud else None,
            "cloud_points": cloud.width * cloud.height if cloud else 0,
            "waypoint_z": waypoint.point.z if waypoint else None,
            "waypoint_xy": (
                [waypoint.point.x, waypoint.point.y] if waypoint else None
            ),
            "safe_waypoint_xy": (
                [safe_waypoint.point.x, safe_waypoint.point.y]
                if safe_waypoint
                else None
            ),
            "safe_waypoint_z": safe_waypoint.point.z if safe_waypoint else None,
            "local_planner_status": local_planner_status,
            "local_planner_blocked": local_planner_blocked,
            "setpoint_frame": setpoint.header.frame_id if setpoint else None,
            "setpoint_z": setpoint.pose.position.z if setpoint else None,
            "setpoint_xy": (
                [setpoint.pose.position.x, setpoint.pose.position.y]
                if setpoint
                else None
            ),
            "setpoint_rate_hz": round(self.setpoint_rate(), 2),
            "max_command_xy_speed_mps": round(max_observed_command_xy_speed, 3),
            "max_horizontal_travel_m": round(max_horizontal_travel, 3),
            "exploration_finished": exploration_finished,
            "max_takeoff_xy_drift_m": round(max_observed_takeoff_xy_drift, 3),
            "takeoff_phase_completed": takeoff_phase_completed,
            "attitude_samples": attitude_count,
            "tilt_p95_deg": None if tilt_p95 is None else round(tilt_p95, 2),
            "tilt_peak_deg": None if tilt_peak is None else round(tilt_peak, 2),
            "heading_samples": heading_count,
            "heading_error_p95_deg": (
                None if heading_p95 is None else round(heading_p95, 2)
            ),
            "heading_error_peak_deg": (
                None if heading_peak is None else round(heading_peak, 2)
            ),
            "height_hold_seconds": round(height_hold_elapsed, 2),
            "collision_object": self.collision_object,
            "collision_timestamp": self.collision_timestamp,
            "collision_baseline_timestamp": self.collision_baseline_timestamp,
            "collision_monitoring_started": self.collision_monitoring_started,
            "safety_geofence_enu": self.geofence,
        }
        return checks, details

    def add_coordinate_check(self, checks, details):
        try:
            airsim_position = self.airsim_position_enu()
            with self.lock:
                odom = self.odom
            if odom is None:
                checks["airsim_mavros_coordinates"] = False
                return

            mavros_position = (
                odom.pose.pose.position.x,
                odom.pose.pose.position.y,
                odom.pose.pose.position.z,
            )
            error = math.sqrt(
                sum(
                    (airsim_position[index] - mavros_position[index]) ** 2
                    for index in range(3)
                )
            )
            checks["airsim_mavros_coordinates"] = error <= self.coordinate_tolerance
            details["airsim_position_enu"] = list(airsim_position)
            details["airsim_mavros_position_error_m"] = round(error, 3)
        except Exception as exc:  # AirSim RPC exceptions vary by msgpack version.
            checks["airsim_mavros_coordinates"] = False
            details["airsim_coordinate_error"] = str(exc)

    def write_report(self, passed, checks, details, elapsed):
        report = {
            "passed": passed,
            "elapsed_seconds": round(elapsed, 2),
            "checks": checks,
            "details": details,
        }
        report_dir = os.path.dirname(self.report_path)
        if report_dir:
            os.makedirs(report_dir, exist_ok=True)
        with open(self.report_path, "w", encoding="utf-8") as report_file:
            json.dump(report, report_file, indent=2, sort_keys=True)
            report_file.write("\n")
        return report

    @staticmethod
    def print_report(report):
        for name, passed in report["checks"].items():
            print("[{}] {}".format("PASS" if passed else "FAIL", name))
        print(json.dumps(report["details"], indent=2, sort_keys=True))
        print("Report: {}".format(report.get("report_path", "")))

    def run(self):
        started = time.monotonic()
        next_status = started
        rate = rospy.Rate(5.0)

        while not rospy.is_shutdown():
            elapsed = time.monotonic() - started
            checks, details = self.evaluate()
            checks["minimum_observation"] = elapsed >= self.min_observation_seconds
            details["observation_seconds"] = round(elapsed, 2)
            if elapsed >= next_status - started:
                missing = [name for name, passed in checks.items() if not passed]
                rospy.loginfo(
                    "Blocks smoke test: %.0fs elapsed, waiting for %s",
                    elapsed,
                    ", ".join(missing) if missing else "coordinate validation",
                )
                next_status += 10.0

            if all(checks.values()):
                self.add_coordinate_check(checks, details)
                if all(checks.values()):
                    report = self.write_report(True, checks, details, elapsed)
                    report["report_path"] = self.report_path
                    self.print_report(report)
                    return 0

            if (
                elapsed >= self.min_observation_seconds
                and details["exploration_finished"]
                and not checks["exploration_travelled"]
            ):
                rospy.logerr(
                    "Exploration finished after only %.2f m (required %.2f m)",
                    details["max_horizontal_travel_m"],
                    self.min_horizontal_travel,
                )
                self.add_coordinate_check(checks, details)
                report = self.write_report(False, checks, details, elapsed)
                report["report_path"] = self.report_path
                self.print_report(report)
                return 1

            if elapsed >= self.timeout:
                self.add_coordinate_check(checks, details)
                report = self.write_report(False, checks, details, elapsed)
                report["report_path"] = self.report_path
                self.print_report(report)
                return 1
            rate.sleep()


if __name__ == "__main__":
    rospy.init_node("blocks_smoke_test", anonymous=True)
    sys.exit(BlocksSmokeTest().run())
