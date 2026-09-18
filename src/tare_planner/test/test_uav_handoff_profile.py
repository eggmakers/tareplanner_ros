#!/usr/bin/env python3

import unittest
import xml.etree.ElementTree as ET
from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
BASE_LAUNCH = PACKAGE_ROOT / "launch" / "tare_uav_fixed_height.launch"
MISSION_LAUNCH = PACKAGE_ROOT / "launch" / "tare_uav_integrated_mission.launch"
PLANNER_HEADER = (PACKAGE_ROOT / "include" / "sensor_coverage_planner" /
                  "sensor_coverage_planner_ground.h")
PLANNER_SOURCE = (PACKAGE_ROOT / "src" / "sensor_coverage_planner" /
                  "sensor_coverage_planner_ground.cpp")


class UavHandoffProfileTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.base_root = ET.parse(BASE_LAUNCH).getroot()
        cls.mission_root = ET.parse(MISSION_LAUNCH).getroot()
        cls.base_text = BASE_LAUNCH.read_text(encoding="utf-8")
        cls.mission_text = MISSION_LAUNCH.read_text(encoding="utf-8")
        cls.header_text = PLANNER_HEADER.read_text(encoding="utf-8")
        cls.source_text = PLANNER_SOURCE.read_text(encoding="utf-8")

    def test_fixed_start_planning_is_planning_only(self):
        self.assertIn("FixedStartGoalCallback", self.header_text)
        self.assertIn("fixed_start_path_pub_.publish(path)", self.source_text)
        callback = self.source_text.split(
            "void SensorCoveragePlanner3D::FixedStartGoalCallback", 1)[1].split(
                "void SensorCoveragePlanner3D::PauseMissionCallback", 1)[0]
        self.assertNotIn("waypoint_pub_.publish", callback)
        self.assertNotIn("MissionMode::NAVIGATION", callback)

    def test_manual_and_fixed_start_use_the_same_graph_builder(self):
        self.assertIn("BuildGraphNavigationPath(pd_.robot_position_, manual_goal_",
                      self.source_text)
        self.assertIn("BuildGraphNavigationPath(start, goal, path, failure_reason)",
                      self.source_text)
        self.assertIn("GetFixedFlightHeightOr(start.z)", self.source_text)

    def test_integrated_launch_has_one_height_argument(self):
        args = {node.attrib["name"]: node.attrib.get("default", "")
                for node in self.mission_root.findall("arg")}
        self.assertIn("fixed_flight_height", args)
        self.assertIn("fixed_flight_height_relative_to_start", args)
        for node in self.mission_root.iter("param"):
            if node.attrib.get("name") == "fixed_flight_height":
                self.assertEqual(node.attrib.get("value"),
                                 "$(arg fixed_flight_height)")
            if node.attrib.get("name") == "fixed_flight_height_relative_to_start":
                self.assertEqual(
                    node.attrib.get("value"),
                    "$(arg fixed_flight_height_relative_to_start)")

    def test_mission_mux_is_the_only_integrated_local_planner_source(self):
        self.assertIn(
            '<arg name="waypoint_topic" value="/tare_uav/source/tare/waypoint"/>',
            self.mission_text)
        self.assertIn(
            '<arg name="local_planner_waypoint_topic" value="/tare_uav/command/waypoint"/>',
            self.mission_text)
        self.assertIn('type="uav_mission_mux.py"', self.mission_text)

    def test_uav_replayer_never_publishes_twist(self):
        replayer = (PACKAGE_ROOT / "scripts" /
                    "uav_path_replayer.py").read_text(encoding="utf-8")
        self.assertNotIn("geometry_msgs.msg import Twist", replayer)
        self.assertNotIn("/cmd_vel", replayer)
        self.assertIn("fixed_flight_height", replayer)

    def test_saved_map_astar_uses_height_and_clearance(self):
        source = (PACKAGE_ROOT / "src" / "uav_saved_map_planner" /
                  "uav_saved_map_planner_node.cpp").read_text(encoding="utf-8")
        self.assertIn('"fixed_flight_height"', source)
        self.assertIn('"horizontal_clearance"', source)
        self.assertIn("RunAStar", source)
        self.assertIn("point.z = FlightHeight()", source)


if __name__ == "__main__":
    unittest.main()
