#!/usr/bin/env python3

import unittest
import xml.etree.ElementTree as ET
from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
LAUNCH_PATH = PACKAGE_ROOT / "launch" / "tare_uav_fixed_height.launch"
CONFIG_PATH = PACKAGE_ROOT / "config" / "uav_fixed_height.yaml"
RVIZ_PATH = PACKAGE_ROOT / "rviz" / "vehicle_simulator.rviz"
PLANNER_SOURCE_PATH = (PACKAGE_ROOT / "src" / "sensor_coverage_planner" /
                       "sensor_coverage_planner_ground.cpp")


def simple_yaml_values(path):
    values = {}
    for raw_line in path.read_text(encoding="utf-8").splitlines():
        line = raw_line.split("#", 1)[0].strip()
        if not line or ":" not in line:
            continue
        key, value = line.split(":", 1)
        values[key.strip()] = value.strip()
    return values


class UavManualNavigationProfileTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.launch_root = ET.parse(LAUNCH_PATH).getroot()
        cls.args = {node.attrib["name"]: node.attrib.get("default", "")
                    for node in cls.launch_root.findall("arg")}
        cls.config = simple_yaml_values(CONFIG_PATH)
        cls.rviz = RVIZ_PATH.read_text(encoding="utf-8")
        cls.planner_source = PLANNER_SOURCE_PATH.read_text(encoding="utf-8")

    def test_standard_rviz_goal_is_wired_to_tare(self):
        self.assertEqual(self.args["manual_goal_topic"], "/move_base_simple/goal")
        self.assertEqual(self.config["sub_manual_goal_topic_"], "/move_base_simple/goal")
        self.assertIn("Class: rviz/SetGoal", self.rviz)
        self.assertIn("Topic: /move_base_simple/goal", self.rviz)
        self.assertNotIn("Class: rviz/WaypointTool", self.rviz)

    def test_manual_navigation_is_enabled_with_conservative_defaults(self):
        self.assertEqual(self.args["enable_manual_goal_navigation"], "true")
        self.assertEqual(self.args["manual_goal_arrival_radius"], "0.35")
        self.assertEqual(self.args["manual_goal_max_graph_distance"], "2.0")
        self.assertEqual(self.args["manual_goal_clearance"], "$(arg uav_horizontal_clearance)")
        self.assertEqual(self.args["manual_goal_vertical_clearance"], "$(arg uav_vertical_clearance)")
        self.assertEqual(self.config["kEnableManualGoalNavigation"], "true")

    def test_tare_outputs_continue_through_local_avoidance(self):
        tare_node = next(node for node in self.launch_root.iter("node")
                         if node.attrib.get("type") == "tare_planner_node")
        tare_params = {node.attrib["name"]: node.attrib.get("value", "")
                       for node in tare_node.findall("param")}
        local_node = next(node for node in self.launch_root.iter("node")
                          if node.attrib.get("type") == "uav_local_planner_node")
        local_params = {node.attrib["name"]: node.attrib.get("value", "")
                        for node in local_node.findall("param")}

        self.assertEqual(tare_params["sub_manual_goal_topic_"], "$(arg manual_goal_topic)")
        self.assertEqual(tare_params["pub_waypoint_topic_"], "$(arg waypoint_topic)")
        self.assertEqual(local_params["raw_waypoint_topic"], "$(arg local_planner_waypoint_topic)")
        self.assertEqual(local_params["reference_path_topic"],
                         "$(arg local_planner_reference_path_topic)")
        self.assertEqual(local_params["safe_waypoint_topic"], "$(arg safe_waypoint_topic)")

    def test_navigation_visualization_and_state_topics_are_present(self):
        expected_topics = (
            "/tare_uav/navigation/path",
            "/tare_uav/navigation/goal",
            "/tare_uav/mission/mode",
            "/tare_uav/navigation/active",
            "/tare_uav/navigation/reached",
        )
        config_text = CONFIG_PATH.read_text(encoding="utf-8")
        launch_text = LAUNCH_PATH.read_text(encoding="utf-8")
        for topic in expected_topics:
            self.assertIn(topic, config_text)
            self.assertIn(topic, launch_text)
        self.assertIn("Topic: /tare_uav/navigation/path", self.rviz)
        self.assertIn("Topic: /tare_uav/navigation/goal", self.rviz)
        self.assertIn("Name: TAREExploredMap", self.rviz)
        self.assertIn("Topic: /sensor_coverage_planner/planner_cloud", self.rviz)

    def test_core_uses_connected_graph_and_goal_clearance(self):
        self.assertIn("MissionMode::NAVIGATION", self.planner_source)
        self.assertIn("GetClosestConnectedNodeIndAndDistance", self.planner_source)
        self.assertIn("GetShortestPath(start, goal, true, graph_path, true)",
                      self.planner_source)
        self.assertIn("ManualGoalHasClearance", self.planner_source)
        self.assertIn("final connection from the explored graph to the goal is obstructed",
                      self.planner_source)


if __name__ == "__main__":
    unittest.main()
