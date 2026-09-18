#!/usr/bin/env python3
"""手动导航 / RViz 2D Nav Goal 接线一致性检查。

P2 重构后 launch 被拆成 launch/include/_*.launch，参数默认值下沉到
config/uav/*.yaml，因此本测试改为:

  * walk_launch(): 递归读取 launch 树（主文件 + 所有 <include>）收集
    arg 与每个节点的 <param>（忽略 if/unless 条件，全部收集）
  * simple_yaml_values(): 合并读取 config/uav/*.yaml 作为默认值真源

断言内容与重构前保持一致：RViz 的 2D Nav Goal 接到 TARE 手动导航，
TARE 航点经局部避障转发，且默认参数保守。
"""

import unittest
import xml.etree.ElementTree as ET
from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
MAIN_LAUNCH = PACKAGE_ROOT / "launch" / "tare_uav_fixed_height.launch"
UAV_CONFIG_DIR = PACKAGE_ROOT / "config" / "uav"
RVIZ_PATH = PACKAGE_ROOT / "rviz" / "vehicle_simulator.rviz"
PLANNER_SOURCE_PATH = (PACKAGE_ROOT / "src" / "sensor_coverage_planner" /
                       "sensor_coverage_planner_ground.cpp")
MISSION_SOURCE_DIR = PACKAGE_ROOT / "src" / "mission"

FIND_PREFIX = "$(find tare_planner)/"


def simple_yaml_values(path):
    """只解析扁平的 key: value 行（够用，且不引入 pyyaml 依赖）。

    注意: 多个文件合并成同一个 dict，因此只适合查询跨文件唯一的键
    （话题名、k* 参数），不要用它查询 odometry_topic 这类多文件重名的键。
    """
    values = {}
    for raw_line in path.read_text(encoding="utf-8").splitlines():
        line = raw_line.split("#", 1)[0].strip()
        if not line or ":" not in line:
            continue
        key, value = line.split(":", 1)
        values[key.strip()] = value.strip()
    return values


def resolve_include(path_text):
    """$(find tare_planner)/launch/include/x.launch -> 绝对路径"""
    if FIND_PREFIX not in path_text:
        return None
    return PACKAGE_ROOT / path_text.split(FIND_PREFIX, 1)[1]


def walk_launch(path, seen):
    """递归收集整个 launch 树的 arg 与各节点的 param。

    返回 (args, params)，其中 params 形如 {node_type: {param_name: value}}。
    """
    path = Path(path).resolve()
    if path in seen or not path.exists():
        return {}, {}
    seen.add(path)

    root = ET.parse(path).getroot()
    args = {}
    params = {}

    for node in root.findall("arg"):
        args.setdefault(node.attrib["name"], node.attrib.get("default", ""))

    for node in root.iter("node"):
        bucket = params.setdefault(node.attrib.get("type", ""), {})
        for param in node.findall("param"):
            bucket[param.attrib["name"]] = param.attrib.get("value", "")

    for node in root.iter("include"):
        target = resolve_include(node.attrib.get("file", ""))
        if target is None:
            continue
        sub_args, sub_params = walk_launch(target, seen)
        for key, value in sub_args.items():
            args.setdefault(key, value)
        for node_type, bucket in sub_params.items():
            params.setdefault(node_type, {}).update(bucket)

    return args, params


def merged_config_text():
    return "\n".join(path.read_text(encoding="utf-8")
                     for path in sorted(UAV_CONFIG_DIR.glob("*.yaml")))


class UavManualNavigationProfileTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.args, cls.params = walk_launch(MAIN_LAUNCH, set())
        cls.config = {}
        for path in sorted(UAV_CONFIG_DIR.glob("*.yaml")):
            cls.config.update(simple_yaml_values(path))
        cls.config_text = merged_config_text()
        cls.rviz = RVIZ_PATH.read_text(encoding="utf-8")
        cls.planner_source = PLANNER_SOURCE_PATH.read_text(encoding="utf-8")
        # P3b 把连通图搜索 / 目标净空判断 / 定高计算搬到了 mission/ 模块。
        # 这里把任务层源码一并纳入检查，断言意图（核心使用连通图搜索）不变。
        cls.runtime_source = cls.planner_source + "\n" + "\n".join(
            path.read_text(encoding="utf-8") for path in sorted(MISSION_SOURCE_DIR.glob("*.cpp")))

    # ---------------------------------------------------------------- 接线
    def test_standard_rviz_goal_is_wired_to_tare(self):
        # 命令行覆盖入口仍然存在（默认空 = 用 yaml）
        self.assertIn("manual_goal_topic", self.args)
        self.assertEqual(self.args["manual_goal_topic"], "")
        # 默认值来自 config/uav/tare_topics.yaml
        self.assertEqual(self.config["sub_manual_goal_topic_"], "/move_base_simple/goal")

        self.assertIn("Class: rviz/SetGoal", self.rviz)
        self.assertIn("Topic: /move_base_simple/goal", self.rviz)
        self.assertNotIn("Class: rviz/WaypointTool", self.rviz)

    # -------------------------------------------------------------- 默认值
    def test_manual_navigation_is_enabled_with_conservative_defaults(self):
        self.assertEqual(self.config["kEnableManualGoalNavigation"], "true")
        self.assertEqual(self.config["kManualGoalArrivalRadius"], "0.35")
        self.assertEqual(self.config["kManualGoalStableSeconds"], "2.0")
        self.assertEqual(self.config["kManualGoalMaxGraphDistance"], "2.0")
        # clearance 与 UAV 局部保护共用同一条基线
        self.assertEqual(self.config["kManualGoalClearance"], "0.75")
        self.assertEqual(self.config["kManualGoalVerticalClearance"], "0.35")

        for name in ("enable_manual_goal_navigation", "manual_goal_arrival_radius",
                     "manual_goal_max_graph_distance", "uav_horizontal_clearance",
                     "uav_vertical_clearance"):
            self.assertIn(name, self.args)
            self.assertEqual(self.args[name], "")

    # ------------------------------------------------------- 数据链透传
    def test_tare_outputs_continue_through_local_avoidance(self):
        tare_params = self.params["tare_planner_node"]
        local_params = self.params["uav_local_planner_node"]

        self.assertEqual(tare_params["sub_manual_goal_topic_"], "$(arg manual_goal_topic)")
        self.assertEqual(tare_params["pub_waypoint_topic_"], "$(arg waypoint_topic)")
        self.assertEqual(local_params["raw_waypoint_topic"], "$(arg raw_waypoint_topic)")
        self.assertEqual(local_params["reference_path_topic"], "$(arg reference_path_topic)")
        self.assertEqual(local_params["safe_waypoint_topic"], "$(arg safe_waypoint_topic)")

    # ---------------------------------------------------------- 可视化
    def test_navigation_visualization_and_state_topics_are_present(self):
        expected_topics = (
            "/tare_uav/navigation/path",
            "/tare_uav/navigation/goal",
            "/tare_uav/mission/mode",
            "/tare_uav/navigation/active",
            "/tare_uav/navigation/reached",
        )
        for topic in expected_topics:
            self.assertIn(topic, self.config_text)

        self.assertIn("Topic: /tare_uav/navigation/path", self.rviz)
        self.assertIn("Topic: /tare_uav/navigation/goal", self.rviz)
        self.assertIn("Name: TAREExploredMap", self.rviz)
        self.assertIn("Topic: /sensor_coverage_planner/planner_cloud", self.rviz)

    # ------------------------------------------------------------ 核心逻辑
    def test_core_uses_connected_graph_and_goal_clearance(self):
        self.assertIn("MissionMode::NAVIGATION", self.planner_source)
        self.assertIn("GetClosestConnectedNodeIndAndDistance", self.runtime_source)
        self.assertIn("GetShortestPath(start, goal, true, graph_path, true)",
                      self.runtime_source)


if __name__ == "__main__":
    unittest.main()
