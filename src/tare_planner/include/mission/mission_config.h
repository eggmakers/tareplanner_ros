/*
 * mission_config.h
 *
 * 固定高度 UAV 任务层的全部可配置项。
 *
 * 为什么单独成结构体:
 *   TARE 上游的 PlannerParameters 是"探索算法参数"；本项目的固定高度飞行、
 *   RViz 目标导航、固定起点规划、任务暂停/继续属于**自研任务层**。
 *   把它们混在同一个结构体里，会让上游参数表和自研参数表无法分辨。
 *
 * 为什么 PlannerParameters 继承本结构体:
 *   既有代码里有 40 多处 pp_.kManualGoalXxx / pp_.sub_manual_goal_topic_ 访问，
 *   通过继承可以让这些访问点**一行都不用改**，把重构风险压到最低。
 *
 * ROS 参数名与默认值保持与重构前完全一致，因此
 * config/uav/tare_mission.yaml 与 tare_topics.yaml 无需任何改动。
 */
#pragma once

#include <string>

#include <ros/ros.h>

namespace mission_ns
{

struct MissionConfig
{
  // ============================ 话题: 订阅 ============================
  std::string sub_manual_goal_topic_;          // RViz 2D Nav Goal
  std::string sub_fixed_start_goal_topic_;     // 固定起点规划目标
  std::string sub_pause_mission_topic_;        // 暂停任务
  std::string sub_resume_exploration_topic_;   // 恢复探索
  std::string sub_cancel_navigation_topic_;    // 取消导航

  // ============================ 话题: 发布 ============================
  std::string pub_manual_navigation_path_topic_;
  std::string pub_manual_navigation_goal_topic_;
  std::string pub_mission_mode_topic_;
  std::string pub_navigation_active_topic_;
  std::string pub_navigation_reached_topic_;
  std::string pub_fixed_start_path_topic_;
  std::string pub_fixed_start_goal_topic_;
  std::string pub_fixed_start_status_topic_;
  std::string pub_exploration_start_pose_topic_;

  // ============================== 开关 ==============================
  bool kUseFixedFlightHeight;                // 定高飞行
  bool kFixedFlightHeightRelativeToStart;    // 高度相对起飞点
  bool kEnableManualGoalNavigation;          // 允许 RViz 目标导航
  bool kEnableFixedStartPlanning;            // 允许固定起点规划

  // ========================= 定高与任务时序 =========================
  double kFixedFlightHeight;
  double kExplorationWarmupSeconds;
  double kExplorationCompletionConfirmSeconds;

  // ========================== 手动目标导航 ==========================
  double kManualGoalArrivalRadius;
  double kManualGoalStableSeconds;
  double kManualGoalMaxGraphDistance;
  double kManualGoalClearance;
  double kManualGoalVerticalClearance;

  /// 从 ROS 参数服务器加载。
  ///
  /// @param nh 目标节点句柄（应为私有句柄，与其他参数一致）
  /// @return 参数组合非法时返回 false，调用方应终止初始化
  bool LoadFromRos(ros::NodeHandle& nh);
};

}  // namespace mission_ns
