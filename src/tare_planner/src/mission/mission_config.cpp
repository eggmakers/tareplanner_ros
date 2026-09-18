/*
 * mission_config.cpp
 *
 * 实现从 sensor_coverage_planner_ground.cpp 的
 * PlannerParameters::ReadParameters() 中搬出的任务层参数读取。
 *
 * 搬家原则: **零行为变化**。ROS 参数名、默认值、校验条件、错误信息全部保持一致。
 */
#include "mission/mission_config.h"

#include "utils/misc_utils.h"

namespace mission_ns
{

bool MissionConfig::LoadFromRos(ros::NodeHandle& nh)
{
  // ============================== 话题 ==============================
  sub_manual_goal_topic_ =
      misc_utils_ns::getParam<std::string>(nh, "sub_manual_goal_topic_", "/move_base_simple/goal");
  sub_fixed_start_goal_topic_ = misc_utils_ns::getParam<std::string>(
      nh, "sub_fixed_start_goal_topic_", "/tare_uav/fixed_start_goal");
  sub_pause_mission_topic_ =
      misc_utils_ns::getParam<std::string>(nh, "sub_pause_mission_topic_", "/tare_uav/mission/pause");
  sub_resume_exploration_topic_ = misc_utils_ns::getParam<std::string>(
      nh, "sub_resume_exploration_topic_", "/tare_uav/mission/resume_exploration");
  sub_cancel_navigation_topic_ = misc_utils_ns::getParam<std::string>(
      nh, "sub_cancel_navigation_topic_", "/tare_uav/navigation/cancel");

  pub_manual_navigation_path_topic_ = misc_utils_ns::getParam<std::string>(
      nh, "pub_manual_navigation_path_topic_", "/tare_uav/navigation/path");
  pub_manual_navigation_goal_topic_ = misc_utils_ns::getParam<std::string>(
      nh, "pub_manual_navigation_goal_topic_", "/tare_uav/navigation/goal");
  pub_mission_mode_topic_ =
      misc_utils_ns::getParam<std::string>(nh, "pub_mission_mode_topic_", "/tare_uav/mission/mode");
  pub_navigation_active_topic_ = misc_utils_ns::getParam<std::string>(
      nh, "pub_navigation_active_topic_", "/tare_uav/navigation/active");
  pub_navigation_reached_topic_ = misc_utils_ns::getParam<std::string>(
      nh, "pub_navigation_reached_topic_", "/tare_uav/navigation/reached");
  pub_fixed_start_path_topic_ = misc_utils_ns::getParam<std::string>(
      nh, "pub_fixed_start_path_topic_", "/tare_uav/fixed_start_path");
  pub_fixed_start_goal_topic_ = misc_utils_ns::getParam<std::string>(
      nh, "pub_fixed_start_goal_topic_", "/tare_uav/fixed_start_goal_accepted");
  pub_fixed_start_status_topic_ = misc_utils_ns::getParam<std::string>(
      nh, "pub_fixed_start_status_topic_", "/tare_uav/fixed_start_status");
  pub_exploration_start_pose_topic_ = misc_utils_ns::getParam<std::string>(
      nh, "pub_exploration_start_pose_topic_", "/tare_uav/exploration_start_pose");

  // ============================== 开关 ==============================
  kUseFixedFlightHeight = misc_utils_ns::getParam<bool>(nh, "kUseFixedFlightHeight", false);
  kFixedFlightHeightRelativeToStart =
      misc_utils_ns::getParam<bool>(nh, "kFixedFlightHeightRelativeToStart", false);
  kEnableManualGoalNavigation = misc_utils_ns::getParam<bool>(nh, "kEnableManualGoalNavigation", false);
  kEnableFixedStartPlanning = misc_utils_ns::getParam<bool>(nh, "kEnableFixedStartPlanning", true);

  // ========================= 定高与任务时序 =========================
  kFixedFlightHeight = misc_utils_ns::getParam<double>(nh, "kFixedFlightHeight", 1.5);
  kExplorationWarmupSeconds = misc_utils_ns::getParam<double>(nh, "kExplorationWarmupSeconds", 3.0);
  kExplorationCompletionConfirmSeconds =
      misc_utils_ns::getParam<double>(nh, "kExplorationCompletionConfirmSeconds", 8.0);

  // ========================== 手动目标导航 ==========================
  kManualGoalArrivalRadius = misc_utils_ns::getParam<double>(nh, "kManualGoalArrivalRadius", 0.35);
  kManualGoalStableSeconds = misc_utils_ns::getParam<double>(nh, "kManualGoalStableSeconds", 2.0);
  kManualGoalMaxGraphDistance = misc_utils_ns::getParam<double>(nh, "kManualGoalMaxGraphDistance", 2.0);
  kManualGoalClearance = misc_utils_ns::getParam<double>(nh, "kManualGoalClearance", 0.75);
  kManualGoalVerticalClearance = misc_utils_ns::getParam<double>(nh, "kManualGoalVerticalClearance", 0.35);

  if (kEnableManualGoalNavigation &&
      (kManualGoalArrivalRadius <= 0.0 || kManualGoalStableSeconds < 0.0 ||
       kManualGoalMaxGraphDistance <= 0.0 || kManualGoalClearance <= 0.0 ||
       kManualGoalVerticalClearance <= 0.0))
  {
    ROS_ERROR("Manual goal navigation parameters must be positive (stable seconds may be zero)");
    return false;
  }

  return true;
}

}  // namespace mission_ns
