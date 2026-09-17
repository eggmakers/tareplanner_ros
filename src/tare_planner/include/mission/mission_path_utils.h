/*
 * mission_path_utils.h
 *
 * 自研任务层的路径与定高工具函数。
 *
 * 这些逻辑原先直接写在 SensorCoveragePlanner3D 的成员函数里，与 TARE 的探索流程
 * 混在同一个 2000 行文件中。抽出来之后：
 *   - 不再依赖 SensorCoveragePlanner3D 的私有状态，只依赖显式入参
 *   - 可以脱离 ROS 节点单独测试（见 test/test_mission_path_utils.cpp）
 *   - SensorCoveragePlanner3D 只保留同名薄委托，调用点无需改动
 *
 * 抽离原则: **零行为变化**，只把 pp_ / pd_ 的隐式访问改成显式入参。
 */
#pragma once

#include <string>

#include <geometry_msgs/Point.h>
#include <nav_msgs/Path.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "keypose_graph/keypose_graph.h"
#include "mission/mission_config.h"

namespace mission_ns
{

using PointICloud = pcl::PointCloud<pcl::PointXYZI>;
using PointICloudConstPtr = PointICloud::ConstPtr;

/// 固定飞行高度。
///
/// - cfg.kUseFixedFlightHeight 为 false → 返回 fallback_z（保持 TARE 原始行为）
/// - 为 true → 返回 cfg.kFixedFlightHeight（可选叠加起飞点高度 initial_z）
double FixedFlightHeight(const MissionConfig& cfg, double fallback_z, double initial_z);

/// 目标点周围是否有足够安全余量。
///
/// 在 goal 的垂直带（±kManualGoalVerticalClearance）内，统计水平距离在
/// kManualGoalClearance 之内的点数；任一点云里达到 2 个即判定余量不足。
bool GoalHasClearance(const MissionConfig& cfg,
                      const geometry_msgs::Point& goal,
                      const PointICloudConstPtr& collision_cloud,
                      const PointICloudConstPtr& registered_cloud);

/// 在已探索的 keypose 连通图上，规划一条从 start 到 goal 的路径。
///
/// 步骤:
///   1. 把 goal 吸附到最近的连通图节点，超出 kManualGoalMaxGraphDistance 则拒绝
///   2. 检查"图节点 → goal"这段直线是否被障碍切断
///   3. 用图最短路算法求路径，并把所有点的高度统一到固定飞行高度
///
/// @param initial_z 起飞点高度，用于 kFixedFlightHeightRelativeToStart
/// @param frame_id  路径坐标系，由调用方传入（= TARE 的 kWorldFrameID），避免两处硬编码
/// @param failure_reason 失败时写入可读原因（用于状态话题）
bool BuildGraphNavigationPath(const MissionConfig& cfg,
                              double initial_z,
                              const std::string& frame_id,
                              keypose_graph_ns::KeyposeGraph* keypose_graph,
                              const PointICloudConstPtr& collision_cloud,
                              const PointICloudConstPtr& registered_cloud,
                              const geometry_msgs::Point& start,
                              const geometry_msgs::Point& goal,
                              nav_msgs::Path& path,
                              std::string& failure_reason);

}  // namespace mission_ns
