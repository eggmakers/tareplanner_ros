/*
 * mission_path_utils.cpp
 *
 * 从 sensor_coverage_planner_ground.cpp 搬出的三个任务层函数。
 *
 * 搬家原则: **零行为变化**。
 *   pp_.kXxx              -> cfg.kXxx
 *   pd_.keypose_graph_    -> keypose_graph
 *   pd_.collision_cloud_->cloud_    -> collision_cloud
 *   pd_.registered_cloud_->cloud_   -> registered_cloud
 *   GetFixedFlightHeightOr(z)       -> FixedFlightHeight(cfg, z, initial_z)
 *   kWorldFrameID                   -> frame_id
 * 判断逻辑、阈值、失败原因字符串全部保持原样。
 */
#include "mission/mission_path_utils.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>

namespace mission_ns
{

double FixedFlightHeight(const MissionConfig& cfg, double fallback_z, double initial_z)
{
  if (!cfg.kUseFixedFlightHeight)
  {
    return fallback_z;
  }
  const double base_height = cfg.kFixedFlightHeightRelativeToStart ? initial_z : 0.0;
  return base_height + cfg.kFixedFlightHeight;
}

bool GoalHasClearance(const MissionConfig& cfg,
                      const geometry_msgs::Point& goal,
                      const PointICloudConstPtr& collision_cloud,
                      const PointICloudConstPtr& registered_cloud)
{
  int nearby_point_count = 0;
  const double clearance_squared = cfg.kManualGoalClearance * cfg.kManualGoalClearance;
  const auto count_nearby_points = [&](const PointICloudConstPtr& cloud) {
    if (!cloud)
    {
      return;
    }
    for (const auto& point : cloud->points)
    {
      if (std::abs(point.z - goal.z) > cfg.kManualGoalVerticalClearance)
      {
        continue;
      }
      const double dx = point.x - goal.x;
      const double dy = point.y - goal.y;
      if (dx * dx + dy * dy <= clearance_squared && ++nearby_point_count >= 2)
      {
        return;
      }
    }
  };

  count_nearby_points(collision_cloud);
  if (nearby_point_count < 2)
  {
    count_nearby_points(registered_cloud);
  }
  return nearby_point_count < 2;
}

bool BuildGraphNavigationPath(const MissionConfig& cfg,
                              double initial_z,
                              const std::string& frame_id,
                              keypose_graph_ns::KeyposeGraph* keypose_graph,
                              const PointICloudConstPtr& collision_cloud,
                              const PointICloudConstPtr& registered_cloud,
                              const geometry_msgs::Point& start,
                              const geometry_msgs::Point& goal,
                              nav_msgs::Path& path,
                              std::string& failure_reason)
{
  path = nav_msgs::Path();
  path.header.frame_id = frame_id;
  path.header.stamp = ros::Time::now();

  if (keypose_graph == nullptr || keypose_graph->GetConnectedNodeNum() < 1)
  {
    failure_reason = "the explored keypose graph is empty";
    return false;
  }

  int closest_goal_node = -1;
  double distance_to_graph = std::numeric_limits<double>::max();
  keypose_graph->GetClosestConnectedNodeIndAndDistance(goal, closest_goal_node, distance_to_graph);
  if (closest_goal_node < 0 || !std::isfinite(distance_to_graph) ||
      distance_to_graph > cfg.kManualGoalMaxGraphDistance)
  {
    failure_reason = "goal is outside the connected explored area (nearest graph distance=" +
                     std::to_string(distance_to_graph) + " m, limit=" +
                     std::to_string(cfg.kManualGoalMaxGraphDistance) + " m)";
    return false;
  }

  const geometry_msgs::Point graph_anchor = keypose_graph->GetNodePosition(closest_goal_node);
  const double segment_x = goal.x - graph_anchor.x;
  const double segment_y = goal.y - graph_anchor.y;
  const double segment_length_squared = segment_x * segment_x + segment_y * segment_y;
  const double clearance_squared = cfg.kManualGoalClearance * cfg.kManualGoalClearance;
  int segment_collision_points = 0;
  const auto segment_is_clear = [&](const PointICloudConstPtr& cloud) {
    if (!cloud)
    {
      return true;
    }
    for (const auto& point : cloud->points)
    {
      if (std::abs(point.z - goal.z) > cfg.kManualGoalVerticalClearance)
      {
        continue;
      }
      double projection = 0.0;
      if (segment_length_squared > 1e-6)
      {
        projection = ((point.x - graph_anchor.x) * segment_x + (point.y - graph_anchor.y) * segment_y) /
                     segment_length_squared;
        projection = std::max(0.0, std::min(1.0, projection));
      }
      const double nearest_x = graph_anchor.x + projection * segment_x;
      const double nearest_y = graph_anchor.y + projection * segment_y;
      const double dx = point.x - nearest_x;
      const double dy = point.y - nearest_y;
      if (dx * dx + dy * dy <= clearance_squared && ++segment_collision_points >= 2)
      {
        return false;
      }
    }
    return true;
  };
  if (!segment_is_clear(collision_cloud) || !segment_is_clear(registered_cloud))
  {
    failure_reason = "the final connection from the explored graph to the goal is obstructed";
    return false;
  }

  nav_msgs::Path graph_path;
  const double path_length = keypose_graph->GetShortestPath(start, goal, true, graph_path, true);
  if (!std::isfinite(path_length) || path_length >= std::numeric_limits<double>::max() / 2.0 ||
      graph_path.poses.empty())
  {
    failure_reason = "no connected path exists through the explored keypose graph";
    return false;
  }

  const double flight_height = FixedFlightHeight(cfg, start.z, initial_z);
  const auto append_point = [&](const geometry_msgs::Point& point) {
    geometry_msgs::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position = point;
    pose.pose.position.z = flight_height;
    pose.pose.orientation.w = 1.0;
    if (!path.poses.empty())
    {
      const auto& last = path.poses.back().pose.position;
      if (std::hypot(last.x - pose.pose.position.x, last.y - pose.pose.position.y) < 0.05)
      {
        path.poses.back() = pose;
        return;
      }
    }
    path.poses.push_back(pose);
  };

  append_point(start);
  for (const auto& graph_pose : graph_path.poses)
  {
    append_point(graph_pose.pose.position);
  }
  append_point(goal);

  if (path.poses.size() < 2)
  {
    append_point(goal);
  }
  return !path.poses.empty();
}

}  // namespace mission_ns
