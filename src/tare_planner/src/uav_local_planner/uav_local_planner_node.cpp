#include <algorithm>
#include <cmath>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <tf/transform_listener.h>

#include "uav_local_planner/uav_local_planner_core.h"

namespace uav_local_planner_ns
{
class UavLocalPlannerNode
{
public:
  UavLocalPlannerNode() : nh_(), private_nh_("~"), tf_listener_(ros::Duration(20.0))
  {
    LoadParameters();
    ValidateParameters();
    planner_ = std::make_unique<UavLocalPlannerCore>(planner_config_);

    odometry_sub_ = nh_.subscribe(odometry_topic_, 10, &UavLocalPlannerNode::OdometryCallback, this);
    waypoint_sub_ = nh_.subscribe(raw_waypoint_topic_, 5, &UavLocalPlannerNode::WaypointCallback, this);
    path_sub_ = nh_.subscribe(reference_path_topic_, 2, &UavLocalPlannerNode::PathCallback, this);
    collision_cloud_sub_ =
        nh_.subscribe(collision_cloud_topic_, 1, &UavLocalPlannerNode::CollisionCloudCallback, this);
    registered_scan_sub_ =
        nh_.subscribe(registered_scan_topic_, 1, &UavLocalPlannerNode::RegisteredScanCallback, this);

    safe_waypoint_pub_ = nh_.advertise<geometry_msgs::PointStamped>(safe_waypoint_topic_, 2);
    safe_path_pub_ = nh_.advertise<nav_msgs::Path>(safe_path_topic_, 2);
    free_paths_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(free_paths_topic_, 1);
    local_obstacles_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(local_obstacles_topic_, 1);
    trajectory_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(trajectory_topic_, 1);
    blocked_pub_ = nh_.advertise<std_msgs::Bool>(blocked_topic_, 1, true);
    clearance_pub_ = nh_.advertise<std_msgs::Float32>(clearance_topic_, 1);
    status_pub_ = nh_.advertise<std_msgs::String>(status_topic_, 1, true);

    blocked_pub_.publish(std_msgs::Bool());
    timer_ = nh_.createTimer(ros::Duration(1.0 / planner_rate_), &UavLocalPlannerNode::PlanningTimer, this);

    const double physical_radius = UavLocalPlannerCore::PhysicalRadius(vehicle_length_, vehicle_width_);
    ROS_WARN("uav_local_planner enabled: body=%.3f x %.3f x %.3f m, mass=%.2f kg, physical_radius=%.3f m, "
             "clearance_xy=%.2f m, clearance_z=%.2f m, local_lookahead=%.2f m",
             vehicle_length_, vehicle_width_, vehicle_height_, vehicle_mass_, physical_radius,
             planner_config_.horizontal_clearance, planner_config_.vertical_clearance,
             planner_config_.lookahead_distance);
    if (vehicle_length_assumed_)
    {
      ROS_WARN("uav_local_planner: vehicle_length is assumed equal to the measured 452.98 mm width; update it "
               "before real flight if the front-to-back dimension is larger");
    }
  }

private:
  struct CloudState
  {
    std::vector<Point3> points;
    ros::Time received;
    bool valid = false;
  };

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  tf::TransformListener tf_listener_;
  std::unique_ptr<UavLocalPlannerCore> planner_;
  PlannerConfig planner_config_;
  std::mutex mutex_;

  ros::Subscriber odometry_sub_;
  ros::Subscriber waypoint_sub_;
  ros::Subscriber path_sub_;
  ros::Subscriber collision_cloud_sub_;
  ros::Subscriber registered_scan_sub_;
  ros::Publisher safe_waypoint_pub_;
  ros::Publisher safe_path_pub_;
  ros::Publisher free_paths_pub_;
  ros::Publisher local_obstacles_pub_;
  ros::Publisher trajectory_pub_;
  ros::Publisher blocked_pub_;
  ros::Publisher clearance_pub_;
  ros::Publisher status_pub_;
  ros::Timer timer_;

  std::string planning_frame_;
  std::string odometry_topic_;
  std::string raw_waypoint_topic_;
  std::string reference_path_topic_;
  std::string collision_cloud_topic_;
  std::string registered_scan_topic_;
  std::string safe_waypoint_topic_;
  std::string safe_path_topic_;
  std::string free_paths_topic_;
  std::string local_obstacles_topic_;
  std::string trajectory_topic_;
  std::string blocked_topic_;
  std::string clearance_topic_;
  std::string status_topic_;

  double planner_rate_ = 10.0;
  double obstacle_range_ = 6.0;
  double cloud_voxel_size_ = 0.10;
  double odometry_timeout_ = 0.5;
  double waypoint_timeout_ = 3.0;
  double path_timeout_ = 3.0;
  double scan_timeout_ = 0.5;
  double collision_cloud_timeout_ = 3.0;
  double trajectory_spacing_ = 0.05;
  double vehicle_length_ = 0.45298;
  double vehicle_width_ = 0.45298;
  double vehicle_height_ = 0.17139;
  double vehicle_mass_ = 3.0;
  bool vehicle_length_assumed_ = true;
  bool require_reference_path_ = false;
  bool require_registered_scan_ = true;
  bool require_collision_cloud_ = false;
  bool allow_identity_cloud_frame_ = false;

  Point3 current_position_;
  double current_speed_ = 0.0;
  ros::Time odometry_received_;
  bool have_odometry_ = false;
  Point3 raw_waypoint_;
  ros::Time waypoint_received_;
  bool have_waypoint_ = false;
  std::vector<Point3> reference_path_;
  ros::Time path_received_;
  bool have_path_ = false;
  CloudState collision_cloud_;
  CloudState registered_scan_;
  std::vector<Point3> trajectory_;
  bool control_active_ = false;
  bool blocked_ = false;
  std::string last_status_;

  template <typename T>
  void Param(const std::string& name, T& value)
  {
    private_nh_.param(name, value, value);
  }

  void LoadParameters()
  {
    planning_frame_ = "map";
    odometry_topic_ = "/mavros/local_position/odom";
    raw_waypoint_topic_ = "/way_point";
    reference_path_topic_ = "/sensor_coverage_planner/exploration_path";
    collision_cloud_topic_ = "/sensor_coverage_planner/collision_cloud";
    registered_scan_topic_ = "/registered_scan";
    safe_waypoint_topic_ = "/tare_uav/safe_waypoint";
    safe_path_topic_ = "/path";
    free_paths_topic_ = "/free_paths";
    local_obstacles_topic_ = "/added_obstacles";
    trajectory_topic_ = "/trajectory";
    blocked_topic_ = "/tare_uav/local_planner/blocked";
    clearance_topic_ = "/tare_uav/local_planner/minimum_clearance";
    status_topic_ = "/tare_uav/local_planner/status";

    Param("planning_frame", planning_frame_);
    Param("odometry_topic", odometry_topic_);
    Param("raw_waypoint_topic", raw_waypoint_topic_);
    Param("reference_path_topic", reference_path_topic_);
    Param("collision_cloud_topic", collision_cloud_topic_);
    Param("registered_scan_topic", registered_scan_topic_);
    Param("safe_waypoint_topic", safe_waypoint_topic_);
    Param("safe_path_topic", safe_path_topic_);
    Param("free_paths_topic", free_paths_topic_);
    Param("local_obstacles_topic", local_obstacles_topic_);
    Param("trajectory_topic", trajectory_topic_);
    Param("blocked_topic", blocked_topic_);
    Param("clearance_topic", clearance_topic_);
    Param("status_topic", status_topic_);
    Param("planner_rate", planner_rate_);
    Param("obstacle_range", obstacle_range_);
    Param("cloud_voxel_size", cloud_voxel_size_);
    Param("odometry_timeout", odometry_timeout_);
    Param("waypoint_timeout", waypoint_timeout_);
    Param("path_timeout", path_timeout_);
    Param("scan_timeout", scan_timeout_);
    Param("collision_cloud_timeout", collision_cloud_timeout_);
    Param("trajectory_spacing", trajectory_spacing_);
    Param("vehicle_length", vehicle_length_);
    Param("vehicle_width", vehicle_width_);
    Param("vehicle_height", vehicle_height_);
    Param("vehicle_mass", vehicle_mass_);
    Param("vehicle_length_assumed", vehicle_length_assumed_);
    Param("require_reference_path", require_reference_path_);
    Param("require_registered_scan", require_registered_scan_);
    Param("require_collision_cloud", require_collision_cloud_);
    Param("allow_identity_cloud_frame", allow_identity_cloud_frame_);

    double candidate_angle_step_degrees = 15.0;
    Param("lookahead_distance", planner_config_.lookahead_distance);
    Param("horizontal_clearance", planner_config_.horizontal_clearance);
    Param("vertical_clearance", planner_config_.vertical_clearance);
    Param("max_deceleration", planner_config_.max_deceleration);
    Param("reaction_time", planner_config_.reaction_time);
    Param("self_filter_radius", planner_config_.self_filter_radius);
    Param("escape_improvement", planner_config_.escape_improvement);
    Param("candidate_angle_step_degrees", candidate_angle_step_degrees);
    Param("candidate_angle_count", planner_config_.candidate_angle_count);
    Param("collision_point_threshold", planner_config_.collision_point_threshold);
    Param("path_weight", planner_config_.path_weight);
    Param("goal_weight", planner_config_.goal_weight);
    Param("heading_weight", planner_config_.heading_weight);
    Param("clearance_weight", planner_config_.clearance_weight);
    Param("switch_weight", planner_config_.switch_weight);
    planner_config_.candidate_angle_step = candidate_angle_step_degrees * 3.14159265358979323846 / 180.0;
  }

  void ValidateParameters() const
  {
    const double physical_radius = UavLocalPlannerCore::PhysicalRadius(vehicle_length_, vehicle_width_);
    if (planner_rate_ <= 0.0 || planner_config_.lookahead_distance <= 0.0 || obstacle_range_ <= 0.0 ||
        cloud_voxel_size_ <= 0.0)
    {
      throw std::runtime_error("uav_local_planner requires positive rate, lookahead, obstacle range, and voxel size");
    }
    if (vehicle_length_ <= 0.0 || vehicle_width_ <= 0.0 || vehicle_height_ <= 0.0 || vehicle_mass_ <= 0.0)
    {
      throw std::runtime_error("uav_local_planner vehicle dimensions and mass must be positive");
    }
    if (planner_config_.horizontal_clearance <= physical_radius)
    {
      throw std::runtime_error("uav_local_planner horizontal_clearance must exceed the physical half diagonal");
    }
    if (planner_config_.vertical_clearance <= vehicle_height_ * 0.5)
    {
      throw std::runtime_error("uav_local_planner vertical_clearance must exceed half the vehicle height");
    }
    if (planner_config_.candidate_angle_count < 0 || planner_config_.collision_point_threshold < 1)
    {
      throw std::runtime_error("uav_local_planner candidate count and collision threshold are invalid");
    }
  }

  void OdometryCallback(const nav_msgs::Odometry::ConstPtr& message)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    current_position_ = Point3{ message->pose.pose.position.x, message->pose.pose.position.y,
                               message->pose.pose.position.z };
    current_speed_ = std::hypot(message->twist.twist.linear.x, message->twist.twist.linear.y);
    odometry_received_ = ros::Time::now();
    have_odometry_ = true;
    if (trajectory_.empty() || UavLocalPlannerCore::DistanceXY(trajectory_.back(), current_position_) >= trajectory_spacing_)
    {
      trajectory_.push_back(current_position_);
      if (trajectory_.size() > 20000)
      {
        trajectory_.erase(trajectory_.begin(), trajectory_.begin() + 1000);
      }
    }
  }

  void WaypointCallback(const geometry_msgs::PointStamped::ConstPtr& message)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    raw_waypoint_ = Point3{ message->point.x, message->point.y, message->point.z };
    waypoint_received_ = ros::Time::now();
    have_waypoint_ = true;
  }

  void PathCallback(const nav_msgs::Path::ConstPtr& message)
  {
    std::vector<Point3> path;
    path.reserve(message->poses.size());
    for (const geometry_msgs::PoseStamped& pose : message->poses)
    {
      path.push_back(Point3{ pose.pose.position.x, pose.pose.position.y, pose.pose.position.z });
    }
    std::lock_guard<std::mutex> lock(mutex_);
    reference_path_ = std::move(path);
    path_received_ = ros::Time::now();
    have_path_ = !reference_path_.empty();
  }

  void CollisionCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& message)
  {
    UpdateCloud(message, collision_cloud_, "collision cloud");
  }

  void RegisteredScanCallback(const sensor_msgs::PointCloud2::ConstPtr& message)
  {
    UpdateCloud(message, registered_scan_, "registered scan");
  }

  void UpdateCloud(const sensor_msgs::PointCloud2::ConstPtr& message, CloudState& destination,
                   const std::string& source_name)
  {
    sensor_msgs::PointCloud2 transformed;
    if (message->header.frame_id.empty() || message->header.frame_id == planning_frame_)
    {
      transformed = *message;
      transformed.header.frame_id = planning_frame_;
    }
    else
    {
      try
      {
        if (!pcl_ros::transformPointCloud(planning_frame_, *message, transformed, tf_listener_))
        {
          if (!allow_identity_cloud_frame_)
          {
            ROS_ERROR_THROTTLE(2.0, "uav_local_planner cannot transform %s from %s to %s", source_name.c_str(),
                               message->header.frame_id.c_str(), planning_frame_.c_str());
            return;
          }
          transformed = *message;
          transformed.header.frame_id = planning_frame_;
          ROS_WARN_THROTTLE(5.0, "uav_local_planner is assuming identity transform for %s (%s -> %s)",
                            source_name.c_str(), message->header.frame_id.c_str(), planning_frame_.c_str());
        }
      }
      catch (const tf::TransformException& exception)
      {
        ROS_ERROR_THROTTLE(2.0, "uav_local_planner transform failed for %s: %s", source_name.c_str(),
                           exception.what());
        return;
      }
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(transformed, *cloud);
    pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
    voxel_filter.setInputCloud(cloud);
    voxel_filter.setLeafSize(cloud_voxel_size_, cloud_voxel_size_, cloud_voxel_size_);
    pcl::PointCloud<pcl::PointXYZ> filtered;
    voxel_filter.filter(filtered);

    std::vector<Point3> points;
    points.reserve(filtered.size());
    for (const pcl::PointXYZ& point : filtered)
    {
      if (std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z))
      {
        points.push_back(Point3{ point.x, point.y, point.z });
      }
    }

    std::lock_guard<std::mutex> lock(mutex_);
    destination.points = std::move(points);
    destination.received = ros::Time::now();
    destination.valid = true;
  }

  bool IsFresh(bool available, const ros::Time& received, double timeout, const ros::Time& now) const
  {
    return available && !received.isZero() && (now - received).toSec() <= timeout;
  }

  void PlanningTimer(const ros::TimerEvent&)
  {
    Point3 current;
    Point3 raw_target;
    double speed = 0.0;
    std::vector<Point3> path;
    std::vector<Point3> trajectory;
    CloudState collision;
    CloudState scan;
    ros::Time odometry_received;
    ros::Time waypoint_received;
    ros::Time path_received;
    bool have_odometry = false;
    bool have_waypoint = false;
    bool have_path = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      current = current_position_;
      raw_target = raw_waypoint_;
      speed = current_speed_;
      path = reference_path_;
      trajectory = trajectory_;
      collision = collision_cloud_;
      scan = registered_scan_;
      odometry_received = odometry_received_;
      waypoint_received = waypoint_received_;
      path_received = path_received_;
      have_odometry = have_odometry_;
      have_waypoint = have_waypoint_;
      have_path = have_path_;
    }

    const ros::Time now = ros::Time::now();
    if (!IsFresh(have_odometry, odometry_received, odometry_timeout_, now))
    {
      HandleUnavailable("waiting for fresh odometry", current, raw_target, have_waypoint);
      return;
    }
    if (!IsFresh(have_waypoint, waypoint_received, waypoint_timeout_, now))
    {
      HandleUnavailable("waiting for fresh TARE waypoint", current, raw_target, false);
      return;
    }

    const bool path_fresh = IsFresh(have_path, path_received, path_timeout_, now);
    if (require_reference_path_ && !path_fresh)
    {
      HandleUnavailable("reference path stale or empty", current, raw_target, true);
      return;
    }
    if (!path_fresh)
    {
      path.clear();
    }

    const bool scan_fresh = IsFresh(scan.valid, scan.received, scan_timeout_, now);
    const bool collision_fresh = IsFresh(collision.valid, collision.received, collision_cloud_timeout_, now);
    if (require_registered_scan_ && !scan_fresh)
    {
      HandleUnavailable("registered scan stale", current, raw_target, true);
      return;
    }
    if (require_collision_cloud_ && !collision_fresh)
    {
      HandleUnavailable("TARE collision cloud stale", current, raw_target, true);
      return;
    }

    std::vector<Point3> obstacles;
    if (scan_fresh)
    {
      AppendLocalObstacles(scan.points, current, obstacles);
    }
    if (collision_fresh)
    {
      AppendLocalObstacles(collision.points, current, obstacles);
    }

    const PlanResult result = planner_->Plan(current, raw_target, path, obstacles, speed);
    PublishVisualization(now, current, result, obstacles, trajectory);
    if (result.safe)
    {
      PublishSafeWaypoint(now, result.target);
      control_active_ = true;
      SetBlocked(false);
      std::ostringstream status;
      status << "tracking local path; clearance=" << result.minimum_clearance << " m";
      PublishStatus(status.str(), false);
    }
    else
    {
      PublishHold(now, current, raw_target.z);
      SetBlocked(true);
      PublishStatus("all local candidate paths blocked; holding for TARE replan", true);
    }
  }

  void AppendLocalObstacles(const std::vector<Point3>& source, const Point3& current,
                            std::vector<Point3>& destination) const
  {
    const double range_sq = obstacle_range_ * obstacle_range_;
    for (const Point3& point : source)
    {
      const double dx = point.x - current.x;
      const double dy = point.y - current.y;
      if (dx * dx + dy * dy <= range_sq)
      {
        destination.push_back(point);
      }
    }
  }

  void HandleUnavailable(const std::string& reason, const Point3& current, const Point3& raw_target,
                         bool have_target)
  {
    if (control_active_)
    {
      PublishHold(ros::Time::now(), current, have_target ? raw_target.z : current.z);
      SetBlocked(true);
    }
    PublishStatus(reason, control_active_);
  }

  void PublishSafeWaypoint(const ros::Time& stamp, const Point3& target)
  {
    geometry_msgs::PointStamped message;
    message.header.stamp = stamp;
    message.header.frame_id = planning_frame_;
    message.point.x = target.x;
    message.point.y = target.y;
    message.point.z = target.z;
    safe_waypoint_pub_.publish(message);
  }

  void PublishHold(const ros::Time& stamp, const Point3& current, double target_z)
  {
    Point3 hold = current;
    hold.z = target_z;
    PublishSafeWaypoint(stamp, hold);
  }

  void SetBlocked(bool blocked)
  {
    if (blocked_ == blocked && !last_status_.empty())
    {
      return;
    }
    blocked_ = blocked;
    std_msgs::Bool message;
    message.data = blocked;
    blocked_pub_.publish(message);
  }

  void PublishStatus(const std::string& status, bool warning)
  {
    if (status != last_status_)
    {
      std_msgs::String message;
      message.data = status;
      status_pub_.publish(message);
      last_status_ = status;
    }
    if (warning)
    {
      ROS_WARN_THROTTLE(2.0, "uav_local_planner: %s", status.c_str());
    }
    else
    {
      ROS_INFO_THROTTLE(5.0, "uav_local_planner: %s", status.c_str());
    }
  }

  void PublishVisualization(const ros::Time& stamp, const Point3& current, const PlanResult& result,
                            const std::vector<Point3>& obstacles, const std::vector<Point3>& trajectory)
  {
    nav_msgs::Path safe_path;
    safe_path.header.stamp = stamp;
    safe_path.header.frame_id = planning_frame_;
    geometry_msgs::PoseStamped start_pose;
    start_pose.header = safe_path.header;
    start_pose.pose.position.x = current.x;
    start_pose.pose.position.y = current.y;
    start_pose.pose.position.z = current.z;
    start_pose.pose.orientation.w = 1.0;
    safe_path.poses.push_back(start_pose);
    geometry_msgs::PoseStamped target_pose = start_pose;
    target_pose.pose.position.x = result.target.x;
    target_pose.pose.position.y = result.target.y;
    target_pose.pose.position.z = result.target.z;
    safe_path.poses.push_back(target_pose);
    safe_path_pub_.publish(safe_path);

    pcl::PointCloud<pcl::PointXYZI> candidates;
    for (int index = 0; index < static_cast<int>(result.candidates.size()); ++index)
    {
      const Candidate& candidate = result.candidates[index];
      for (int sample = 0; sample <= 15; ++sample)
      {
        const double ratio = static_cast<double>(sample) / 15.0;
        pcl::PointXYZI point;
        point.x = current.x + ratio * (candidate.target.x - current.x);
        point.y = current.y + ratio * (candidate.target.y - current.y);
        point.z = current.z + ratio * (candidate.target.z - current.z);
        point.intensity = candidate.safe ? 1.0f + static_cast<float>(index % 5) : 0.0f;
        candidates.push_back(point);
      }
    }
    PublishCloud(stamp, candidates, free_paths_pub_);

    pcl::PointCloud<pcl::PointXYZI> obstacle_cloud;
    obstacle_cloud.reserve(obstacles.size());
    for (const Point3& obstacle : obstacles)
    {
      pcl::PointXYZI point;
      point.x = obstacle.x;
      point.y = obstacle.y;
      point.z = obstacle.z;
      point.intensity = 1.0f;
      obstacle_cloud.push_back(point);
    }
    PublishCloud(stamp, obstacle_cloud, local_obstacles_pub_);

    pcl::PointCloud<pcl::PointXYZI> trajectory_cloud;
    trajectory_cloud.reserve(trajectory.size());
    for (int index = 0; index < static_cast<int>(trajectory.size()); ++index)
    {
      pcl::PointXYZI point;
      point.x = trajectory[index].x;
      point.y = trajectory[index].y;
      point.z = trajectory[index].z;
      point.intensity = static_cast<float>(index % 100) / 100.0f;
      trajectory_cloud.push_back(point);
    }
    PublishCloud(stamp, trajectory_cloud, trajectory_pub_);

    std_msgs::Float32 clearance;
    clearance.data = std::isfinite(result.minimum_clearance) ? static_cast<float>(result.minimum_clearance) : -1.0f;
    clearance_pub_.publish(clearance);
  }

  void PublishCloud(const ros::Time& stamp, const pcl::PointCloud<pcl::PointXYZI>& cloud, ros::Publisher& publisher)
  {
    sensor_msgs::PointCloud2 message;
    pcl::toROSMsg(cloud, message);
    message.header.stamp = stamp;
    message.header.frame_id = planning_frame_;
    publisher.publish(message);
  }
};
}  // namespace uav_local_planner_ns

int main(int argc, char** argv)
{
  ros::init(argc, argv, "uav_local_planner");
  try
  {
    uav_local_planner_ns::UavLocalPlannerNode node;
    ros::spin();
  }
  catch (const std::exception& exception)
  {
    ROS_FATAL("uav_local_planner startup failed: %s", exception.what());
    return 1;
  }
  return 0;
}
