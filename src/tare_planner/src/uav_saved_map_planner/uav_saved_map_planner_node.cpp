#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <queue>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/point_tests.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>

namespace
{
struct OpenCell
{
  int index;
  double score;
  bool operator<(const OpenCell& other) const { return score > other.score; }
};

double Distance2D(const geometry_msgs::Point& a, const geometry_msgs::Point& b)
{
  return std::hypot(a.x - b.x, a.y - b.y);
}
}  // namespace

class UavSavedMapPlanner
{
public:
  UavSavedMapPlanner()
    : nh_()
    , pnh_("~")
  {
    pnh_.param<std::string>("map_file", map_file_, "");
    pnh_.param<std::string>("frame_id", frame_id_, "map");
    pnh_.param("fixed_flight_height", fixed_height_, 1.5);
    pnh_.param("fixed_flight_height_relative_to_start", relative_height_, false);
    pnh_.param("resolution", resolution_, 0.20);
    pnh_.param("horizontal_clearance", horizontal_clearance_, 0.75);
    pnh_.param("vertical_clearance", vertical_clearance_, 0.35);
    pnh_.param("vehicle_height", vehicle_height_, 0.17139);
    pnh_.param("map_padding", map_padding_, 1.0);
    pnh_.param("goal_snap_radius", goal_snap_radius_, 1.0);
    pnh_.param("path_spacing", path_spacing_, 0.30);
    pnh_.param("max_grid_cells", max_grid_cells_, 4000000);
    pnh_.param("map_offset_x", map_offset_x_, 0.0);
    pnh_.param("map_offset_y", map_offset_y_, 0.0);
    pnh_.param("map_offset_z", map_offset_z_, 0.0);
    pnh_.param("map_offset_yaw", map_offset_yaw_, 0.0);
    pnh_.param("use_explicit_start", use_explicit_start_, false);

    std::string odometry_topic;
    std::string goal_topic;
    std::string start_topic;
    std::string path_topic;
    std::string waypoint_topic;
    std::string map_topic;
    std::string grid_topic;
    std::string status_topic;
    pnh_.param<std::string>("odometry_topic", odometry_topic, "/state_estimation");
    pnh_.param<std::string>("goal_topic", goal_topic, "/tare_uav/source/saved_map/goal");
    pnh_.param<std::string>("start_topic", start_topic, "/tare_uav/saved_map/start");
    pnh_.param<std::string>("path_topic", path_topic, "/tare_uav/source/saved_map/path");
    pnh_.param<std::string>("waypoint_topic", waypoint_topic, "/tare_uav/source/saved_map/waypoint");
    pnh_.param<std::string>("map_topic", map_topic, "/tare_uav/saved_map/cloud");
    pnh_.param<std::string>("grid_topic", grid_topic, "/tare_uav/saved_map/occupancy");
    pnh_.param<std::string>("status_topic", status_topic, "/tare_uav/saved_map/status");

    path_pub_ = nh_.advertise<nav_msgs::Path>(path_topic, 1, true);
    waypoint_pub_ = nh_.advertise<geometry_msgs::PointStamped>(waypoint_topic, 2, true);
    map_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(map_topic, 1, true);
    grid_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>(grid_topic, 1, true);
    status_pub_ = nh_.advertise<std_msgs::String>(status_topic, 1, true);
    odometry_sub_ = nh_.subscribe(odometry_topic, 10, &UavSavedMapPlanner::OdometryCallback, this);
    goal_sub_ = nh_.subscribe(goal_topic, 1, &UavSavedMapPlanner::GoalCallback, this);
    start_sub_ = nh_.subscribe(start_topic, 1, &UavSavedMapPlanner::StartCallback, this);
    republish_timer_ = nh_.createTimer(ros::Duration(0.2), &UavSavedMapPlanner::RepublishCallback, this);

    if (resolution_ <= 0.0 || horizontal_clearance_ <= 0.0 || vertical_clearance_ <= 0.0)
    {
      throw std::runtime_error("saved-map planner dimensions must be positive");
    }
    LoadMap();
  }

private:
  void PublishStatus(const std::string& text, bool error = false)
  {
    std_msgs::String message;
    message.data = text;
    status_pub_.publish(message);
    if (error)
    {
      ROS_ERROR_STREAM("uav_saved_map_planner: " << text);
    }
    else
    {
      ROS_INFO_STREAM("uav_saved_map_planner: " << text);
    }
  }

  bool LoadMap()
  {
    if (map_file_.empty())
    {
      PublishStatus("map_file is empty; saved-map planning is unavailable", true);
      return false;
    }
    pcl::PointCloud<pcl::PointXYZ> raw;
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(map_file_, raw) != 0 || raw.empty())
    {
      PublishStatus("failed to load PCD map: " + map_file_, true);
      return false;
    }

    const double cosine = std::cos(map_offset_yaw_);
    const double sine = std::sin(map_offset_yaw_);
    map_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>());
    map_cloud_->reserve(raw.size());
    for (const auto& point : raw.points)
    {
      if (!pcl::isFinite(point))
      {
        continue;
      }
      pcl::PointXYZ transformed;
      transformed.x = map_offset_x_ + cosine * point.x - sine * point.y;
      transformed.y = map_offset_y_ + sine * point.x + cosine * point.y;
      transformed.z = map_offset_z_ + point.z;
      map_cloud_->push_back(transformed);
    }
    map_cloud_->width = map_cloud_->size();
    map_cloud_->height = 1;
    map_cloud_->is_dense = false;

    sensor_msgs::PointCloud2 message;
    pcl::toROSMsg(*map_cloud_, message);
    message.header.frame_id = frame_id_;
    message.header.stamp = ros::Time::now();
    map_pub_.publish(message);
    PublishStatus("loaded PCD map with " + std::to_string(map_cloud_->size()) + " points");
    if (!relative_height_)
    {
      return BuildGrid();
    }
    return true;
  }

  double FlightHeight() const
  {
    return (relative_height_ ? mission_start_z_ : 0.0) + fixed_height_;
  }

  void OdometryCallback(const nav_msgs::Odometry::ConstPtr& message)
  {
    current_position_ = message->pose.pose.position;
    current_position_set_ = true;
    if (!mission_start_z_set_)
    {
      mission_start_z_ = current_position_.z;
      mission_start_z_set_ = true;
      if (relative_height_)
      {
        BuildGrid();
      }
    }
  }

  void StartCallback(const geometry_msgs::PoseStamped::ConstPtr& message)
  {
    if (!message->header.frame_id.empty() && message->header.frame_id != frame_id_)
    {
      PublishStatus("explicit start frame does not match " + frame_id_, true);
      return;
    }
    explicit_start_ = message->pose.position;
    explicit_start_set_ = true;
    PublishStatus("explicit saved-map planning start accepted");
  }

  bool BuildGrid()
  {
    if (!map_cloud_ || map_cloud_->empty() || (relative_height_ && !mission_start_z_set_))
    {
      return false;
    }
    double min_x = std::numeric_limits<double>::max();
    double min_y = std::numeric_limits<double>::max();
    double max_x = -std::numeric_limits<double>::max();
    double max_y = -std::numeric_limits<double>::max();
    for (const auto& point : map_cloud_->points)
    {
      min_x = std::min(min_x, static_cast<double>(point.x));
      min_y = std::min(min_y, static_cast<double>(point.y));
      max_x = std::max(max_x, static_cast<double>(point.x));
      max_y = std::max(max_y, static_cast<double>(point.y));
    }
    origin_x_ = min_x - map_padding_;
    origin_y_ = min_y - map_padding_;
    width_ = static_cast<int>(std::ceil((max_x - min_x + 2.0 * map_padding_) / resolution_)) + 1;
    height_ = static_cast<int>(std::ceil((max_y - min_y + 2.0 * map_padding_) / resolution_)) + 1;
    const long long cell_count = static_cast<long long>(width_) * height_;
    if (width_ < 3 || height_ < 3 || cell_count > max_grid_cells_)
    {
      PublishStatus("PCD grid is invalid or exceeds max_grid_cells (" +
                        std::to_string(cell_count) + ")",
                    true);
      return false;
    }

    std::vector<uint8_t> raw_occupied(cell_count, 0);
    const double z_half_band = vertical_clearance_ + vehicle_height_ * 0.5;
    const double flight_z = FlightHeight();
    for (const auto& point : map_cloud_->points)
    {
      if (std::abs(point.z - flight_z) > z_half_band)
      {
        continue;
      }
      int x;
      int y;
      if (WorldToCell(point.x, point.y, x, y))
      {
        raw_occupied[Index(x, y)] = 1;
      }
    }

    occupied_.assign(cell_count, 0);
    const int inflation = static_cast<int>(std::ceil(horizontal_clearance_ / resolution_));
    for (int y = 0; y < height_; ++y)
    {
      for (int x = 0; x < width_; ++x)
      {
        if (!raw_occupied[Index(x, y)])
        {
          continue;
        }
        for (int dy = -inflation; dy <= inflation; ++dy)
        {
          for (int dx = -inflation; dx <= inflation; ++dx)
          {
            if (dx * dx + dy * dy > inflation * inflation)
            {
              continue;
            }
            const int nx = x + dx;
            const int ny = y + dy;
            if (InBounds(nx, ny))
            {
              occupied_[Index(nx, ny)] = 1;
            }
          }
        }
      }
    }
    for (int x = 0; x < width_; ++x)
    {
      occupied_[Index(x, 0)] = 1;
      occupied_[Index(x, height_ - 1)] = 1;
    }
    for (int y = 0; y < height_; ++y)
    {
      occupied_[Index(0, y)] = 1;
      occupied_[Index(width_ - 1, y)] = 1;
    }
    grid_ready_ = true;
    PublishGrid();
    PublishStatus("built A* grid at z=" + std::to_string(flight_z) + " m, " +
                  std::to_string(width_) + "x" + std::to_string(height_));
    return true;
  }

  void PublishGrid()
  {
    nav_msgs::OccupancyGrid message;
    message.header.frame_id = frame_id_;
    message.header.stamp = ros::Time::now();
    message.info.resolution = resolution_;
    message.info.width = width_;
    message.info.height = height_;
    message.info.origin.position.x = origin_x_;
    message.info.origin.position.y = origin_y_;
    message.info.origin.orientation.w = 1.0;
    message.data.resize(occupied_.size());
    for (std::size_t index = 0; index < occupied_.size(); ++index)
    {
      message.data[index] = occupied_[index] ? 100 : 0;
    }
    grid_pub_.publish(message);
  }

  int Index(int x, int y) const { return y * width_ + x; }
  bool InBounds(int x, int y) const { return x >= 0 && y >= 0 && x < width_ && y < height_; }

  bool WorldToCell(double world_x, double world_y, int& cell_x, int& cell_y) const
  {
    cell_x = static_cast<int>(std::floor((world_x - origin_x_) / resolution_));
    cell_y = static_cast<int>(std::floor((world_y - origin_y_) / resolution_));
    return InBounds(cell_x, cell_y);
  }

  geometry_msgs::Point CellToWorld(int index) const
  {
    geometry_msgs::Point point;
    point.x = origin_x_ + (index % width_ + 0.5) * resolution_;
    point.y = origin_y_ + (index / width_ + 0.5) * resolution_;
    point.z = FlightHeight();
    return point;
  }

  bool SnapToFree(int& x, int& y) const
  {
    if (InBounds(x, y) && !occupied_[Index(x, y)])
    {
      return true;
    }
    const int radius = static_cast<int>(std::ceil(goal_snap_radius_ / resolution_));
    const int original_x = x;
    const int original_y = y;
    double best_distance = std::numeric_limits<double>::max();
    int best_x = -1;
    int best_y = -1;
    for (int dy = -radius; dy <= radius; ++dy)
    {
      for (int dx = -radius; dx <= radius; ++dx)
      {
        const int nx = original_x + dx;
        const int ny = original_y + dy;
        const double distance = std::hypot(dx, dy);
        if (distance <= radius && InBounds(nx, ny) && !occupied_[Index(nx, ny)] &&
            distance < best_distance)
        {
          best_distance = distance;
          best_x = nx;
          best_y = ny;
        }
      }
    }
    if (best_x < 0)
    {
      return false;
    }
    x = best_x;
    y = best_y;
    return true;
  }

  bool LineIsFree(int from, int to) const
  {
    int x0 = from % width_;
    int y0 = from / width_;
    const int x1 = to % width_;
    const int y1 = to / width_;
    const int dx = std::abs(x1 - x0);
    const int sx = x0 < x1 ? 1 : -1;
    const int dy = -std::abs(y1 - y0);
    const int sy = y0 < y1 ? 1 : -1;
    int error = dx + dy;
    while (true)
    {
      if (!InBounds(x0, y0) || occupied_[Index(x0, y0)])
      {
        return false;
      }
      if (x0 == x1 && y0 == y1)
      {
        return true;
      }
      const int twice_error = 2 * error;
      if (twice_error >= dy)
      {
        error += dy;
        x0 += sx;
      }
      if (twice_error <= dx)
      {
        error += dx;
        y0 += sy;
      }
    }
  }

  bool RunAStar(const geometry_msgs::Point& requested_start, const geometry_msgs::Point& requested_goal,
                std::vector<int>& result, std::string& failure)
  {
    int start_x;
    int start_y;
    int goal_x;
    int goal_y;
    if (!WorldToCell(requested_start.x, requested_start.y, start_x, start_y) ||
        !WorldToCell(requested_goal.x, requested_goal.y, goal_x, goal_y))
    {
      failure = "start or goal lies outside the PCD map bounds";
      return false;
    }
    if (!SnapToFree(start_x, start_y) || !SnapToFree(goal_x, goal_y))
    {
      failure = "no collision-free start/goal cell exists within goal_snap_radius";
      return false;
    }
    const int start = Index(start_x, start_y);
    const int goal = Index(goal_x, goal_y);
    const int cell_count = width_ * height_;
    std::vector<double> cost(cell_count, std::numeric_limits<double>::max());
    std::vector<int> parent(cell_count, -1);
    std::vector<uint8_t> closed(cell_count, 0);
    std::priority_queue<OpenCell> open;
    cost[start] = 0.0;
    open.push({ start, 0.0 });
    const int neighbor_x[8] = { -1, 0, 1, -1, 1, -1, 0, 1 };
    const int neighbor_y[8] = { -1, -1, -1, 0, 0, 1, 1, 1 };
    while (!open.empty())
    {
      const int current = open.top().index;
      open.pop();
      if (closed[current])
      {
        continue;
      }
      closed[current] = 1;
      if (current == goal)
      {
        break;
      }
      const int x = current % width_;
      const int y = current / width_;
      for (int neighbor = 0; neighbor < 8; ++neighbor)
      {
        const int nx = x + neighbor_x[neighbor];
        const int ny = y + neighbor_y[neighbor];
        if (!InBounds(nx, ny) || occupied_[Index(nx, ny)])
        {
          continue;
        }
        if (neighbor_x[neighbor] != 0 && neighbor_y[neighbor] != 0 &&
            (occupied_[Index(x + neighbor_x[neighbor], y)] ||
             occupied_[Index(x, y + neighbor_y[neighbor])]))
        {
          continue;
        }
        const int next = Index(nx, ny);
        const double step = neighbor_x[neighbor] == 0 || neighbor_y[neighbor] == 0 ? 1.0 : std::sqrt(2.0);
        const double next_cost = cost[current] + step;
        if (next_cost >= cost[next])
        {
          continue;
        }
        cost[next] = next_cost;
        parent[next] = current;
        const double heuristic = std::hypot(goal_x - nx, goal_y - ny);
        open.push({ next, next_cost + heuristic });
      }
    }
    if (parent[goal] < 0 && start != goal)
    {
      failure = "A* found no path through the inflated PCD obstacle layer";
      return false;
    }
    result.clear();
    for (int current = goal; current >= 0; current = parent[current])
    {
      result.push_back(current);
      if (current == start)
      {
        break;
      }
    }
    std::reverse(result.begin(), result.end());
    return !result.empty() && result.front() == start;
  }

  std::vector<int> Simplify(const std::vector<int>& cells) const
  {
    if (cells.size() <= 2)
    {
      return cells;
    }
    std::vector<int> simplified;
    simplified.push_back(cells.front());
    std::size_t anchor = 0;
    while (anchor + 1 < cells.size())
    {
      std::size_t next = cells.size() - 1;
      while (next > anchor + 1 && !LineIsFree(cells[anchor], cells[next]))
      {
        --next;
      }
      simplified.push_back(cells[next]);
      anchor = next;
    }
    return simplified;
  }

  nav_msgs::Path BuildPathMessage(const std::vector<int>& cells, const geometry_msgs::Point& requested_start,
                                  const geometry_msgs::Point& requested_goal) const
  {
    std::vector<geometry_msgs::Point> control_points;
    control_points.push_back(requested_start);
    for (std::size_t index = 1; index + 1 < cells.size(); ++index)
    {
      control_points.push_back(CellToWorld(cells[index]));
    }
    control_points.push_back(requested_goal);
    for (auto& point : control_points)
    {
      point.z = FlightHeight();
    }

    std::vector<geometry_msgs::Point> samples;
    samples.push_back(control_points.front());
    for (std::size_t index = 1; index < control_points.size(); ++index)
    {
      const auto& from = control_points[index - 1];
      const auto& to = control_points[index];
      const double length = Distance2D(from, to);
      const int count = std::max(1, static_cast<int>(std::ceil(length / path_spacing_)));
      for (int sample = 1; sample <= count; ++sample)
      {
        const double ratio = static_cast<double>(sample) / count;
        geometry_msgs::Point point;
        point.x = from.x + ratio * (to.x - from.x);
        point.y = from.y + ratio * (to.y - from.y);
        point.z = FlightHeight();
        samples.push_back(point);
      }
    }

    nav_msgs::Path path;
    path.header.frame_id = frame_id_;
    path.header.stamp = ros::Time::now();
    for (std::size_t index = 0; index < samples.size(); ++index)
    {
      geometry_msgs::PoseStamped pose;
      pose.header = path.header;
      pose.pose.position = samples[index];
      double yaw = 0.0;
      if (index + 1 < samples.size())
      {
        yaw = std::atan2(samples[index + 1].y - samples[index].y,
                         samples[index + 1].x - samples[index].x);
      }
      else if (index > 0)
      {
        yaw = std::atan2(samples[index].y - samples[index - 1].y,
                         samples[index].x - samples[index - 1].x);
      }
      pose.pose.orientation.z = std::sin(yaw * 0.5);
      pose.pose.orientation.w = std::cos(yaw * 0.5);
      path.poses.push_back(pose);
    }
    return path;
  }

  void GoalCallback(const geometry_msgs::PoseStamped::ConstPtr& message)
  {
    if (!message->header.frame_id.empty() && message->header.frame_id != frame_id_)
    {
      PublishStatus("goal frame does not match " + frame_id_, true);
      return;
    }
    if (!grid_ready_ && !BuildGrid())
    {
      PublishStatus("occupancy grid is not ready", true);
      return;
    }
    geometry_msgs::Point start;
    if (use_explicit_start_)
    {
      if (!explicit_start_set_)
      {
        PublishStatus("use_explicit_start is true but no start pose was published", true);
        return;
      }
      start = explicit_start_;
    }
    else
    {
      if (!current_position_set_)
      {
        PublishStatus("odometry is unavailable", true);
        return;
      }
      start = current_position_;
    }
    geometry_msgs::Point goal = message->pose.position;
    start.z = FlightHeight();
    goal.z = FlightHeight();
    std::vector<int> cells;
    std::string failure;
    if (!RunAStar(start, goal, cells, failure))
    {
      PublishStatus(failure, true);
      return;
    }
    const std::vector<int> simplified = Simplify(cells);
    last_path_ = BuildPathMessage(simplified, start, goal);
    last_waypoint_.header = last_path_.header;
    last_waypoint_.point = goal;
    path_pub_.publish(last_path_);
    waypoint_pub_.publish(last_waypoint_);
    path_ready_ = true;
    PublishStatus("A* path ready: " + std::to_string(last_path_.poses.size()) +
                  " poses at z=" + std::to_string(FlightHeight()));
  }

  void RepublishCallback(const ros::TimerEvent&)
  {
    if (!path_ready_)
    {
      return;
    }
    last_path_.header.stamp = ros::Time::now();
    for (auto& pose : last_path_.poses)
    {
      pose.header = last_path_.header;
    }
    last_waypoint_.header = last_path_.header;
    path_pub_.publish(last_path_);
    waypoint_pub_.publish(last_waypoint_);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber odometry_sub_;
  ros::Subscriber goal_sub_;
  ros::Subscriber start_sub_;
  ros::Publisher path_pub_;
  ros::Publisher waypoint_pub_;
  ros::Publisher map_pub_;
  ros::Publisher grid_pub_;
  ros::Publisher status_pub_;
  ros::Timer republish_timer_;

  std::string map_file_;
  std::string frame_id_;
  double fixed_height_;
  bool relative_height_;
  double resolution_;
  double horizontal_clearance_;
  double vertical_clearance_;
  double vehicle_height_;
  double map_padding_;
  double goal_snap_radius_;
  double path_spacing_;
  int max_grid_cells_;
  double map_offset_x_;
  double map_offset_y_;
  double map_offset_z_;
  double map_offset_yaw_;
  bool use_explicit_start_;

  pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud_;
  std::vector<uint8_t> occupied_;
  double origin_x_{ 0.0 };
  double origin_y_{ 0.0 };
  int width_{ 0 };
  int height_{ 0 };
  bool grid_ready_{ false };
  geometry_msgs::Point current_position_;
  bool current_position_set_{ false };
  geometry_msgs::Point explicit_start_;
  bool explicit_start_set_{ false };
  double mission_start_z_{ 0.0 };
  bool mission_start_z_set_{ false };
  nav_msgs::Path last_path_;
  geometry_msgs::PointStamped last_waypoint_;
  bool path_ready_{ false };
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "uav_saved_map_planner");
  try
  {
    UavSavedMapPlanner planner;
    ros::spin();
  }
  catch (const std::exception& error)
  {
    ROS_FATAL_STREAM("uav_saved_map_planner failed: " << error.what());
    return 1;
  }
  return 0;
}
