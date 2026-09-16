#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace uav_local_planner_ns
{
struct Point3
{
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};

struct PlannerConfig
{
  double lookahead_distance = 1.5;
  double horizontal_clearance = 0.75;
  double vertical_clearance = 0.35;
  double max_deceleration = 0.10;
  double reaction_time = 0.20;
  double self_filter_radius = 0.28;
  double escape_improvement = 0.08;
  double candidate_angle_step = 3.14159265358979323846 / 12.0;
  int candidate_angle_count = 6;
  int collision_point_threshold = 2;
  double path_weight = 1.0;
  double goal_weight = 0.20;
  double heading_weight = 0.30;
  double clearance_weight = 5.0;
  double switch_weight = 0.30;
};

struct Candidate
{
  Point3 target;
  double heading_offset = 0.0;
  double minimum_clearance = std::numeric_limits<double>::infinity();
  int collision_points = 0;
  double score = std::numeric_limits<double>::infinity();
  bool safe = false;
};

struct PlanResult
{
  bool safe = false;
  Point3 target;
  Point3 reference_target;
  double effective_clearance = 0.0;
  double minimum_clearance = 0.0;
  int collision_points = 0;
  std::vector<Candidate> candidates;
};

class UavLocalPlannerCore
{
public:
  explicit UavLocalPlannerCore(const PlannerConfig& config) : config_(config)
  {
  }

  static double DistanceXY(const Point3& lhs, const Point3& rhs)
  {
    return std::hypot(lhs.x - rhs.x, lhs.y - rhs.y);
  }

  static double PhysicalRadius(double length, double width)
  {
    return 0.5 * std::hypot(length, width);
  }

  PlanResult Plan(const Point3& current, const Point3& raw_target, const std::vector<Point3>& reference_path,
                  const std::vector<Point3>& obstacles, double current_speed)
  {
    PlanResult result;
    result.target = current;
    result.target.z = raw_target.z;
    result.reference_target = SelectReferenceTarget(current, raw_target, reference_path);

    const double reference_distance = DistanceXY(current, result.reference_target);
    if (reference_distance < 1e-3)
    {
      result.safe = true;
      result.target = result.reference_target;
      result.minimum_clearance = std::numeric_limits<double>::infinity();
      return result;
    }

    const double braking_distance = current_speed * config_.reaction_time +
                                    current_speed * current_speed /
                                        (2.0 * std::max(1e-3, config_.max_deceleration));
    result.effective_clearance = config_.horizontal_clearance + braking_distance;

    const double reference_heading = std::atan2(result.reference_target.y - current.y,
                                                result.reference_target.x - current.x);
    const double candidate_distance = std::min(config_.lookahead_distance, reference_distance);
    const std::vector<double> offsets = CandidateOffsets();

    int best_index = -1;
    for (double offset : offsets)
    {
      Candidate candidate;
      candidate.heading_offset = offset;
      const double heading = reference_heading + offset;
      candidate.target.x = current.x + candidate_distance * std::cos(heading);
      candidate.target.y = current.y + candidate_distance * std::sin(heading);
      candidate.target.z = raw_target.z;
      CheckSegment(current, candidate.target, obstacles, result.effective_clearance, candidate.minimum_clearance,
                   candidate.collision_points);
      candidate.safe = candidate.collision_points < config_.collision_point_threshold;

      const double path_error = DistanceXY(candidate.target, result.reference_target);
      const double goal_error = DistanceXY(candidate.target, raw_target);
      const double bounded_clearance = std::min(candidate.minimum_clearance, 2.0 * result.effective_clearance);
      candidate.score = config_.path_weight * path_error + config_.goal_weight * goal_error +
                        config_.heading_weight * std::abs(offset) - config_.clearance_weight * bounded_clearance +
                        config_.switch_weight * std::abs(offset - last_heading_offset_);

      result.candidates.push_back(candidate);
      if (candidate.safe && (best_index < 0 || candidate.score < result.candidates[best_index].score))
      {
        best_index = static_cast<int>(result.candidates.size()) - 1;
      }
    }

    if (best_index >= 0)
    {
      const Candidate& best = result.candidates[best_index];
      result.safe = true;
      result.target = best.target;
      result.minimum_clearance = best.minimum_clearance;
      result.collision_points = best.collision_points;
      last_heading_offset_ = best.heading_offset;
    }
    else
    {
      result.minimum_clearance = std::numeric_limits<double>::infinity();
      result.collision_points = 0;
      for (const Candidate& candidate : result.candidates)
      {
        result.minimum_clearance = std::min(result.minimum_clearance, candidate.minimum_clearance);
        result.collision_points = std::max(result.collision_points, candidate.collision_points);
      }
    }
    return result;
  }

private:
  struct PathSample
  {
    Point3 point;
    bool valid = false;
  };

  PlannerConfig config_;
  double last_heading_offset_ = 0.0;

  std::vector<double> CandidateOffsets() const
  {
    std::vector<double> offsets{ 0.0 };
    for (int index = 1; index <= config_.candidate_angle_count; ++index)
    {
      const double offset = config_.candidate_angle_step * index;
      offsets.push_back(offset);
      offsets.push_back(-offset);
    }
    return offsets;
  }

  Point3 SelectReferenceTarget(const Point3& current, const Point3& raw_target,
                               const std::vector<Point3>& path) const
  {
    if (path.size() < 2)
    {
      return LimitDirectTarget(current, raw_target);
    }

    int nearest_index = 0;
    double nearest_distance = std::numeric_limits<double>::infinity();
    for (int index = 0; index < static_cast<int>(path.size()); ++index)
    {
      const double distance = DistanceXY(current, path[index]);
      if (distance < nearest_distance)
      {
        nearest_distance = distance;
        nearest_index = index;
      }
    }

    const PathSample forward = SamplePath(current, path, nearest_index, 1);
    const PathSample backward = SamplePath(current, path, nearest_index, -1);
    if (!forward.valid && !backward.valid)
    {
      return LimitDirectTarget(current, raw_target);
    }
    if (!forward.valid)
    {
      return backward.point;
    }
    if (!backward.valid)
    {
      return forward.point;
    }

    const double direct_heading = std::atan2(raw_target.y - current.y, raw_target.x - current.x);
    const auto direction_score = [&](const Point3& point) {
      const double heading = std::atan2(point.y - current.y, point.x - current.x);
      return DistanceXY(point, raw_target) + 0.25 * std::abs(WrapAngle(heading - direct_heading));
    };
    return direction_score(forward.point) <= direction_score(backward.point) ? forward.point : backward.point;
  }

  Point3 LimitDirectTarget(const Point3& current, const Point3& target) const
  {
    const double distance = DistanceXY(current, target);
    if (distance <= config_.lookahead_distance || distance < 1e-6)
    {
      return target;
    }
    const double scale = config_.lookahead_distance / distance;
    return Point3{ current.x + (target.x - current.x) * scale, current.y + (target.y - current.y) * scale, target.z };
  }

  PathSample SamplePath(const Point3& current, const std::vector<Point3>& path, int nearest_index, int direction) const
  {
    PathSample sample;
    Point3 previous = current;
    double accumulated = 0.0;
    for (int index = nearest_index; index >= 0 && index < static_cast<int>(path.size()); index += direction)
    {
      const Point3& next = path[index];
      const double segment_length = DistanceXY(previous, next);
      if (segment_length > 1e-6)
      {
        if (accumulated + segment_length >= config_.lookahead_distance)
        {
          const double ratio = (config_.lookahead_distance - accumulated) / segment_length;
          sample.point = Point3{ previous.x + (next.x - previous.x) * ratio,
                                 previous.y + (next.y - previous.y) * ratio,
                                 next.z };
          sample.valid = true;
          return sample;
        }
        accumulated += segment_length;
        sample.point = next;
        sample.valid = true;
      }
      previous = next;
    }
    return sample;
  }

  void CheckSegment(const Point3& start, const Point3& end, const std::vector<Point3>& obstacles,
                    double effective_clearance, double& minimum_clearance, int& collision_points) const
  {
    minimum_clearance = std::numeric_limits<double>::infinity();
    collision_points = 0;
    const double segment_x = end.x - start.x;
    const double segment_y = end.y - start.y;
    const double segment_norm_sq = segment_x * segment_x + segment_y * segment_y;

    for (const Point3& obstacle : obstacles)
    {
      const double start_distance = DistanceXY(start, obstacle);
      if (start_distance < config_.self_filter_radius)
      {
        continue;
      }

      double projection = 0.0;
      if (segment_norm_sq > 1e-9)
      {
        projection = ((obstacle.x - start.x) * segment_x + (obstacle.y - start.y) * segment_y) /
                     segment_norm_sq;
        projection = std::max(0.0, std::min(1.0, projection));
      }
      const double closest_x = start.x + projection * segment_x;
      const double closest_y = start.y + projection * segment_y;
      const double closest_z = start.z + projection * (end.z - start.z);
      if (std::abs(obstacle.z - closest_z) > config_.vertical_clearance)
      {
        continue;
      }

      const double clearance = std::hypot(obstacle.x - closest_x, obstacle.y - closest_y);
      // Ignore the immediate start cap for clearance scoring so a candidate
      // that moves away from a nearby wall is preferred. It is still included
      // in the hard collision test below.
      if (projection >= 0.15)
      {
        minimum_clearance = std::min(minimum_clearance, clearance);
      }
      if (clearance >= effective_clearance)
      {
        continue;
      }

      const double end_distance = DistanceXY(end, obstacle);
      const bool escaping_existing_intrusion = start_distance < effective_clearance &&
                                               end_distance > start_distance + config_.escape_improvement;
      if (!escaping_existing_intrusion)
      {
        ++collision_points;
      }
    }
  }

  static double WrapAngle(double angle)
  {
    return std::atan2(std::sin(angle), std::cos(angle));
  }
};
}  // namespace uav_local_planner_ns
