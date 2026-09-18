#include <cmath>
#include <iostream>
#include <string>
#include <vector>

#include "uav_local_planner/uav_local_planner_core.h"

namespace
{
using uav_local_planner_ns::PlannerConfig;
using uav_local_planner_ns::Point3;
using uav_local_planner_ns::UavLocalPlannerCore;

int failures = 0;

void Expect(bool condition, const std::string& message)
{
  if (!condition)
  {
    std::cerr << "FAILED: " << message << std::endl;
    ++failures;
  }
}

void TestMeasuredFootprint()
{
  Expect(std::abs(UavLocalPlannerCore::PhysicalRadius(0.45298, 0.45298) - 0.3203) < 1e-3,
         "measured footprint half diagonal");
}

void TestReferenceLookahead()
{
  PlannerConfig config;
  config.lookahead_distance = 1.5;
  config.collision_point_threshold = 1;
  UavLocalPlannerCore planner(config);
  const Point3 current{ 0.0, 0.0, 1.0 };
  const Point3 target{ 4.0, 0.0, 1.0 };
  const std::vector<Point3> path{ current, Point3{ 1.0, 0.0, 1.0 }, Point3{ 2.0, 0.0, 1.0 }, target };

  const auto result = planner.Plan(current, target, path, {}, 0.0);

  Expect(result.safe, "unobstructed path must be safe");
  Expect(std::abs(result.reference_target.x - 1.5) < 1e-6, "reference lookahead must be 1.5 m");
  Expect(std::abs(result.target.x - 1.5) < 1e-6 && std::abs(result.target.y) < 1e-6,
         "safe target must follow the path centerline");
}

void TestSideCandidate()
{
  PlannerConfig config;
  config.lookahead_distance = 1.5;
  config.collision_point_threshold = 1;
  UavLocalPlannerCore planner(config);
  const Point3 current{ 0.0, 0.0, 1.0 };
  const Point3 target{ 3.0, 0.0, 1.0 };
  const std::vector<Point3> obstacles{ Point3{ 1.0, 0.0, 1.0 } };

  const auto result = planner.Plan(current, target, {}, obstacles, 0.0);

  Expect(result.safe, "a side candidate should remain available");
  Expect(std::abs(result.target.y) > 0.2, "blocked centerline must select a side candidate");
}

void TestAllDirectionsBlocked()
{
  PlannerConfig config;
  config.lookahead_distance = 1.5;
  config.collision_point_threshold = 1;
  UavLocalPlannerCore planner(config);
  const Point3 current{ 0.0, 0.0, 1.0 };
  const Point3 target{ 3.0, 0.0, 1.0 };
  std::vector<Point3> obstacles;
  for (int degree = -90; degree <= 90; degree += 3)
  {
    const double angle = degree * 3.14159265358979323846 / 180.0;
    obstacles.push_back(Point3{ std::cos(angle), std::sin(angle), 1.0 });
  }

  const auto result = planner.Plan(current, target, {}, obstacles, 0.0);

  Expect(!result.safe, "blocked fan must produce a hold result");
  Expect(std::abs(result.target.x - current.x) < 1e-6 && std::abs(result.target.y - current.y) < 1e-6,
         "blocked result must hold current XY");
}

void TestMovesAwayFromNearbyWall()
{
  PlannerConfig config;
  config.lookahead_distance = 1.5;
  config.collision_point_threshold = 2;
  UavLocalPlannerCore planner(config);
  const Point3 current{ 0.0, 0.0, 1.0 };
  const Point3 target{ 3.0, 0.0, 1.0 };
  std::vector<Point3> obstacles;
  for (double x = 0.2; x <= 1.6; x += 0.1)
  {
    obstacles.push_back(Point3{ x, 0.82, 1.0 });
    obstacles.push_back(Point3{ x, -2.0, 1.0 });
  }

  const auto result = planner.Plan(current, target, {}, obstacles, 0.0);

  Expect(result.safe, "near-wall corridor must retain a safe path");
  Expect(result.target.y < -0.1, "clearance scoring must move away from the nearby wall");
}

void TestDynamicBrakingClearance()
{
  PlannerConfig config;
  config.collision_point_threshold = 1;
  UavLocalPlannerCore stopped_planner(config);
  UavLocalPlannerCore moving_planner(config);
  const Point3 current{ 0.0, 0.0, 1.0 };
  const Point3 target{ 2.0, 0.0, 1.0 };

  const auto stopped = stopped_planner.Plan(current, target, {}, {}, 0.0);
  const auto moving = moving_planner.Plan(current, target, {}, {}, 0.2);

  Expect(std::abs(stopped.effective_clearance - 0.75) < 1e-6, "stationary clearance must match configured value");
  Expect(moving.effective_clearance > stopped.effective_clearance + 0.2,
         "moving clearance must include reaction and braking distance");
}
}  // namespace

int main()
{
  TestMeasuredFootprint();
  TestReferenceLookahead();
  TestSideCandidate();
  TestAllDirectionsBlocked();
  TestMovesAwayFromNearbyWall();
  TestDynamicBrakingClearance();
  if (failures == 0)
  {
    std::cout << "uav_local_planner_core tests passed" << std::endl;
  }
  return failures == 0 ? 0 : 1;
}
