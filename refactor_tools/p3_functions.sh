#!/usr/bin/env bash
# P3 侦察: 列出相对上游基线新增的函数定义、方法声明，以及被修改掉的原有代码行。
cd "$(dirname "$0")/.." || exit 1

BASE=4450059
PKG=src/tare_planner
F="$PKG/src/sensor_coverage_planner/sensor_coverage_planner_ground.cpp"
H="$PKG/include/sensor_coverage_planner/sensor_coverage_planner_ground.h"

echo "==== [1] 新增函数定义 (cpp) ===="
git diff "$BASE" -- "$F" | grep '^+[A-Za-z_].*::' | sed 's/^+//'

echo
echo "==== [2] 新增方法声明 (header) ===="
git diff "$BASE" -- "$H" | grep -E '^\+ +(void|bool|double|int|std::|geometry_msgs|nav_msgs|Eigen)' | sed 's/^+//'

echo
echo "==== [3] 被修改/删除的原有代码行 (cpp) ===="
git diff "$BASE" -- "$F" | grep '^-[^-]' | sed 's/^-//'

echo
echo "==== [4] 被修改/删除的原有代码行 (header) ===="
git diff "$BASE" -- "$H" | grep '^-[^-]' | sed 's/^-//'

echo
echo "==== [5] viewpoint / planning_env / navBoundary 的全部改动 ===="
git diff "$BASE" -- "$PKG/src/viewpoint_manager/viewpoint_manager.cpp" \
                     "$PKG/include/planning_env/planning_env.h" \
                     "$PKG/src/planning_env/planning_env.cpp" \
                     "$PKG/src/navigation_boundary_publisher/navigationBoundary.cpp" \
  | grep -E '^[+-]' | grep -vE '^(\+\+\+|---)'
