#!/usr/bin/env bash
# P3 侦察: 定位 ReadParameters、自研函数的所有调用点，以及若干需要改动的钩子。
cd "$(dirname "$0")/.." || exit 1

PKG=src/tare_planner
F="$PKG/src/sensor_coverage_planner/sensor_coverage_planner_ground.cpp"
H="$PKG/include/sensor_coverage_planner/sensor_coverage_planner_ground.h"

echo "==== ReadParameters 定义位置 ===="
grep -rn 'ReadParameters' "$PKG/include" "$PKG/src" | head

echo
echo "==== GetFixedFlightHeightOr 全部调用点 ===="
grep -n 'GetFixedFlightHeightOr' "$F" "$H"

echo
echo "==== kUseFixedFlightHeight / kFixedFlightHeight 全部引用 ===="
grep -n 'kUseFixedFlightHeight\|kFixedFlightHeight' "$F" "$H"

echo
echo "==== MissionMode / ExecuteManualNavigation / exploration_completion_pending_ ===="
grep -n 'MissionMode\|ExecuteManualNavigation\|exploration_completion_pending_\|exploration_completion_candidate_time_\|initial_position_set_' "$F" "$H"

echo
echo "==== manual_goal_ / hold_position_ / mission_status_ 引用 ===="
grep -n 'manual_goal_\|hold_position_\|mission_status_\|manual_goal_reached_\|manual_goal_arrival_pending_\|manual_navigation_path_' "$F" "$H"

echo
echo "==== misc_utils.h 里的 getParam 是否只是通用 helper ===="
grep -n 'getParam' "$PKG/include/utils/misc_utils.h" | head -8
