#!/usr/bin/env bash
# P3 侦察: 列出 cpp 中所有顶层函数定义的起始行号，用于定位需要抽取的代码块。
cd "$(dirname "$0")/.." || exit 1

F=src/tare_planner/src/sensor_coverage_planner/sensor_coverage_planner_ground.cpp

echo "==== cpp 顶层定义行号 ===="
grep -nE '^[A-Za-z_].*SensorCoveragePlanner3D::' "$F"

echo
echo "==== 文件总行数 ===="
wc -l "$F"
