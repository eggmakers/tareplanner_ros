#!/usr/bin/env bash
# 生成 P3 所需的"上游基线 vs 当前"差异文件，便于逐段审阅。
#
# 用法: bash refactor_tools/make_p3_diffs.sh
#
# BASE = 4450059 是合并上游 melodic-noetic 的最后一个提交，
#        即所有自研改动都发生在它之后。
cd "$(dirname "$0")/.." || exit 1

BASE=4450059
OUT=refactor_tools/p3
PKG=src/tare_planner

mkdir -p "$OUT"

git diff "$BASE" -- "$PKG/include/sensor_coverage_planner/sensor_coverage_planner_ground.h"  > "$OUT/01_header.diff"
git diff "$BASE" -- "$PKG/src/sensor_coverage_planner/sensor_coverage_planner_ground.cpp"   > "$OUT/02_planner.diff"
git diff "$BASE" -- "$PKG/src/viewpoint_manager/viewpoint_manager.cpp"                      > "$OUT/03_viewpoint.diff"
git diff "$BASE" -- "$PKG/include/planning_env/planning_env.h"                              > "$OUT/04_planning_env_h.diff"
git diff "$BASE" -- "$PKG/src/planning_env/planning_env.cpp"                                > "$OUT/05_planning_env_cpp.diff"
git diff "$BASE" -- "$PKG/src/navigation_boundary_publisher/navigationBoundary.cpp"         > "$OUT/06_navboundary.diff"

echo "== line counts (added / removed) =="
for f in "$OUT"/*.diff; do
    printf '%-46s +%-5s -%s\n' "$(basename "$f")" \
        "$(grep -c '^+[^+]' "$f")" "$(grep -c '^-[^-]' "$f")"
done
