#!/usr/bin/env bash
# 抓取 launch 解析快照 (节点 / 参数 / 加载的文件 / args), 用于重构前后 diff。
#
# 用法:  bash refactor_tools/snapshot_launch.sh <输出目录名>
#   bash refactor_tools/snapshot_launch.sh baseline      # 重构前
#   bash refactor_tools/snapshot_launch.sh after_p2      # 重构后
#
# 参数取自 refactor_tools/args_fixed_height.txt (即用户验收命令的等价展开)。
# 注意: 不要开 set -u, ROS 的 setup.bash 会引用未定义变量。

cd "$(dirname "$0")/.." || exit 1

# shellcheck disable=SC1091
source /opt/ros/noetic/setup.bash
if [ -f devel/setup.bash ]; then
  # shellcheck disable=SC1091
  source devel/setup.bash
fi

OUT_DIR="refactor_tools/${1:-baseline}"
ARGS_FILE=refactor_tools/args_fixed_height.txt
PKG=tare_planner
LAUNCH=tare_uav_fixed_height.launch

mkdir -p "$OUT_DIR"

# 展开参数文件为 roslaunch 的 arg:=value 列表
ARGS=$(grep -v '^[[:space:]]*#' "$ARGS_FILE" | grep -v '^[[:space:]]*$' | tr -d ' \r' | tr '\n' ' ')

echo "== snapshot -> $OUT_DIR =="
echo "args: $(echo "$ARGS" | tr ' ' '\n' | grep -c .) overrides"

roslaunch --nodes       "$PKG" "$LAUNCH" $ARGS 2>&1 | sort > "$OUT_DIR/nodes.txt"
roslaunch --files       "$PKG" "$LAUNCH" $ARGS 2>&1 | sort > "$OUT_DIR/files.txt"
roslaunch --ros-args    "$PKG" "$LAUNCH" $ARGS 2>&1 | sort > "$OUT_DIR/ros_args.txt"
roslaunch --dump-params "$PKG" "$LAUNCH" $ARGS 2>&1 | sort > "$OUT_DIR/params.txt"

wc -l "$OUT_DIR"/*.txt
