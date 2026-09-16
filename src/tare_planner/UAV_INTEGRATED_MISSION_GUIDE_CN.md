# TARE 无人机综合任务操作说明

## 1. 当前功能

1. TARE 固定高度自主探索，规划和输出高度统一使用 `fixed_flight_height`。
2. TARE 已探索图内的 RViz `2D Nav Goal` 指点到达，路径从无人机当前位置开始。
3. 无人机局部实时避障：参考全局路径、实时点云、机体尺寸和动态制动距离选择安全局部路径。
4. 机头对准运动方向、速度/加速度/加加速度限制、起飞稳定等待和 PX4 位置设定点输出。
5. 固定探索起点路径规划：从首次定位点到 RViz 目标生成路径，但不控制无人机。
6. 路径和 Fast-LIO PCD 交接包导出，包含 SHA256、高度策略、机体尺寸和安全边界。
7. 路径回放：另一架无人机按交接路径飞行，仍经过本机实时局部避障和 PX4 飞控。
8. 保存 PCD 地图 A*：在指定飞行高度切障碍层、膨胀安全边界，再规划恒高路径。
9. RViz 同时显示实时点云、TARE 路径、安全路径、固定起点路径、回放路径和保存地图路径。
10. RC 接管锁存、任务 HOLD、定位/点云超时保持、进度超时停止。

探索完成后，TARE 发布完成状态并保持当前位置，不会自动降落。降落必须由操作者在 QGC/遥控器执行，或明确运行：

```bash
rosrun tare_planner uav_mode_command.py land
```

## 2. 控制优先级

从高到低：

1. PX4/QGC/遥控器模式切换和 RC 紧急接管。
2. 仲裁器紧急锁存和 `HOLD`。
3. 局部避障的数据超时、全部候选路径阻塞、制动距离检查。
4. 当前明确选择的任务源：TARE、路线回放或保存地图 A*，三者互斥。
5. TARE 内部任务：当前点指点导航高于自主探索。
6. 固定起点规划只生成文件用路径，永不直接取得飞控控制权。

RC 接管恢复后不会自动继续自主飞行。先把遥控器开关恢复，再执行：

```bash
rosrun tare_planner uav_mission_command.py clear-emergency
rosrun tare_planner uav_mission_command.py hold
```

确认状态正常后再选择自主任务。

## 3. 编译

```bash
cd ~/tare_planner
source /opt/ros/noetic/setup.bash
export LD_LIBRARY_PATH="$HOME/tare_planner/src/tare_planner/or-tools/lib:$LD_LIBRARY_PATH"

# 如果 build/devel 来自另一台机器，才执行下面两行。
rm -rf build devel

catkin_make -DCMAKE_BUILD_TYPE=Release -j2
source devel/setup.bash
```

机载端 `or-tools/lib` 必须是 aarch64 版本，并保持：

```bash
cd ~/tare_planner/src/tare_planner/or-tools/lib
ln -sfn libortools.so.9.8.3296 libortools.so.9
ln -sfn libortools.so.9 libortools.so
```

## 4. 综合启动

先启动 Livox 驱动、FAST-LIO、MAVROS，确认：

```bash
rostopic hz /cloud_registered
rostopic hz /mavros/local_position/odom
rostopic echo -n 1 /mavros/state
```

再启动综合任务栈。下面示例为相对起点飞高 1.0 m：

```bash
source /opt/ros/noetic/setup.bash
source ~/tare_planner/devel/setup.bash
export LD_LIBRARY_PATH="$HOME/tare_planner/src/tare_planner/or-tools/lib:$LD_LIBRARY_PATH"

roslaunch tare_planner tare_uav_integrated_mission.launch \
  rviz:=true \
  default_mission_mode:=tare \
  external_odom_topic:=/mavros/local_position/odom \
  external_cloud_topic:=/cloud_registered \
  run_input_bridge:=true \
  run_static_tf_bridge:=true \
  fixed_flight_height:=1.0 \
  fixed_flight_height_relative_to_start:=true \
  local_path_lookahead:=1.8 \
  uav_horizontal_clearance:=0.65 \
  uav_vertical_clearance:=0.35 \
  max_xy_speed:=0.25 \
  max_xy_accel:=0.20 \
  max_xy_jerk:=0.50 \
  max_z_speed:=0.15 \
  max_z_accel:=0.10 \
  max_z_jerk:=0.20 \
  max_yaw_rate:=0.40 \
  takeoff_stabilization_seconds:=5.0 \
  run_offboard_manager:=false \
  auto_arm:=false \
  rc_override_enabled:=false
```

实机默认由操作者手动解锁并切换 `OFFBOARD`。桥接节点先发布起飞高度目标；达到高度并稳定 5 秒后才发布 `/start_exploration`，不会一边起飞一边开始水平观察。

## 5. 功能切换

### 自主探索 / 当前点指点到达

```bash
rosrun tare_planner uav_mission_command.py explore
```

在 `tare` 模式下点击 RViz `2D Nav Goal`，目标会进入当前 TARE 已探索图的 A* 指点导航；发布：

- `/tare_uav/navigation/path`
- `/tare_uav/navigation/goal`
- `/tare_uav/navigation/reached`

恢复探索：

```bash
rosrun tare_planner uav_mission_command.py explore
```

### 暂停并悬停

```bash
rosrun tare_planner uav_mission_command.py hold
```

### 从探索起点生成可交接路径

```bash
rosrun tare_planner uav_mission_command.py plan
```

此时无人机保持当前位置。在 RViz 点击 `2D Nav Goal` 后，青色 `/tare_uav/fixed_start_path` 从首次定位点开始，不会发给飞控。

导出路径：

```bash
rosrun tare_planner uav_mission_command.py export
```

默认输出：

```text
~/tare_uav_handoff/uav_route.json
~/tare_uav_handoff/manifest.json
```

如需把 Fast-LIO PCD 一起放入交接包，启动时增加：

```bash
map_file:=$HOME/livox_ws/src/FAST_LIO/PCD/scans.pcd
```

应先让 Fast-LIO 完成 PCD 写盘，再调用导出服务，避免复制仍在写入的文件。

### 回放交接路径

把交接目录放到第二架无人机的 `~/tare_uav_handoff`，使用与导出时相同的 `fixed_flight_height` 和相对高度策略启动。验证并开始：

```bash
rosrun tare_planner uav_mission_command.py validate
rosrun tare_planner uav_mission_command.py replay
```

停止回放并悬停：

```bash
rosrun tare_planner uav_mission_command.py replay-stop
```

回放轨迹只是参考路径，实际控制仍经过 `/uav_local_planner`，会使用第二架无人机的实时点云绕开新障碍物。

### 保存 PCD 地图上重新 A*

```bash
roslaunch tare_planner tare_uav_integrated_mission.launch \
  rviz:=true \
  default_mission_mode:=saved_map \
  run_tare_planner:=false \
  run_path_exporter:=false \
  run_saved_map_planner:=true \
  run_bundle_validator:=true \
  map_file:=$HOME/tare_uav_handoff/scans.pcd \
  fixed_flight_height:=1.0 \
  fixed_flight_height_relative_to_start:=true \
  uav_horizontal_clearance:=0.65 \
  uav_vertical_clearance:=0.35
```

然后执行并点击 RViz `2D Nav Goal`：

```bash
rosrun tare_planner uav_mission_command.py saved
```

保存地图 A* 默认以当前里程计位置为起点。离线指定起点时，启动增加 `saved_map_use_explicit_start:=true`，再发布：

```bash
rostopic pub -1 /tare_uav/saved_map/start geometry_msgs/PoseStamped \
"header: {frame_id: 'map'}
pose:
  position: {x: 0.0, y: 0.0, z: 0.0}
  orientation: {w: 1.0}"
```

PCD 只描述障碍表面，不包含严格的“未知/已知自由空间”语义。跨无人机使用保存地图前，必须保证第二架无人机已经定位到同一 `map` 坐标系；必要时通过 `saved_map_offset_x/y/z/yaw` 做已测量的刚体对齐。

## 6. RViz 图例

- `TAREReferencePath`：TARE 探索或当前点导航参考路径。
- `ManualNavigationPath`：当前无人机到 RViz 目标的 TARE 图路径。
- `FixedStartExportPath`（青色）：探索起点到目标，只用于导出。
- `ReplayPath`（紫色）：交接路线回放参考路径。
- `SavedMapAStarPath`（黄色）：保存 PCD 地图 A* 路径。
- `MissionCommandWaypoint`：仲裁后送入局部规划的唯一原始目标。
- `SafePath` / `SafeWaypoint`（绿色）：局部避障最终输出。
- `TAREExploredMap`：TARE 实时规划点云。
- `SavedFastLioMap`：保存的完整 PCD，默认关闭，可在 Displays 中勾选。

## 7. 本次机载端覆盖文件

覆盖原包中的同名文件：

```text
src/tare_planner/CMakeLists.txt
src/tare_planner/package.xml
src/tare_planner/config/uav/                     # 原 config/uav_fixed_height.yaml 已按节点拆分到此目录
src/tare_planner/launch/include/                 # 原 tare_uav_fixed_height.launch 拆出的组件
src/tare_planner/include/sensor_coverage_planner/sensor_coverage_planner_ground.h
src/tare_planner/src/sensor_coverage_planner/sensor_coverage_planner_ground.cpp
src/tare_planner/launch/tare_uav_fixed_height.launch
src/tare_planner/rviz/vehicle_simulator.rviz
src/tare_planner/test/test_uav_manual_navigation_profile.py
```

新增文件：

```text
src/tare_planner/launch/tare_uav_integrated_mission.launch
src/tare_planner/scripts/uav_bundle_validator.py
src/tare_planner/scripts/uav_mission_command.py
src/tare_planner/scripts/uav_mission_mux.py
src/tare_planner/scripts/uav_path_exporter.py
src/tare_planner/scripts/uav_path_replayer.py
src/tare_planner/src/uav_saved_map_planner/uav_saved_map_planner_node.cpp
src/tare_planner/test/test_uav_handoff_profile.py
src/tare_planner/UAV_INTEGRATED_MISSION_GUIDE_CN.md
```

上传后执行：

```bash
chmod +x ~/tare_planner/src/tare_planner/scripts/uav_*.py
```
