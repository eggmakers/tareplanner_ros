# 上游同步指南（UPSTREAM SYNC）

> 目标：把「哪些文件是上游的、被我们改了哪里、下次 `git pull` 上游时怎么办」讲清楚。
> 配套文档：`REFACTOR_PLAN_CN.md`（重构方案与执行记录）。

---

## 1. 基线

| 项 | 值 |
| --- | --- |
| 上游仓库 | `origin` = `https://github.com/caochao39/tare_planner.git` |
| 上游分支 | `melodic-noetic` |
| **基线 commit** | `44500592b86138257273e0cab264e6a847ccefc7`（2024-06-01, "use joystick follow-waypoint-axis to reset waypoint"） |
| 我们的分支 | `refactor/uav-layer` |
| 基线以来的提交 | 6 个（见 `REFACTOR_PLAN_CN.md` 执行记录） |

---

## 2. 改动面总览

统计命令：

```bash
cd /home/l/tare_planner
GIT_PAGER=cat git diff --numstat 4450059 -- src/tare_planner/include src/tare_planner/src
```

| 类别 | 文件数 | 增 | 删 |
| --- | --- | --- | --- |
| **自研新文件**（上游根本不存在） | 7 | **+2059** | −0 |
| **上游文件被改** | **6** | **+570** | **−26** |

自研新文件清单（全部落在上游没有的目录里，永远不会冲突）：

```
include/mission/mission_config.h                 +72
include/mission/mission_path_utils.h             +68
src/mission/mission_config.cpp                   +81
src/mission/mission_path_utils.cpp              +190
include/uav_local_planner/uav_local_planner_core.h   +315
src/uav_local_planner/uav_local_planner_node.cpp     +634
src/uav_saved_map_planner/uav_saved_map_planner_node.cpp +698
```

**结论：约 78% 的自研代码（2059 / 2628 行）完全不碰上游文件。**

---

## 3. 被改动的上游文件逐项说明

### 3.1 `src/sensor_coverage_planner/sensor_coverage_planner_ground.cpp` +479 / −18
### 3.2 `include/sensor_coverage_planner/sensor_coverage_planner_ground.h` +63 / −1

**改了什么**

- 新增 `MissionMode` 任务模式（`NAVIGATION` / 定高任务）与相应状态机、回调、发布器。
- P3a：26 个任务层参数读取 + 校验搬去 `mission_ns::MissionConfig::LoadFromRos()`。
- P3b：定高计算 / 目标净空判断 / 连通图导航路径规划搬到 `mission_ns`，
  类内保留同名薄委托（**头文件签名一字未改，13 处调用点全部不动**）。

**为什么必须改这个文件**

上游这个类本身就是「规划器 + 任务控制 + ROS 接线」混在一起的 god object。
我们需要的无人机任务模式只能挂在它上面，**没有第二种接法**。

**已做的收敛**

| 阶段 | 该文件行数 |
| --- | --- |
| 重构前 | 2014 |
| P3a 后 | 1935 |
| **P3b 后** | **1843** |

参数读取（26 个）与纯逻辑（171 行）已全部移出到 `src/mission/`。

**能否回推上游**：`mission_ns` 抽离部分（P3a/P3b）**可以**，是纯粹的职责拆分、零行为变化。
`MissionMode` 部分属于自研功能，不适合直接回推。

**下次 pull 的冲突风险：高。** 这是唯一需要人工看冲突的文件。

---

### 3.3 `src/viewpoint_manager/viewpoint_manager.cpp` +14 / −4

**改了什么**

```diff
- dimension_ = 2;                                    // 硬编码
+ dimension_ = getParam<int>(nh, "viewpoint_manager/dimension", 2);
+ if (dimension_ < 2 || dimension_ > 3) { ROS_WARN... dimension_ = 2; }
  ...
+ if (dimension_ == 2) { kNumber.z() = 1; kResolution.z() = 0.0; }
  kViewPointNumber = kNumber.x() * kNumber.y() * kNumber.z();
```

**为什么：这是一个真 bug 修复。** 上游把 `dimension_` 硬编码为 2（2D 模式），
但 `kNumber.z()` 仍取配置值 40、`kResolution.z()` 仍取 0.5，于是
`kViewPointNumber = 80 × 80 × 40 = 256000`——2D 场景下白算 40 层。
修正后 2D 时 `kNumber.z()=1`、`kResolution.z()=0`，viewpoint 数降到 6400。

**能否移出上游文件**：不能，必须改这一行附近。
**能否回推上游**：**推荐回推**，这是上游的真实缺陷。
**下次 pull 的冲突风险：中。** 冲突范围就在 `ReadParameters()` 开头 20 行内，很好解。

---

### 3.4 `src/navigation_boundary_publisher/navigationBoundary.cpp` +5 / −1

**改了什么**

```diff
- nh.advertise<geometry_msgs::PolygonStamped>("/navigation_boundary", 5);
+ nh.advertise<geometry_msgs::PolygonStamped>("/navigation_boundary", 1, true);  // latch
  ...
+ boundaryMsgs.header.stamp = ros::Time::now();
+ pubBoundary.publish(boundaryMsgs);          // 启动时立刻发一次
+ ROS_INFO("Published latched navigation boundary with %d points", boundarySize);
```

**为什么**：上游只在「每 `sendBoundaryInterval` 秒」发一次且**不 latch**。
无人机侧订阅者如果晚于发布者启动，就会一直收不到边界，规划器静默退化。
加 `latch=true` + 启动即发一次，彻底消除这个启动竞态。

**能否回推上游**：**推荐回推**，同样是上游真实缺陷。
**下次 pull 的冲突风险：低**（改动集中，前后代码稳定）。

---

### 3.5 `include/planning_env/planning_env.h` +1 / `src/planning_env/planning_env.cpp` +8 / −2

**改了什么**

- 新增 `void PlanningEnv::PublishPlannerCloud()`，作为 `planner_cloud_->Publish()` 的公开入口。
- `kExtractFrontierRange.z()` 的硬编码 `2` 改为可配参数 `kExtractFrontierRangeZ`（默认 2.0）。
- 顺手补了文件末尾缺失的换行。

**为什么必须改这个文件**：`planner_cloud_` 是 `PlanningEnv` 的 **private** 成员，
无人机节点需要把「已注册点云」重新发布出去给地面站看，外部拿不到这个成员，
**只能在类内加这 3 行**。

**能否移出**：不能（private 访问）。
**下次 pull 的冲突风险：低。** 若上游把 `PublishPlannerCloud` 改名或挪走，按新名字重接即可。

> 注意：`kExtractFrontierRangeZ` 这个参数已经进了 `config/uav/tare_core.yaml`，
> 是「参数 215 个与基线一致」的一部分。**回退它会让参数集变化，导致 P2 的验收失效**，
> 因此不要为了「缩小 diff」而回退。

---

## 4. 三条红线

1. **不要**为了「减少上游 diff」去回退 3.3 / 3.4 / 3.5 的改动。
   它们分别修掉了「2D viewpoint 白算 40 层」「边界启动竞态」「参数硬编码」，
   都是有实际后果的真问题，且 `kExtractFrontierRangeZ` 已进入 YAML 参数集。
2. **不要**动 `config/uav/*.yaml` 里的键名或默认值与 `mission_ns` 里的读取名——
   两者的对应关系就是「launch 参数集合与基线逐一相同」这份验收结论的基础。
3. 新增自研代码**一律**放进 `src/mission/`、`include/mission/`、
   `include/uav_local_planner/`、`src/uav_local_planner/`、`src/uav_saved_map_planner/`
   这类上游不存在的目录，不要再往上游文件里加新功能。

---

## 5. 下次合并上游的操作流程

```bash
cd /home/l/tare_planner

# 0) 先确认工作区干净、分支正确
git status --short
git branch --show-current          # refactor/uav-layer

# 1) 拉上游
git fetch origin

# 2) 记录当前上游改动面，作为「合并前基线」
GIT_PAGER=cat git diff --numstat origin/melodic-noetic -- \
  src/tare_planner/include src/tare_planner/src | tee /tmp/before_merge.txt

# 3) 合并
git merge origin/melodic-noetic

# 4) 逐个处理冲突（按第 3 节的优先级）
GIT_PAGER=cat git diff --name-only --diff-filter=U
```

**冲突处理顺序（从易到难）**

1. `planning_env.{h,cpp}` —— 找 `PublishPlannerCloud`，确认没被上游同名函数占用。
2. `navigationBoundary.cpp` —— 确认 advertise 仍是 `1, true`，启动发布那段还在。
3. `viewpoint_manager.cpp` —— 确认 `dimension_` 仍走参数，且 `dimension_ == 2` 分支还在。
4. `sensor_coverage_planner_ground.{h,cpp}` —— **重点**。检查三件事：
   - `PlannerParameters` 是否仍是 `: public mission_ns::MissionConfig`（别被上游还原成字段展开）；
   - `LoadFromRos()` 调用是否还在构造函数里；
   - 三个薄委托（`GetFixedFlightHeightOr` / `ManualGoalHasClearance` / `BuildGraphNavigationPath`）
     的函数体是否还在——若上游改了这三个的逻辑，需要把改动**搬进** `src/mission/mission_path_utils.cpp`。

**合并后必做的验证**（与 P0–P3 用的同一套）

```bash
cd /home/l/tare_planner/src/tare_planner

# 5) 编译
cd /home/l/tare_planner && catkin_make

# 6) 解析结果必须与重构后基线一致：节点 10 个、参数 215 个
#    把你那条 roslaunch 命令的 28 个参数原样接到下面两条命令后面
roslaunch --nodes       tare_planner tare_uav_fixed_height.launch <参数> | sort | wc -l   # 应为 10
roslaunch --dump-params tare_planner tare_uav_fixed_height.launch <参数> | sort | wc -l   # 应为 215

# 7) profile 测试
cd /home/l/tare_planner/src/tare_planner && source /opt/ros/noetic/setup.bash
python3 test/test_uav_manual_navigation_profile.py
python3 test/test_uav_handoff_profile.py
python3 test/test_uav_waypoint_bridge_profile.py
python3 test/test_uav_offboard_manager.py

# 8) 实飞一次（用户验收口径，见 D7）
roslaunch tare_planner tare_uav_fixed_height.launch <28 个参数原样>
```

> 说明：原先的 `refactor_tools/`（`snapshot_launch.sh` + `baseline/` 快照）已删除，
> 以保持仓库干净。它仍在 git 历史里，随时可取回：
> `git checkout 4836397 -- refactor_tools`，取回后即可恢复
> `bash refactor_tools/snapshot_launch.sh after_merge` + `diff` 的完整流程。

---

## 6. 上游改动面速查表

| 上游文件 | +/− | 可以移出吗 | 建议回推上游 | pull 冲突风险 |
| --- | --- | --- | --- | --- |
| `src/sensor_coverage_planner/sensor_coverage_planner_ground.cpp` | +479/−18 | 否（God object，只能挂这里） | 部分可（P3a/P3b 抽离） | **高** |
| `include/sensor_coverage_planner/sensor_coverage_planner_ground.h` | +63/−1 | 否 | 部分可 | **高** |
| `src/viewpoint_manager/viewpoint_manager.cpp` | +14/−4 | 否 | **是** | 中 |
| `src/planning_env/planning_env.cpp` | +8/−2 | 否（private 成员访问） | 是 | 低 |
| `include/planning_env/planning_env.h` | +1/−0 | 否 | 是 | 低 |
| `src/navigation_boundary_publisher/navigationBoundary.cpp` | +5/−1 | 否 | **是** | 低 |
