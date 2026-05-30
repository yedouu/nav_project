# 自主探索踩坑记录

> 日期：2026-05-30
> 环境：ROS Noetic + Ubuntu 20.04 + Gazebo
> 机器人：TurtleBot3 Waffle Pi
> 局部规划器：DWA
> 探索方案：explore_lite
> SLAM：Cartographer

---

## 目录

1. [explore_lite 收不到地图 —— 话题名称解析问题](#1-explore_lite-收不到地图--话题名称解析问题)
2. [Cartographer 地图没有 free 细胞 —— 概率阈值问题](#2-cartographer-地图没有-free-细胞--概率阈值问题)
3. [occupied_space_override 参数不存在 —— 编译版本问题](#3-occupied_space_override-参数不存在--编译版本问题)
4. [explore_lite 找到 frontier 却发不出目标 —— 死锁问题](#4-explore_lite-找到-frontier-却发不出目标--死锁问题)
5. [导航中 TF 时间戳报错 —— 帧转换问题](#5-导航中-tf-时间戳报错--帧转换问题)
6. [问题总结](#6-问题总结)

---

## 1. explore_lite 收不到地图

**日期：** 2026-05-30
**错误现象：** explore_lite 启动后一直打印 `Waiting for costmap to become available, topic: map`，永远不会收到地图。

**原因分析：**

explore_lite 的 `costmap_topic` 参数默认值是 `map`（相对路径），而 explore 节点使用私有 NodeHandle 进行订阅。在 ROS 中，相对路径 `map` 在私有命名空间下解析为 `/explore/map`，而非全局的 `/map`。

```cpp
// explore.cpp 构造函数
Explore::Explore()
  : private_nh_("~")          // 私有命名空间
  , relative_nh_()            // 默认命名空间
{ }

// costmap_client 初始化
costmap_client_(private_nh_, relative_nh_, &tf_listener_)
//                                    ↑
//                          relative_nh_ 用于订阅
//                          解析 "map" → /explore/map
```

但 Cartographer 发布的是全局话题 `/map`。订阅者和发布者话题不匹配，explore 永远收不到地图。

**解决方案：** 在参数前加 `/`，明确指定全局话题。

```xml
<!-- explore.launch -->
<param name="costmap_topic" value="/map"/>
<param name="costmap_updates_topic" value="/map_updates"/>
```

**验证方法：** 启动后观察 ROS 话题列表，确认 explore 节点已订阅 `/map`。

```
rostopic info /map
# 应看到 Subscribers 中包含 /explore
```

---

## 2. Cartographer 地图没有 free 细胞

**日期：** 2026-05-30
**错误现象：** 话题订阅修好后，explore_lite 仍然找不到 frontier。用 `check_map.py` 检查地图发现 free 细胞为 0。

```
地图 137x122 @ 0.05m/px
  unknown: 11856 cells (70.9%)
  free:    0 cells (0.0%)        ← 没有 free 细胞！
  occ:     360 cells (2.2%)
  frontiers: 0 cells (0.00 m²)
```

**原因分析：**

Cartographer 的 occupancy grid 节点使用固定公式将子图概率转换为占用值（`msg_conversion.cc` 第 410 行）：

```cpp
const int value =
    observed == 0
        ? -1    // 未知
        : ::cartographer::common::RoundToInt((1. - color / 255.) * 100.);
```

默认概率阈值被硬编码在映射公式中。当 `occupied_space_override` 等效值为 0.5 时，Cartographer 将超过 50% 概率的区域都标为 occupied，几乎没有区域被标为 free。这导致：

- 激光扫过的区域几乎全部标记为 occupied
- 已探索和未探索之间的边界（frontier）不存在
- explore_lite 找不到探索目标

**解决方案：** 修改 Cartographer 源码，添加 `occupied_space_override` 参数，将阈值提高到 0.7（只有 >70% 确信才标 occupied）。

**改动文件：** `catkin_cartographer/src/cartographer_ros/cartographer_ros/cartographer_ros/occupancy_grid_node_main.cc`

```cpp
// 新增参数声明（第 42-43 行）
DEFINE_double(occupied_space_override, 0.5,
              "Probability at which to consider cells as occupied.");

// DrawAndPublish 中应用阈值（第 173-179 行）
const double occ_threshold = FLAGS_occupied_space_override;
for (auto& cell : msg_ptr->data) {
  if (cell >= 0) {
    cell = (cell / 100.0 > occ_threshold) ? 100 : 0;
  }
}
```

**使用方式：** 在 launch 文件中传参

```xml
<node pkg="cartographer_ros" type="cartographer_occupancy_grid_node"
      name="cartographer_occupancy_grid_node" 
      args="-resolution 0.05 -occupied_space_override 0.7"/>
```

**验证结果：** 修改后地图恢复正常

```
地图 140x128 @ 0.05m/px
  unknown: 11997 cells (66.9%)
  free:    4710 cells (26.3%)    ← free 细胞恢复正常！
  occ:     565 cells (3.2%)
  frontiers: 16 cells (0.04 m²)
```

---

## 3. occupied_space_override 参数不存在

**日期：** 2026-05-30
**错误现象：** 启动 cartographer_occupancy_grid_node 时传入 `-occupied_space_override 0.7`，节点崩溃。

```
ERROR: unknown command line flag 'occupied_space_override'
```

也试了 `--occupied_space_override 0.7`，同样报错。

**原因分析：**

编译安装的 cartographer_ros 版本较旧（master 分支），`occupancy_grid_node_main.cc` 中没有 `DEFINE_double(occupied_space_override, ...)` 声明。该参数是高版本才加入的。

通过查看源码确认现有支持的参数：

```cpp
DEFINE_double(resolution, 0.05, ...);
DEFINE_double(publish_period_sec, 1.0, ...);
DEFINE_bool(include_frozen_submaps, true, ...);
DEFINE_bool(include_unfrozen_submaps, true, ...);
DEFINE_string(occupancy_grid_topic, ..., ...);
// ← 没有 occupied_space_override
```

**解决方案：** 修改 `occupancy_grid_node_main.cc` 源码，手动添加该参数，并在 `DrawAndPublish()` 中应用阈值。添加后需重新编译：

```bash
catkin_make_isolated --install --use-ninja --pkg cartographer_ros
```

**验证方法：**

```bash
cartographer_occupancy_grid_node --help 2>&1 | grep occupied
# 输出: -occupied_space_override (Probability at which to consider cells as
#        occupied.) type: double default: 0.5
```

---

## 4. explore_lite 找到 frontier 却发不出目标

**日期：** 2026-05-30
**错误现象：** 地图有 free 细胞后，explore_lite 仍不发目标。`check_map.py` 发现 frontier 只有 0.04 m²，但 `min_frontier_size` 设的是 0.5 m²。

**原因分析：**

explore_lite 的前沿搜索使用 `min_frontier_size` 参数过滤过小的 frontier。默认值 0.5 m² 对于初始探索阶段来说太大。初始时 Cartographer 只在机器人周围扫描出一个小区域，前沿面积很小（0.04 m²），全部被过滤。

此外，这是一个死锁问题：

```
需要大 frontier → 机器人需要移动 → 需要 explore 发目标 → 需要大 frontier
```

**解决方案：** 将 `min_frontier_size` 从 0.5 降低到 0.02。

```xml
<param name="min_frontier_size" value="0.02"/>
```

**后续优化：** 当机器人开始移动后，地图扩大，frontier 自然变大。可以在首次探索成功后调回更大的值。

---

## 5. 导航中 TF 时间戳报错

**日期：** 2026-05-30
**错误现象：** 探索过程中机器人突然卡住，控制台出现大量 TF 报错。

```
[ERROR] Extrapolation Error: Lookup would require extrapolation 0.020s into the future.
  Requested time 314.900 but the latest data is at time 314.880,
  when looking up transform from frame [odom] to frame [map]

[WARN] Could not transform the global plan to the frame of the controller
[ERROR] Could not get local plan
```

**原因分析：**

局部 costmap 使用 `global_frame: odom`，全局路径（global plan）在 `map` 帧。DWA/TEB 控制器每次需要将全局路径从 `map` 帧转换到 `odom` 帧才能规划局部轨迹。

转换需要查找 `map → odom` 的 TF，这个 TF 由 Cartographer 发布。当 Cartographer 处理负载较大时（地图扩大、回环检测），TF 延迟从几毫秒涨到几十毫秒，转换失败，控制器无法生成局部轨迹，机器人停止。

```
             ┌──────────────────┐
全局路径      │  map 帧          │
             └────────┬─────────┘
                      │ 需要 map → odom TF
                      ▼
             ┌──────────────────┐
局部 costmap  │  odom 帧         │  ← 转换失败 → 无法规划
             └──────────────────┘
```

**解决方案：** 将局部 costmap 的 `global_frame` 从 `odom` 改为 `map`。

```yaml
local_costmap:
  global_frame: map          # 原来是 odom
  robot_base_frame: base_footprint
  rolling_window: true
  width: 3
  height: 3
  resolution: 0.05
```

**原理：** 两个帧一致后，控制器不需要进行帧转换，TF 时间戳错误彻底消除。

```
             ┌──────────────────┐
全局路径      │  map 帧          │
             └────────┬─────────┘
                      │ 同一帧，无需转换
                      ▼
             ┌──────────────────┐
局部 costmap  │  map 帧          │  ← 直接使用，零误差
             └──────────────────┘
```

**注意事项：** 此改动在 costmap 有正常 free 细胞的前提下才有效。如果地图全是 occupied，无论什么帧都无法规划。

---

## 6. 问题总结

### 问题根因拓扑图

```
探索不动 ─┬─ 话题订阅(相对路径) ─────────── 加 /
          ├─ 地图无 free 细胞 ──────────── 改阈值 0.7
          ├─ min_frontier_size 太大 ────── 降到 0.02
          └─ 阈值参数不存在 ────────────── 改源码

导航卡住 ── TF 帧转换错误 ────────────── local costmap 用 map 帧
```

### 最终生效的关键改动

| 改动 | 解决的问题 |
|------|-----------|
| `costmap_topic: /map`（加前导 /） | explore 订阅不到 Cartographer 地图 |
| `occupied_space_override: 0.7`（改源码实现） | 地图 0% free 细胞 |
| `min_frontier_size: 0.02` | 第一次探索需要小 frontier 起步 |
| `local_costmap.global_frame: map` | 运行时 TF 转换报错导致卡住 |
| `inflation_radius: 0.3`（根据车宽） | 绕大圈、窄通道过不去 |
| `planner_frequency: 0.5` | 重规划太频繁导致路径抖动 |

### 这些改动在仓库中的位置

```
nav_project/
├── catkin_turtlebot3/src/turtlebot3/turtlebot3_slam/launch/
│   ├── turtlebot3_cartographer.launch   ← SLAM + 导航 + RViz
│   ├── xtdrone_world.launch             ← 世界 + 机器人
│   └── explore.launch                    ← 探索参数
├── catkin_turtlebot3/src/turtlebot3/turtlebot3_navigation/param/
│   ├── local_costmap_params.yaml         ← global_frame: map
│   ├── move_base_params.yaml             ← planner_frequency
│   └── costmap_common_params_waffle_pi.yaml ← inflation_radius
├── catkin_cartographer/src/cartographer_ros/cartographer_ros/
│   └── cartographer_ros/
│       └── occupancy_grid_node_main.cc   ← occupied_space_override
└── XTDrone 相关/
    ├── xtdrone_maps/     ← indoor3~5, mission1 地图
    └── xtdrone_worlds/   ← indoor1~7 Gazebo 世界
```
