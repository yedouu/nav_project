# scan_to_map — ICP 定位 + 导航

基于 SCAU-RM 的 ICP scan-to-map 定位方案，移植适配 TurtleBot3 Waffle Pi。

## 已实现功能

### ICP 定位
- 加载先验地图（OccupancyGrid）→ PCL 点云
- 实时激光（LaserScan）→ map 坐标系点云
- VoxelGrid 降采样 + 动态障碍点剔除
- PCL ICP 匹配 scan ↔ map（x, y, yaw 修正）
- 多重质量检查：距离/角度/fitness score/角速度阈值过滤
- 发布 `/location_match`（PoseWithCovarianceStamped, frame_id=map）

### robot_localization EKF 融合
- `pose0: /location_match`（ICP 结果, x, y, yaw）
- `odom0: /odom`（轮式里程计速度, differential 模式）
- `world_frame: map`
- `smooth_lagged_data: true`（补偿 ICP 计算延迟）
- `use_control: true`（控制命令辅助预测）

### 重定位
- 暴力旋转搜索：0°~350°，步长 10°，共 36 次 ICP
- 加权评分：w1*score + w2*distance + w3*yaw
- 选最优结果发布到 `/initialpose`（latched）
- 自动触发：连续 10 帧 ICP 匹配失败后自动重定位
- 线程安全：condition_variable 同步 service 和 scan 回调

### 导航（move_base + TEB）
- global_planner（A* 全局路径）
- teb_local_planner（局部避障 + 轨迹优化）
- costmap：静态地图 + 障碍物层 + 膨胀层
- RViz 显示：全局路径（蓝线）、局部轨迹（红线）、代价地图

### RViz 可视化
| 层 | 颜色 | 话题 |
|----|------|------|
| Map | 黑白 | /map |
| LaserScan | 彩色 | /scan |
| MapPointCloud | 蓝色 | /map_pointcloud |
| ScanPointCloud | 红色 | /scan_pointcloud |
| ICP_PointCloud | 绿色 | /icp_pointcloud |
| RemovalPointCloud | 黄色 | /removal_pointcloud |
| ICP_Pose | 绿色坐标轴 | /location_match |
| RotateClouds | 紫色 | /rotate_pointcloud |
| RelocateCandidates | 黄色坐标轴 | /relocate_visual_pose |
| RobotModel | 灰色 | robot_description |
| TF | — | /tf |
| LocalCostmap | 黑底 | /move_base/local_costmap/costmap |
| GlobalCostmap | 彩色 | /move_base/global_costmap/costmap |
| GlobalPlan | 蓝线 | /move_base/GlobalPlanner/plan |
| LocalPlan | 红线 | /move_base/TebLocalPlannerROS/local_plan |

## 包结构

```
scan_to_map/
├── CMakeLists.txt
├── package.xml
├── include/scan_to_map/
│   └── scan_to_map_icp.h
├── src/
│   └── scan_to_map_icp.cpp          # 主节点（ICP + 重定位）
├── launch/
│   ├── scan_to_map.launch           # 一键定位+导航+RViz
│   └── robot_localization_icp.launch # 仅 EKF
├── param/
│   ├── icp_params.yaml              # ICP 参数
│   ├── robot_localization.yaml      # EKF 参数
│   ├── costmap_common_params.yaml   # costmap 通用
│   ├── global_costmap_params.yaml   # 全局 costmap
│   ├── local_costmap_params.yaml    # 局部 costmap
│   └── teb_local_planner_params.yaml# TEB 规划器参数
├── msg/
│   └── LocationInfo.msg            # 定位诊断消息
└── rviz/
    └── scan_to_map.rviz            # RViz 配置
```

## 启动命令

### Gazebo + 定位 + 导航

```bash
# 终端 1 — Gazebo indoor3 场景
source ~/nav_project/catkin_turtlebot3/devel/setup.bash
export TURTLEBOT3_MODEL=waffle_pi
roslaunch turtlebot3_slam xtdrone_world.launch

# 终端 2 — 定位 + 导航 + RViz（等 Gazebo 加载完）
source ~/nav_project/catkin_turtlebot3/devel/setup.bash
roslaunch scan_to_map scan_to_map.launch
```

### 键盘控制

```bash
source ~/nav_project/catkin_turtlebot3/devel/setup.bash
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
```

### 2D Nav Goal（导航目标）

RViz 工具栏 → 点 "2D Nav Goal" → 在地图上点目标位置 → 机器人自动规划路径并避障导航过去。

### 更换地图

```bash
roslaunch scan_to_map scan_to_map.launch map_file:=/path/to/your_map.yaml
```

默认地图：`~/nav_project/xtdrone_maps/indoor3.yaml`

### 手动触发重定位

```bash
rosservice call /relocalization "{}"
```

自动重定位：`icp_params.yaml` 中 `Loss_Num_Threshold: 10`（连续 10 帧匹配失败后自动触发）。

## 关键参数

`param/icp_params.yaml`：

| 参数 | 说明 | 默认值 |
|------|------|--------|
| Maximum_Iterations | ICP 迭代次数 | 50 |
| SCORE_THRESHOLD_MAX | 匹配分数上限 | 0.3 |
| VoxelGridRemoval_LeafSize | 体素滤波边长（m） | 0.05 |
| ObstacleRemoval_Distance_Max | 障碍剔除距离（m） | 2.0 |
| DIST_THRESHOLD | 最小变换距离（m） | 0.01 |
| Loss_Num_Threshold | 自动重定位触发阈值 | 10 |
| Relocation_Score_Threshold_Max | 重定位成功分数上限 | 0.8 |

## 坐标系

```
map → odom（robot_localization EKF 发布）
odom → base_footprint（Gazebo）
base_footprint → base_link → base_scan（robot_state_publisher）
```
