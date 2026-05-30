# TurtleBot3 + Cartographer 踩坑记录

> 日期：2026-05-30
> 环境：ROS Noetic + Ubuntu 20.04 + Gazebo
> 机器人：TurtleBot3 Waffle Pi（Waffle）
> 局部规划器：DWA → TEB

---

## 一、运行环境搭建

### 1. Cartographer 编译安装

**问题：** 一开始用 `catkin_make_isolated --install` 产生了 `install_isolated/` 目录，用户不熟悉这种结构。

**原因：** Cartographer 核心库是 plain cmake 包，必须用 `catkin_make_isolated` 或单独 cmake 编译。官方推荐 `catkin_make_isolated --install --use-ninja`。

**解决：** 按官方 doc 操作：
```
wstool merge -t src cartographer_ros.rosinstall
wstool update -t src
rosdep install --from-paths src --ignore-src
src/cartographer/scripts/install_abseil.sh  # Ubuntu 20.04 无 libabsl-dev
catkin_make_isolated --install --use-ninja
```

### 2. 工作区 source 顺序

**问题：** 两个工作区（turtlebot3 + cartographer）先后 source 时，`install_isolated/setup.bash` 覆盖了 turtlebot3 的 `ROS_PACKAGE_PATH`。

**原因：** `install_isolated/setup.bash` 默认不 extend 已有工作区。

**解决：** 加 `--extend` 参数：
```
source ~/nav_project/catkin_cartographer/install_isolated/setup.bash --extend
```

**改动文件：** `~/.bashrc` — 新增第 135 行

---

## 二、启动流程修复

### 3. launch 文件缺失 robot_state_publisher

**问题：** 启动 `turtlebot3_cartographer.launch` 后，`base_scan` TF 帧不存在，Cartographer 报 `"base_scan" passed to lookupTransform argument source_frame does not exist`。

**原因：** `turtlebot3_cartographer.launch` 没有包含 `robot_state_publisher`。官方设计假设真机上由 `turtlebot3_bringup` 负责发布 TF，但 Gazebo 仿真不会自动启动它。

**解决：** 在 launch 文件中添加 `robot_state_publisher` 节点。

**改动文件：** `turtlebot3_slam/launch/turtlebot3_cartographer.launch`

### 4. launch 文件缺失 RViz

**问题：** Cartographer 正常工作但没有可视化窗口。

**解决：** 在 launch 文件末尾添加 rviz 节点，加载自带的 rviz 配置。

**改动文件：** `turtlebot3_slam/launch/turtlebot3_cartographer.launch`

### 5. Gazebo 仿真要用 gazebo 版 lua 配置

**问题：** 默认 `tracking_frame = "imu_link"`，但 Gazebo 的 IMU 插件有 bug，不发布 `imu_link` 的 TF。

**解决：** 启动时传入 `configuration_basename:=turtlebot3_lds_2d_gazebo.lua`（tracking_frame = base_footprint）。

---

## 三、DWA 局部规划器问题

### 6. 频繁重规划导致倒车

**问题：**
- `planner_frequency: 5.0` → 每秒重规划 5 次，地图不断更新，路径反复变
- `path_distance_bias: 32 > goal_distance_bias: 20` → 更在意贴路径而不是奔目标
- 新路径起点在车后 → 倒车追路径 → 线路越来越长

**解决过程：**
```
planner_frequency: 5.0 → 0.0 → 1.0 → 0.5
path_distance_bias: 32 → 24
goal_distance_bias: 20 → 24
```

**改动文件：** `turtlebot3_navigation/param/move_base_params.yaml`
`turtlebot3_navigation/param/dwa_local_planner_params_waffle_pi.yaml`

### 7. 速度调优

**问题：** 默认 `max_vel_x: 0.26 m/s` 太慢；提到 `0.40` 又太快，Cartographer scan matching 跟丢。

**最终值：** `max_vel_x: 0.30 m/s`

**改动文件：** `turtlebot3_navigation/param/dwa_local_planner_params_waffle_pi.yaml`

---

## 四、切换 TEB 局部规划器

### 8. 安装与配置

**安装：** `sudo apt install ros-noetic-teb-local-planner`

**改动文件：**
- 新建：`turtlebot3_navigation/param/teb_local_planner_params_waffle_pi.yaml`
- 新建：`turtlebot3_navigation/param/teb_local_planner_params_waffle.yaml`
- 修改：`turtlebot3_navigation/launch/move_base.launch` — 切换 base_local_planner

**切换方法：** 注释/取消注释 `move_base.launch` 中的 DWA / TEB 段落。

### 9. TEB 震荡不前进

**问题：** `possible oscillation detected`，车卡住不走。

**原因：**
- `weight_kinematics_forward_drive: 1.0` → 鼓励前进的权重太低
- `weight_optimaltime: 1.0` → 不在乎尽快到达

**解决：**
```
weight_kinematics_forward_drive: 1 → 1000
weight_optimaltime: 1 → 5
```

### 10. TF 时间戳外推错误

**错误信息：**
```
Extrapolation Error: Lookup would require extrapolation 0.003s into the future
Could not transform the global plan to the frame of the controller
```

**原因：** Cartographer 发布的 `map → odom` TF 比实际时间慢了几毫秒（仿真时钟同步问题）。

**解决：**
- `transform_tolerance: 0.5 → 1.0`（local 和 global costmap）
- launch 文件添加 `<param name="/use_sim_time" value="true"/>`

**改动文件：**
- `turtlebot3_navigation/param/local_costmap_params.yaml`
- `turtlebot3_navigation/param/global_costmap_params.yaml`
- `turtlebot3_slam/launch/turtlebot3_cartographer.launch`

---

## 五、传感器与参数调优

### 11. 激光雷达频率

**问题：** 默认 5Hz，数据太少，与 200Hz 的 Cartographer 位姿发布不匹配。

**解决：** 5Hz → 30Hz（`.gazebo.xacro` 文件）

**改动文件：** `turtlebot3_description/urdf/turtlebot3_*_pi.gazebo.xacro` 等 6 个文件

### 12. Cartographer 位姿发布频率

**问题：** `pose_publish_period_sec = 5e-3`（200Hz），远超传感器更新率，TF 频繁微调导致局部地图抖动。

**尝试方案 1：** 降为 30e-3（33Hz）→ 匹配里程计频率，但用户要求改回 200Hz + 提高雷达频率。

**最终方案：** 雷达 30Hz + 位姿 200Hz，让更多数据支撑高频 TF 更新。

**改动文件：** `turtlebot3_slam/config/turtlebot3_lds_2d_gazebo.lua`

### 13. 膨胀半径

**问题：** `inflation_radius: 1.0` → 机器人绕大圈，窄走廊过不去。

**确定过程：** 1.0 → 0.4 → 0.2 → **0.3m**（按车宽 0.31m 计算，取一半 + 余量）

**改动文件：** `turtlebot3_navigation/param/costmap_common_params_waffle_pi.yaml`

---

## 六、最终工作流

### 终端 1 — Gazebo 仿真
```bash
export TURTLEBOT3_MODEL=waffle_pi
roslaunch turtlebot3_gazebo turtlebot3_world.launch
```

### 终端 2 — Cartographer SLAM（含导航 + RViz）
```bash
export TURTLEBOT3_MODEL=waffle_pi
roslaunch turtlebot3_slam turtlebot3_cartographer.launch \
  configuration_basename:=turtlebot3_lds_2d_gazebo.lua
```

### 终端 3 — 键盘控制（可选，建图用）
```bash
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
```

### 保存地图
```bash
rosservice call /write_state "{filename: '$HOME/mymap.pbstream'}"
rosrun cartographer_ros cartographer_pbstream_to_ros_map \
  -pbstream_filename $HOME/mymap.pbstream \
  -map_filestem $HOME/mymap
```

---

## 七、文件改动清单

| 文件 | 改动 |
|------|------|
| `~/.bashrc` | 新增 cartographer source（--extend） |
| `turtlebot3_slam/launch/turtlebot3_cartographer.launch` | 加 use_sim_time、robot_state_publisher、rviz |
| `turtlebot3_navigation/launch/move_base.launch` | DWA → TEB（DWA 保留注释） |
| `turtlebot3_navigation/param/move_base_params.yaml` | planner_frequency: 5 → 0.5 |
| `turtlebot3_navigation/param/dwa_local_planner_params_waffle_pi.yaml` | max_vel 0.26→0.30, biases 32/20→24/24 |
| `turtlebot3_navigation/param/teb_local_planner_params_waffle_pi.yaml` | 新建（TEB 参数） |
| `turtlebot3_navigation/param/teb_local_planner_params_waffle.yaml` | 新建 |
| `turtlebot3_navigation/param/costmap_common_params_waffle_pi.yaml` | inflation_radius: 1.0 → 0.3 |
| `turtlebot3_navigation/param/local_costmap_params.yaml` | transform_tolerance: 0.5 → 1.0 |
| `turtlebot3_navigation/param/global_costmap_params.yaml` | transform_tolerance: 0.5 → 1.0 |
| `turtlebot3_slam/config/turtlebot3_lds_2d_gazebo.lua` | pose_publish_period_sec: 30e-3 → 5e-3 |
| `turtlebot3_description/urdf/turtlebot3_waffle_pi.gazebo.xacro` | laser update_rate: 5 → 30 |
| `turtlebot3_description/urdf/turtlebot3_waffle.gazebo.xacro` | laser update_rate: 5 → 30 |
| `turtlebot3_description/urdf/turtlebot3_burger.gazebo.xacro` | laser update_rate: 5 → 30 |
