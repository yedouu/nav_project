# Cartographer SLAM + 导航笔记

> TurtleBot3 Waffle Pi + ROS Noetic + Cartographer
> 工作区：`~/nav_project/catkin_turtlebot3/`
> Cartographer 工作区：`~/nav_project/catkin_cartographer/`

---

## 一、环境准备

`.bashrc` 中已配置以下内容（按 source 顺序）：

```bash
source /opt/ros/noetic/setup.bash           # ROS 本体
export TURTLEBOT3_MODEL=waffle              # 默认模型（终端可临时覆盖）
source ~/nav_project/catkin_turtlebot3/devel/setup.bash           # TurtleBot3 包
source ~/nav_project/catkin_cartographer/install_isolated/setup.bash --extend  # Cartographer（--extend 避免覆盖前面的路径）
```

如果使用 `waffle_pi`，在终端执行：
```bash
export TURTLEBOT3_MODEL=waffle_pi
```

---

## 二、文件改动记录

### 1. `turtlebot3_slam/launch/turtlebot3_cartographer.launch`

新增内容：
- **robot_state_publisher** — 发布机器人 TF 树（base_footprint → base_scan 等）
- **rviz** — 自动启动 RViz 并加载 cartographer 配置

完整文件见 `~/nav_project/catkin_turtlebot3/src/turtlebot3/turtlebot3_slam/launch/turtlebot3_cartographer.launch`

### 2. `~/.bashrc`

新增：
```bash
source ~/nav_project/catkin_cartographer/install_isolated/setup.bash --extend
```

---

## 三、模式 1：建图（纯 SLAM）

### 步骤

**终端 1 — 启动 Gazebo 仿真**
```bash
export TURTLEBOT3_MODEL=waffle_pi
roslaunch turtlebot3_gazebo turtlebot3_world.launch
```

**终端 2 — 启动 Cartographer SLAM**（自动启动 robot_state_publisher + RViz）
```bash
export TURTLEBOT3_MODEL=waffle_pi
roslaunch turtlebot3_slam turtlebot3_cartographer.launch \
  configuration_basename:=turtlebot3_lds_2d_gazebo.lua
```

**终端 3 — 键盘控制机器人建图**
```bash
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
```

在 RViz 中观察地图逐渐构建。

---

## 四、保存地图

Cartographer 使用自己的 `.pbstream` 格式保存，然后可转换为 ROS 标准地图格式（pgm + yaml）。

### 4.1 保存 pbstream

```bash
rosservice call /write_state "{filename: '$HOME/mymap.pbstream'}"
```

### 4.2 转换为 ROS 地图（pgm + yaml）

```bash
rosrun cartographer_ros cartographer_pbstream_to_ros_map \
  -pbstream_filename $HOME/mymap.pbstream \
  -map_filestem $HOME/mymap
```

生成文件：
- `$HOME/mymap.pgm` — 栅格地图图片
- `$HOME/mymap.yaml` — 地图配置文件

---

## 五、模式 2：已知地图导航（已有地图，仅导航）

使用之前保存好的地图（如 `~/mymap.yaml`）进行导航。

### 步骤

**终端 1 — 启动 Gazebo 仿真**（必须与建图时用同一个世界）
```bash
export TURTLEBOT3_MODEL=waffle_pi
roslaunch turtlebot3_gazebo turtlebot3_world.launch
```

**终端 2 — 启动机器人 TF 发布**
```bash
rosrun robot_state_publisher robot_state_publisher
```

**终端 3 — 启动导航 + 已知地图**
```bash
export TURTLEBOT3_MODEL=waffle_pi
roslaunch turtlebot3_navigation turtlebot3_navigation.launch \
  map_file:=$HOME/mymap.yaml
```

在 RViz 中点击 **2D Nav Goal**，在地图上点一个目标位置，机器人自动规划路径并导航。

### 导航栈说明

- **map_server** — 加载保存好的 `mymap.yaml`
- **AMCL** — 粒子滤波定位，判断机器人在已建地图中的位置
- **move_base** — 全局 + 局部路径规划（DWA 算法）
  - 全局路径：Dijkstra / A* 在已知地图上规划
  - 局部路径：动态窗口法避障

---

## 六、模式 3：边建图边导航（SLAM + Navigation）

Cartographer 本身就是 SLAM 系统，建图的同时也在做定位。`turtlebot3_cartographer.launch` 已经包含了 `move_base`，所以建图过程中就能导航。

### 步骤

**终端 1 — Gazebo 仿真**
```bash
export TURTLEBOT3_MODEL=waffle_pi
roslaunch turtlebot3_gazebo turtlebot3_world.launch
```

**终端 2 — 启动 Cartographer SLAM**
```bash
export TURTLEBOT3_MODEL=waffle_pi
roslaunch turtlebot3_slam turtlebot3_cartographer.launch \
  configuration_basename:=turtlebot3_lds_2d_gazebo.lua
```

**在 RViz 中：**
1. 观察地图正在构建
2. 点击 **2D Nav Goal** 设置目标点
3. 机器人会边建图边导航过去
4. Cartographer 的实时定位（局部 SLAM）+ 回环检测（全局优化）同时进行

### 原理

```
Gazebo                        Cartographer
  │                                │
  ├─ /scan（激光雷达）              ├─ 前端 scan-to-submap 匹配（实时定位）
  ├─ /odom（里程计）                ├─ 后端回环检测 + 图优化
  ├─ /tf（odom→base_footprint）     ├─ 发布 map→odom TF
  └─ gazebo 物理引擎                ├─ 发布 /map（栅格地图）
                                      │
                                  move_base
                                      │
                                 ┌────┴────┐
                                 │ 全局规划  │
                                 │ 局部规划  │
                                 └─────────┘
                                      │
                                   /cmd_vel → Gazebo
```

---

## 七、常见问题

### TF 帧缺失（base_scan / imu_link 不存在）

**原因：** `robot_state_publisher` 没有运行，或者用了错误的 tracking_frame。

**解决：**
- `robot_state_publisher` 已在 launch 文件中自动启动
- Gazebo 仿真时务必使用 `turtlebot3_lds_2d_gazebo.lua`（tracking_frame = base_footprint）

### 找不到 cartographer_ros 包

**原因：** 没有 source cartographer 工作区。

**解决：** 已在 `.bashrc` 中配置，新开终端即可。
```bash
source ~/nav_project/catkin_cartographer/install_isolated/setup.bash --extend
```

### launch 文件改动被覆盖

如果重新克隆或更新 turtlebot3 包，`turtlebot3_cartographer.launch` 的修改会丢失。届时重新添加：
- robot_state_publisher 节点
- rviz 节点
