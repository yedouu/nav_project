# nav_project 命令参考手册

## 目录

- [1. 构建与编译](#1-构建与编译)
- [2. Docker Compose 模块化启动](#2-docker-compose-模块化启动)
- [3. Make 快捷命令](#3-make-快捷命令)
- [4. 容器内 ROS 命令](#4-容器内-ros-命令)
- [6. 诊断与调试](#6-诊断与调试)
- [7. 清理命令](#7-清理命令)
- [8. 环境变量](#8-环境变量)
- [9. 参考表](#9-参考表)

---

## 1. 构建与编译

### 1.1 构建 Docker 镜像

```bash
# 构建镜像 (首次部署，约 15-30 分钟)
make build

# 无缓存重新构建
make build-no-cache

# 等价于
DOCKER_BUILDKIT=1 docker build -t nav_project:latest .
```

### 1.2 编译 ROS 工作区

```bash
# 编译所有工作区 (TurtleBot3 + Cartographer + DQN)
make build-ws

# 单独编译
make build-turtlebot3      # TurtleBot3 + scan_to_map + scan_tools (j4)
make build-cartographer    # Cartographer SLAM (j2, 最吃内存)
```

---

## 2. Docker Compose 模块化启动

### 2.1 Profile 说明

| Profile | 包含服务 | 用途 |
|---------|---------|------|
| `gazebo` | `gazebo` | 仿真环境 |
| `slam` | `slam` | Cartographer 建图 |
| `nav` | `gazebo`, `navigation` | 已知地图导航 |
| `mapping` | `gazebo`, `slam` | 建图环境 |
| `teleop` | `teleop` | 键盘遥控 |
| `viz` / `rviz` | `rviz` | 可视化 |
| `all` | `gazebo`, `slam`, `navigation`, `teleop` | 一键全栈 |
| `shell` | `shell` | 交互式 Shell |

### 2.2 建图模式 (Gazebo + Cartographer + 遥控)

```bash
# 启动全部
docker compose --profile gazebo --profile slam --profile teleop up -d

# 分步启动
docker compose --profile gazebo up -d gazebo         # 先开仿真
docker compose --profile slam up -d slam             # 再开 SLAM
docker compose --profile teleop up -d teleop         # 最后遥控

# 保存地图
docker compose run --rm shell bash -c "
  rosservice call /write_state '{filename: \"\$HOME/mymap.pbstream\"}' &&
  rosrun cartographer_ros cartographer_pbstream_to_ros_map \
    -pbstream_filename \$HOME/mymap.pbstream \
    -map_filestem \$HOME/mymap
"
```

### 2.3 已知地图导航 (Gazebo + ICP 定位 + TEB 导航 + RViz)

```bash
# 使用默认地图 indoor3
docker compose --profile nav up -d

# 使用其他地图
MAP_FILE=/root/nav_project/xtdrone_maps/indoor4.yaml \
  docker compose --profile nav up -d

# 使用其他世界
WORLD_FILE=/root/nav_project/xtdrone_worlds/indoor4.world \
  MAP_FILE=/root/nav_project/xtdrone_maps/indoor4.yaml \
  docker compose --profile nav up -d
```

### 2.4 XTDrone + Cartographer 边建图边导航

```bash
# Gazebo + Cartographer SLAM + 导航 + 遥控 (全栈)
docker compose --profile all up -d
```

### 2.5 交互式 Shell

```bash
# 进入容器
docker compose run --rm shell bash

# 执行单条命令
docker compose run --rm shell roslaunch scan_to_map scan_to_map.launch
docker compose run --rm shell rostopic list
docker compose run --rm shell rosservice call /relocalization "{}"
```

### 2.6 停止与清理

```bash
# 停止指定 profile 的容器
docker compose --profile nav down
docker compose --profile all down

# 停止所有
```

---

## 3. Make 快捷命令

```bash
make help             # 显示帮助

# 构建
make build            # 构建镜像
make build-ws         # 编译所有工作区

# 运行
make run-shell        # 进入容器交互 Shell
make run-all          # 一键启动: Gazebo + Cartographer SLAM + 遥控

# 训练

# 训练 + 上传

# 清理
make stop             # 停止所有容器
make clean            # 删除容器 + 缓存
make clean-all        # 删除容器 + 镜像
```

---

## 4. 容器内 ROS 命令

### 4.1 世界与机器人

```bash
# 标准 TurtleBot3 世界
roslaunch turtlebot3_gazebo turtlebot3_world.launch
roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch
roslaunch turtlebot3_gazebo turtlebot3_house.launch

# XTDrone 室内世界 (indoor3-indoor7)
roslaunch /root/nav_project/catkin_turtlebot3/src/turtlebot3/turtlebot3_slam/launch/xtdrone_world.launch \
  world_name:=/root/nav_project/xtdrone_worlds/indoor3.world

# XTDrone + Cartographer
roslaunch /root/nav_project/catkin_turtlebot3/src/turtlebot3/turtlebot3_slam/launch/xtdrone_cartographer.launch \
  world_name:=/root/nav_project/xtdrone_worlds/indoor3.world
```

### 4.2 SLAM

```bash
# Cartographer
roslaunch turtlebot3_slam turtlebot3_cartographer.launch \
  configuration_basename:=turtlebot3_lds_2d_gazebo.lua

# Gmapping
roslaunch turtlebot3_slam turtlebot3_gmapping.launch

# Hector
roslaunch turtlebot3_slam turtlebot3_hector.launch

# Karto
roslaunch turtlebot3_slam turtlebot3_karto.launch

# 保存 Cartographer 地图
rosservice call /write_state "{filename: '$HOME/mymap.pbstream'}"
rosrun cartographer_ros cartographer_pbstream_to_ros_map \
  -pbstream_filename $HOME/mymap.pbstream -map_filestem $HOME/mymap

# 保存 Gmapping 地图
rosrun map_server map_saver -f ~/my_map
```

### 4.3 导航 (已知地图)

```bash
# ICP 定位 + TEB 导航 + RViz
roslaunch scan_to_map scan_to_map.launch \
  map_file:=/root/nav_project/xtdrone_maps/indoor3.yaml

# 仅 EKF 融合 (不含 Gazebo)
roslaunch scan_to_map robot_localization_icp.launch

# 标准 AMCL 导航
roslaunch turtlebot3_navigation turtlebot3_navigation.launch \
  map_file:=$HOME/mymap.yaml
```

### 4.4 重定位

```bash
# 手动触发 SAC-IA + ICP 全局重定位
rosservice call /relocalization "{}"

# 发布初始位姿 (RViz 中也可操作)
rostopic pub /initialpose geometry_msgs/PoseWithCovarianceStamped "header: ...
  pose: {position: {x: 0.0, y: 0.0}, orientation: {z: 0.0, w: 1.0}}"
```

### 4.5 遥控与控制

```bash
# 键盘遥控
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch

# 发布速度指令
rostopic pub /cmd_vel geometry_msgs/Twist "linear: {x: 0.2} angular: {z: 0.0}" -r 10
```

### 4.6 激光雷达畸变补偿

```bash
# C++ 版本 (推荐)
rosrun scan_tools scan_corrector_node

# Python 版本
python3 /root/nav_project/scan_corrector.py
```

### 4.7 话题监控

```bash
# 列出所有话题
rostopic list

# 查看话题内容
rostopic echo /location_match
rostopic echo /scan -n 3
rostopic echo /odom

# 查看话题频率
rostopic hz /scan

# 查看 TF 树
rosrun rqt_tf_tree rqt_tf_tree
rqt_graph
```

---

## 5. 诊断与调试

### 5.1 Python 诊断脚本

```bash
# 实时诊断监控 (检测机器人停顿、地图更新、传感器状态)
docker compose run --rm shell python3 /root/nav_project/diagnostic_monitor.py

# 检查地图 frontier 细胞
docker compose run --rm shell python3 /root/nav_project/check_map.py
```

### 5.2 Shell 诊断

```bash
# 快速快照诊断
./diagnostic_v2.sh

# 持续后台诊断 (每 10 秒)
./diagnostic_continuous.sh
```

### 5.3 查看容器日志

```bash
# 实时日志
docker logs -f nav-gazebo
docker logs -f nav-navigation

# 最近 20 行
docker logs --tail 20 nav-slam
```

### 5.4 进入运行中的容器

```bash
docker exec -it nav-gazebo bash
docker exec -it nav-navigation bash
```

### 5.5 端口占用排查

```bash
# 查看端口占用
ss -tlnp | grep -E "12345|12346|12347|11311|11312|11313"

# 杀掉占用
fuser -k 12345/tcp
```

### 5.6 Gazebo 进程清理

```bash
# 清理所有残留
pkill -9 -f gz 2>/dev/null
pkill -9 -f Xvfb 2>/dev/null
```

---

## 6. 清理命令

```bash
# Docker Compose 方式
docker compose --profile all down       # 停止全栈容器
docker compose --profile nav down       # 停止导航容器

# Make 方式
make stop                               # 停止所有容器
make clean                              # 停止 + 删除容器
make clean-all                          # 停止 + 删除容器 + 删除镜像

# 手动清理
docker rmi nav_project:latest 2>/dev/null
docker builder prune -f
docker volume rm nav_project_gazebo-cache 2>/dev/null
```

---

## 7. 环境变量

| 变量 | 默认值 | 说明 |
|------|--------|------|
| `IMAGE_NAME` | `nav_project` | Docker 镜像名 |
| `IMAGE_TAG` | `latest` | 镜像标签 |
| `DISPLAY` | `:0` | X11 显示 |
| `TURTLEBOT3_MODEL` | `waffle_pi` | TurtleBot3 型号 |
| `WORLD_FILE` | `indoor3.world` | Gazebo 世界文件 |
| `MAP_FILE` | `indoor3.yaml` | 导航地图文件 |
| `GAZEBO_MODEL_DATABASE_URI` | `""` | 设为空避免联网卡顿 |

---

## 8. 参考表

### 8.1 可用的地图

| 地图 | 文件 | 大小 |
|------|------|------|
| indoor3 | `xtdrone_maps/indoor3.yaml` | 256KB |
| indoor4 | `xtdrone_maps/indoor4.yaml` | 4.0MB |
| indoor5 | `xtdrone_maps/indoor5.yaml` | 1.0MB |
| mission1 | `xtdrone_maps/mission1.yaml` | 11KB |
| mission1_new | `xtdrone_maps/mission1_new.yaml` | 16KB |

### 8.2 可用的世界

| 世界 | 文件 | 联网依赖 |
|------|------|---------|
| indoor3 | `xtdrone_worlds/indoor3.world` | 无 |
| indoor4 | `xtdrone_worlds/indoor4.world` | 无 |
| indoor5 | `xtdrone_worlds/indoor5.world` | 无 |
| indoor6 | `xtdrone_worlds/indoor6.world` | 无 |
| indoor7 | `xtdrone_worlds/indoor7.world` | 无 |
| indoor1 | `xtdrone_worlds/indoor1.world` | 需要 |
| indoor2 | `xtdrone_worlds/indoor2.world` | 需要 |

### 8.4 常用 ROS 话题

| 话题 | 类型 | 说明 |
|------|------|------|
| `/scan` | LaserScan | 激光雷达数据 |
| `/scan_corrected` | LaserScan | 畸变校正后的扫描 |
| `/odom` | Odometry | 里程计 |
| `/location_match` | PoseWithCovarianceStamped | ICP 定位结果 |
| `/map` | OccupancyGrid | 地图 |
| `/cmd_vel` | Twist | 速度控制指令 |
| `/relocalization` | Trigger | 手动重定位服务 |
| `/initialpose` | PoseWithCovarianceStamped | 初始位姿 |
