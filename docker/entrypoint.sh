#!/bin/bash
# ============================================================
# nav_project Docker 统一入口脚本
# 自动 source ROS + Cartographer + 项目工作区
# ============================================================

# 1. ROS Noetic
if [ -f /opt/ros/noetic/setup.bash ]; then
  source /opt/ros/noetic/setup.bash
  echo "[entrypoint] sourced ROS Noetic"
fi

# 2. Cartographer ROS (源码编译版)
if [ -f /opt/cartographer_ros/setup.bash ]; then
  source /opt/cartographer_ros/setup.bash --extend
  echo "[entrypoint] sourced Cartographer ROS"
fi

# 3. 项目工作区: TurtleBot3 (导航 + scan_to_map)
if [ -f /root/nav_project/catkin_turtlebot3/devel/setup.bash ]; then
  source /root/nav_project/catkin_turtlebot3/devel/setup.bash --extend
  echo "[entrypoint] sourced TurtleBot3 workspace"
fi

# 4. 项目工作区: Cartographer (修改版)
if [ -f /root/nav_project/catkin_cartographer/install_isolated/setup.bash ]; then
  source /root/nav_project/catkin_cartographer/install_isolated/setup.bash --extend
  echo "[entrypoint] sourced Cartographer project workspace"
fi

export TURTLEBOT3_MODEL=${TURTLEBOT3_MODEL:-waffle_pi}
export GAZEBO_MODEL_DATABASE_URI=${GAZEBO_MODEL_DATABASE_URI:-""}

# Gazebo 模型路径: TurtleBot3 + nav_project indoor 世界
TURTLEBOT3_MODELS=/root/nav_project/catkin_turtlebot3/src/turtlebot3/turtlebot3_simulations/turtlebot3_gazebo/models
export GAZEBO_MODEL_PATH=${TURTLEBOT3_MODELS}:${GAZEBO_MODEL_PATH:-/usr/share/gazebo-11/models}
export GAZEBO_RESOURCE_PATH=/root/nav_project/xtdrone_worlds:/root/nav_project/xtdrone_maps:${GAZEBO_RESOURCE_PATH}

# Gazebo 端口 (避免与宿主机冲突)
export GAZEBO_MASTER_URI=${GAZEBO_MASTER_URI:-http://localhost:12345}

# 加载 Gazebo 环境 (shader 支持)
if [ -f /usr/share/gazebo/setup.bash ]; then
  source /usr/share/gazebo/setup.bash
fi

echo "[entrypoint] TURTLEBOT3_MODEL=$TURTLEBOT3_MODEL GAZEBO_MASTER_PORT=12345"
echo "[entrypoint] Ready: $*"

exec "$@"
