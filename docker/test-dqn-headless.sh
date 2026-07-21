#!/bin/bash
# ============================================================
# DQN_ROS Headless 测试 - 快速验证强化学习训练流程
# ============================================================
set -e

echo "========================================="
echo " DQN_ROS 强化学习 Headless 测试"
echo "========================================="

# 1. 清理
echo "[Step 1] 清理残留进程..."
pkill -9 gzserver 2>/dev/null || true
pkill -9 gzclient 2>/dev/null || true
pkill -9 roscore 2>/dev/null || true
sleep 2

# 2. 安装额外 Python 依赖
echo "[Step 2] 安装 Python 依赖..."
pip3 install -q GitPython defusedxml 2>&1 | tail -1

# 3. 启动虚拟显示器
echo "[Step 3] 启动 Xvfb..."
export DISPLAY=:99
Xvfb :99 -screen 0 1024x768x24 +extension GLX +render &
sleep 1
echo "  DISPLAY=$DISPLAY"

# 4. Source 环境
echo "[Step 4] 加载 ROS 环境..."
source /opt/ros/noetic/setup.bash
source /opt/cartographer_ros/setup.bash --extend
source /root/nav_project/dqn_ros_ws/devel/setup.bash --extend

export TURTLEBOT3_MODEL=waffle_pi
export GAZEBO_MODEL_DATABASE_URI=""
export HEADLESS_MODE=true

# 5. 启动 roscore
echo "[Step 5] 启动 ROS Master..."
roscore &
sleep 3

# 6. 修改训练参数用于快速测试
echo "[Step 6] 配置训练参数 (小规模测试)..."
rosparam set /turtlebot3/ros_ws_abspath "/root/nav_project/dqn_ros_ws"
rosparam set /turtlebot3/task_and_robot_environment_name "TurtleBot3World-v0"
rosparam load /root/nav_project/dqn_ros/src/my_turtlebot3_openai_example/config/my_turtlebot3_openai_deepqlearn_params.yaml /turtlebot3/

# 仅测试 3 个 episode
rosparam set /turtlebot3/n_episodes 3

# 7. 运行训练
echo ""
echo "========================================="
echo " [Step 7] 开始 DQN 训练 (3 episodes)"
echo "========================================="
export MPLBACKEND=Agg
cd /root/nav_project/dqn_ros_ws
python3 /root/nav_project/dqn_ros/src/my_turtlebot3_openai_example/scripts/start_deepqlearning.py 2>&1 | head -200

echo ""
echo "========================================="
echo " 测试完成!"
echo "========================================="
