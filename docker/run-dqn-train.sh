#!/bin/bash
# DQN_ROS 强化学习训练 (500 episodes)
# 用法: bash run-dqn-train.sh
set -e

# 清理残留
pkill -9 -f gz 2>/dev/null || true
pkill -9 Xvfb 2>/dev/null || true
sleep 2

# 虚拟显示器
DISP=111
export DISPLAY=:$DISP
Xvfb :$DISP -screen 0 1280x1024x24 +extension GLX -noreset &
XVFB_PID=$!
sleep 1

# Python 依赖
pip3 install -q GitPython defusedxml 2>/dev/null || true

# 环境
export HEADLESS_MODE=true
export MPLBACKEND=Agg
export GAZEBO_MASTER_URI=http://localhost:12345
export TURTLEBOT3_MODEL=waffle_pi
export GAZEBO_MODEL_DATABASE_URI=''

# 启动 roscore
source /opt/ros/noetic/setup.bash
roscore &
sleep 3

# 加载参数: 300 episodes (优化后的配置)
rosparam load /root/nav_project/dqn_ros/src/my_turtlebot3_openai_example/config/my_turtlebot3_openai_deepqlearn_params.yaml
rosparam set /turtlebot3/ros_ws_abspath '/root/nav_project/dqn_ros_ws'

echo "========================================="
echo "  DQN 正式训练 - 300 episodes (优化参数)"
echo "  开始时间: $(date)"
echo "========================================="

cd /root/nav_project/dqn_ros_ws
source /root/nav_project/dqn_ros_ws/devel/setup.bash --extend

python3 /root/nav_project/dqn_ros/src/my_turtlebot3_openai_example/scripts/start_deepqlearning.py 2>&1 | \
    grep -E "EP:|SAVING|Reward|Error|error" --line-buffered

RET=$?
echo ""
echo "========================================="
echo " 训练结束 (exit=$RET)"
echo " 结束时间: $(date)"
echo "========================================="

kill $XVFB_PID 2>/dev/null
exit $RET
