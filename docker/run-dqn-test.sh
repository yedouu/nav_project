#!/bin/bash
# DQN_ROS Headless 训练测试
set -e

# 1. 清理残留
pkill -9 -f gz 2>/dev/null || true
pkill -9 Xvfb 2>/dev/null || true
sleep 2

# 2. 虚拟显示器
DISP=111
export DISPLAY=:$DISP
Xvfb :$DISP -screen 0 1280x1024x24 +extension GLX -noreset &
XVFB_PID=$!
sleep 1

# 3. Python 依赖
pip3 install -q GitPython defusedxml 2>/dev/null || true

# 4. Headless + Gazebo 环境
export HEADLESS_MODE=true
export MPLBACKEND=Agg
export GAZEBO_MASTER_URI=http://localhost:12345
export TURTLEBOT3_MODEL=waffle_pi
export GAZEBO_MODEL_DATABASE_URI=''

# 5. 启动 roscore (ROSLauncher 会启动，但我们提前启动确保 rosparam 可用)
source /opt/ros/noetic/setup.bash
roscore &
sleep 3
echo "roscore started"

# 6. 加载训练参数 (YAML 顶层 key 是 turtlebot3:，直接 load 不进 namespace)
rosparam load /root/nav_project/dqn_ros/src/my_turtlebot3_openai_example/config/my_turtlebot3_openai_deepqlearn_params.yaml
rosparam set /turtlebot3/n_episodes 3
rosparam set /turtlebot3/ros_ws_abspath '/root/nav_project/dqn_ros_ws'
echo "params loaded:"
rosparam get /turtlebot3/n_episodes
rosparam get /turtlebot3/gamma

echo '========================================='
echo '  DQN 训练测试 (3 episodes)'
echo '========================================='

# 7. 运行训练
cd /root/nav_project/dqn_ros_ws
source /root/nav_project/dqn_ros_ws/devel/setup.bash --extend

python3 /root/nav_project/dqn_ros/src/my_turtlebot3_openai_example/scripts/start_deepqlearning.py 2>&1 &
TRAIN_PID=$!

# 8. 等待训练完成（最多5分钟）
sleep 300 &
WAIT_PID=$!
wait -n $TRAIN_PID $WAIT_PID 2>/dev/null
RET=$?

echo ""
echo "========================================="
echo " 训练结束 (exit=$RET)"
echo "========================================="

kill $XVFB_PID 2>/dev/null
pkill -9 -f gz 2>/dev/null
exit $RET
