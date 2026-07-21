#!/bin/bash
# DQN_ROS V3 训练 (Double DQN + 轻量网络 + 稳定参数)
# 独立端口 12347，不与 V1/V2 冲突
set -e

# 清理端口 12347 上的残留
fuser -k 12347/tcp 2>/dev/null || true
sleep 1

# 虚拟显示器 (独立端口)
DISP=113
export DISPLAY=:$DISP
Xvfb :$DISP -screen 0 1280x1024x24 +extension GLX -noreset &
XVFB_PID=$!
sleep 1

# Python 依赖
pip3 install -q GitPython defusedxml 2>/dev/null || true

# V3 环境: 独立端口
export HEADLESS_MODE=true
export MPLBACKEND=Agg
export GAZEBO_MASTER_URI=http://localhost:12347
export TURTLEBOT3_MODEL=waffle_pi
export GAZEBO_MODEL_DATABASE_URI=''

# ROS Master (source 必须在 export 之前)
source /opt/ros/noetic/setup.bash
export ROS_MASTER_URI=http://localhost:11313
roscore -p 11313 &
sleep 3

# 加载 V3 配置
rosparam load /root/nav_project/dqn_ros/src/my_turtlebot3_openai_example/config/my_turtlebot3_openai_deepqlearn_params_v3.yaml
rosparam set /turtlebot3/ros_ws_abspath '/root/nav_project/dqn_ros_ws'

echo "========================================="
echo "  DQN V3 训练 (Double DQN + 轻量网络)"
echo "  Gazebo:12347  ROS:11313"
echo "  开始: $(date)"
echo "========================================="
echo "  V2→V3 改进:"
echo "  - Double DQN: 消除Q值高估偏差"
echo "  - 轻量网络: 12→128→128→3 (原6层→3层)"
echo "  - epsilon_decay: 15000→8000 (更快收敛)"
echo "  - target_update: 50→100"
echo "  - batch_size: 64→128"
echo "========================================="

cd /root/nav_project/dqn_ros_ws
source /root/nav_project/dqn_ros_ws/devel/setup.bash --extend

python3 /root/nav_project/dqn_ros/src/my_turtlebot3_openai_example/scripts/start_deepqlearning.py 2>&1 | \
    grep -E "EP:|SAVING|Reward|Error|error" --line-buffered

RET=$?
echo ""
echo "========================================="
echo " V3 训练结束 (exit=$RET)  结束: $(date)"
echo "========================================="

# 自动上传 (由 UPLOAD_RESULTS 和 GITHUB_TOKEN 控制)
bash /root/nav_project/docker/upload-results.sh V3

kill $XVFB_PID 2>/dev/null
exit $RET
