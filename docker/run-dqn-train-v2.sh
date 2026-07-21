#!/bin/bash
# DQN_ROS V2 强化学习训练 (改进 Reward + 降维观测)
# 使用独立端口 12346，不与旧训练冲突
set -e

# 清理端口 12346 上的残留
fuser -k 12346/tcp 2>/dev/null || true
sleep 1

# 虚拟显示器 (独立端口)
DISP=112
export DISPLAY=:$DISP
Xvfb :$DISP -screen 0 1280x1024x24 +extension GLX -noreset &
XVFB_PID=$!
sleep 1

# Python 依赖
pip3 install -q GitPython defusedxml 2>/dev/null || true

# --- V2 关键改动 ---
export HEADLESS_MODE=true
export MPLBACKEND=Agg
export GAZEBO_MASTER_URI=http://localhost:12346  # 独立端口，不冲突
export TURTLEBOT3_MODEL=waffle_pi
export GAZEBO_MODEL_DATABASE_URI=''

# roscore (source 必须在 export 之前，因为 setup.bash 会覆盖 ROS_MASTER_URI)
source /opt/ros/noetic/setup.bash
export ROS_MASTER_URI=http://localhost:11312
roscore -p 11312 &
sleep 3

# 加载 V2 配置
rosparam load /root/nav_project/dqn_ros/src/my_turtlebot3_openai_example/config/my_turtlebot3_openai_deepqlearn_params_v2.yaml
rosparam set /turtlebot3/ros_ws_abspath '/root/nav_project/dqn_ros_ws'

echo "========================================="
echo "  DQN V2 训练 (改进Reward + 12维观测)"
echo "  Gazebo端口: 12346  ROS端口: 11312"
echo "  开始: $(date)"
echo "========================================="
echo "  V1→V2 改动:"
echo "  - 观测: 120维 → 12维"
echo "  - 撞墙惩罚: -200 → -10"
echo "  - 距离渐进惩罚: 新增"
echo "  - 前进奖励: 5 → 2"
echo "  - episodes: 300"
echo "========================================="

cd /root/nav_project/dqn_ros_ws
source /root/nav_project/dqn_ros_ws/devel/setup.bash --extend

python3 /root/nav_project/dqn_ros/src/my_turtlebot3_openai_example/scripts/start_deepqlearning.py 2>&1 | \
    grep -E "EP:|SAVING|Reward|Error|error" --line-buffered

RET=$?
echo ""
echo "========================================="
echo " V2 训练结束 (exit=$RET)"
echo " 结束: $(date)"
echo "========================================="

# 自动上传 (由 UPLOAD_RESULTS 和 GITHUB_TOKEN 控制)
bash /root/nav_project/docker/upload-results.sh V2

kill $XVFB_PID 2>/dev/null
exit $RET
