#!/bin/bash
# ============================================================
# 在容器内编译 ROS 工作区
# 用法: docker exec nav-all-in-one /root/nav_project/docker/build-workspace.sh
# ============================================================

set -e
source /opt/ros/noetic/setup.bash

echo "========================================="
echo " Building TurtleBot3 workspace"
echo "========================================="
cd /root/nav_project/catkin_turtlebot3
catkin_make -j$(nproc) 2>&1 | tail -20
echo "  -> TurtleBot3 done."

echo ""
echo "========================================="
echo " Building Cartographer workspace"
echo "========================================="
cd /root/nav_project/catkin_cartographer
catkin_make_isolated --install --use-ninja -j$(nproc) 2>&1 | tail -20
echo "  -> Cartographer done."

echo ""
echo "All workspaces built successfully!"
echo "Now re-source your shell or use:"
echo "  source /root/nav_project/catkin_turtlebot3/devel/setup.bash --extend"
echo "  source /root/nav_project/catkin_cartographer/install_isolated/setup.bash --extend"
