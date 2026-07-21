#!/bin/bash
# ============================================================
# 首次部署引导脚本 (在宿主机执行)
# 作用: 引导用户完成首次 Docker 部署
# 用法: bash setup.sh
# ============================================================

set -e

echo "========================================="
echo " nav_project Docker 首次部署引导"
echo "========================================="
echo ""

# 1. 检查 Docker
if ! command -v docker &> /dev/null; then
  echo "[ERROR] 未检测到 Docker，请先安装 Docker:"
  echo "  https://docs.docker.com/engine/install/"
  exit 1
fi
echo "[OK] Docker $(docker --version)"

# 2. 检查 docker compose
if docker compose version &> /dev/null; then
  echo "[OK] Docker Compose $(docker compose version --short)"
elif command -v docker-compose &> /dev/null; then
  echo "[OK] docker-compose $(docker-compose --version | grep -oP '\d+\.\d+\.\d+')"
else
  echo "[WARN] 未找到 Docker Compose，多容器编排功能不可用"
fi

# 3. 检查 NVIDIA Docker (可选)
if command -v nvidia-smi &> /dev/null; then
  echo "[OK] NVIDIA GPU 已检测到，将使用 nvidia runtime"
  GPU_ENABLED=1
else
  echo "[INFO] 未检测到 NVIDIA GPU，将跳过 GPU 加速"
  GPU_ENABLED=0
fi

# 4. 检查 X11 (GUI 显示)
if [ -n "$DISPLAY" ]; then
  echo "[OK] DISPLAY=$DISPLAY"
else
  echo "[WARN] DISPLAY 未设置，GUI 应用 (Gazebo/RViz) 可能无法显示"
  echo "  请执行: export DISPLAY=:0"
fi

# 5. 构建镜像
echo ""
echo "========================================="
echo " 开始构建 Docker 镜像..."
echo " 预计需要 15-30 分钟 (取决于网络速度和机器性能)"
echo "========================================="
echo ""

read -p "是否立即构建镜像? [Y/n] " do_build
if [[ "$do_build" =~ ^[Nn] ]]; then
  echo "已跳过构建。稍后可执行: make build"
else
  DOCKER_BUILDKIT=1 docker build -t nav_project:latest .
  echo ""
  echo "[OK] 镜像构建完成!"
fi

# 6. 使用说明
echo ""
echo "========================================="
echo " 部署完成! 以下为常用命令:"
echo "========================================="
echo ""
echo "  # 进入容器 shell:"
echo "  make run-shell"
echo ""
echo "  # 一键启动 Gazebo + SLAM + 遥控:"
echo "  make run-all"
echo ""
echo "  # 分步启动:"
echo "  make run-gazebo     # 终端1: Gazebo 仿真"
echo "  make run-slam       # 终端2: Cartographer SLAM"
echo "  make run-teleop     # 终端3: 键盘控制"
echo ""
echo "  # 停止:"
echo "  make stop"
echo ""
echo "详情请查看: make help"
