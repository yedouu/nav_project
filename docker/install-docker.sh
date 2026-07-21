#!/bin/bash
# ============================================================
# 一键安装 Docker + NVIDIA Container Toolkit
# 适用: Ubuntu 22.04
# 用法: bash install-docker.sh
# ============================================================
set -e

echo "========================================="
echo " Step 1/2: 安装 Docker CE"
echo "========================================="

curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /usr/share/keyrings/docker-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/docker-archive-keyring.gpg] https://download.docker.com/linux/ubuntu $(lsb_release -cs) stable" | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
sudo apt-get update
sudo apt-get install -y docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
sudo systemctl enable docker --now
sudo usermod -aG docker $USER
echo "[OK] Docker 安装完成"

echo ""
echo "========================================="
echo " Step 2/2: 安装 NVIDIA Container Toolkit"
echo "========================================="

curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
    sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
    sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list
sudo apt-get update
sudo apt-get install -y nvidia-container-toolkit
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker
echo "[OK] NVIDIA Container Toolkit 安装完成"

echo ""
echo "========================================="
echo " 验证安装"
echo "========================================="
docker --version
echo ""
echo "测试 GPU 访问:"
docker run --rm --runtime=nvidia --gpus all nvidia/cuda:12.0-base nvidia-smi 2>&1 || echo "[WARN] GPU 测试失败，如果你没有 NVIDIA GPU 这是正常的"
echo ""
echo "========================================="
echo " 安装完成!"
echo " 如果 docker 命令仍需要 sudo，请执行:"
echo "   newgrp docker"
echo " 或者重新登录。"
echo "========================================="
