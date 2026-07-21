IMAGE_NAME ?= nav_project
IMAGE_TAG  ?= latest

.PHONY: help build build-no-cache build-ws run-shell run-all run-dqn stop clean clean-all

help: ## 显示帮助
	@echo "nav_project + DQN_ROS Docker 部署命令:"
	@echo ""
	@echo "  构建:"
	@echo "    make build          构建 Docker 镜像"
	@echo "    make build-no-cache 无缓存重新构建"
	@echo "    make build-ws       编译所有 ROS 工作区"
	@echo "    make build-dqn      编译 DQN_ROS 工作区"
	@echo ""
	@echo "  运行:"
	@echo "    make run-shell      进入容器交互式 Shell"
	@echo "    make run-all        一键启动 Gazebo + SLAM + 遥控"
	@echo "    make run-dqn        测试 DQN 训练 (3 episodes)"
	@echo "    make train-dqn      正式 DQN 训练 (500 episodes)"
	@echo ""
	@echo "  清理:"
	@echo "    make stop           停止所有容器"
	@echo "    make clean          删除容器 + 缓存"
	@echo "    make clean-all      删除容器 + 镜像"
	@echo ""

build: ## 构建镜像
	DOCKER_BUILDKIT=1 docker build -t $(IMAGE_NAME):$(IMAGE_TAG) .

build-no-cache: ## 无缓存构建
	DOCKER_BUILDKIT=1 docker build --no-cache -t $(IMAGE_NAME):$(IMAGE_TAG) .

# ---- 编译工作区 ----
build-ws: build-turtlebot3 build-cartographer build-dqn

build-turtlebot3: ## 编译 TurtleBot3 工作区
	docker run --rm --runtime=nvidia --network host \
		-v $(PWD):/root/nav_project \
		$(IMAGE_NAME):$(IMAGE_TAG) \
		bash -c "cd /root/nav_project/catkin_turtlebot3 && catkin_make -j$$(nproc)"

build-cartographer: ## 编译 Cartographer 工作区
	docker run --rm --runtime=nvidia --network host \
		-v $(PWD):/root/nav_project \
		$(IMAGE_NAME):$(IMAGE_TAG) \
		bash -c "cd /root/nav_project/catkin_cartographer && catkin_make_isolated --install --use-ninja -j$$(nproc)"

build-dqn: ## 编译 DQN_ROS 工作区
	docker run --rm --runtime=nvidia --network host \
		-v $(PWD):/root/nav_project \
		$(IMAGE_NAME):$(IMAGE_TAG) \
		bash -c "cd /root/nav_project/dqn_ros_ws && rm -rf build devel && source /opt/ros/noetic/setup.bash && catkin_make -j$$(nproc)"

# ---- 运行 ----
run-shell: ## 进入容器 Shell
	xhost +local:docker > /dev/null 2>&1 || true
	docker run -it --rm --runtime=nvidia --network host \
		-v $(PWD):/root/nav_project \
		-e DISPLAY=$${DISPLAY:-:0} \
		$(IMAGE_NAME):$(IMAGE_TAG) bash

run-all: ## 一键启动 (Gazebo + SLAM + 遥控)
	xhost +local:docker > /dev/null 2>&1 || true
	docker run -it --rm --runtime=nvidia --network host \
		-v $(PWD):/root/nav_project \
		-e DISPLAY=$${DISPLAY:-:0} \
		$(IMAGE_NAME):$(IMAGE_TAG) \
		bash -c "roslaunch turtlebot3_gazebo turtlebot3_world.launch & \
			sleep 12 && \
			roslaunch turtlebot3_slam turtlebot3_cartographer.launch configuration_basename:=turtlebot3_lds_2d_gazebo.lua & \
			sleep 5 && \
			roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch & wait"

run-dqn: ## 测试 DQN (3 episodes)
	docker run --rm --runtime=nvidia --network host \
		-v $(PWD):/root/nav_project \
		--entrypoint "" \
		$(IMAGE_NAME):$(IMAGE_TAG) \
		/ros_entrypoint.sh bash /root/nav_project/docker/run-dqn-test.sh

train-dqn: ## 正式 DQN 训练 (300 episodes)
	docker run --rm --runtime=nvidia --network host \
		-v $(PWD):/root/nav_project \
		--entrypoint "" \
		$(IMAGE_NAME):$(IMAGE_TAG) \
		/ros_entrypoint.sh bash /root/nav_project/docker/run-dqn-train.sh

train-dqn-v2: ## DQN V2 训练 (优化Reward+降维, 300 episodes, 端口12346)
	docker run --rm --runtime=nvidia --network host \
		-v $(PWD):/root/nav_project \
		--entrypoint "" \
		$(IMAGE_NAME):$(IMAGE_TAG) \
		/ros_entrypoint.sh bash /root/nav_project/docker/run-dqn-train-v2.sh

train-dqn-v3: ## DQN V3 训练 (Double DQN+轻量网络, 300 episodes, 端口12347)
	docker run --rm --runtime=nvidia --network host \
		-v $(PWD):/root/nav_project \
		--entrypoint "" \
		$(IMAGE_NAME):$(IMAGE_TAG) \
		/ros_entrypoint.sh bash /root/nav_project/docker/run-dqn-train-v3.sh

train-dqn-v2-upload: ## V2 训练 + 自动上传 (需设置 GITHUB_TOKEN)
	docker run --rm --runtime=nvidia --network host \
		-v $(PWD):/root/nav_project \
		-e UPLOAD_RESULTS=true \
		-e GITHUB_TOKEN=$${GITHUB_TOKEN} \
		--entrypoint "" \
		$(IMAGE_NAME):$(IMAGE_TAG) \
		/ros_entrypoint.sh bash /root/nav_project/docker/run-dqn-train-v2.sh

train-dqn-v3-upload: ## DQN V3 训练 + 自动上传 (需设置 GITHUB_TOKEN)
	docker run --rm --runtime=nvidia --network host \
		-v $(PWD):/root/nav_project \
		-e UPLOAD_RESULTS=true \
		-e GITHUB_TOKEN=$${GITHUB_TOKEN} \
		--entrypoint "" \
		$(IMAGE_NAME):$(IMAGE_TAG) \
		/ros_entrypoint.sh bash /root/nav_project/docker/run-dqn-train-v3.sh

# ---- 清理 ----
stop: ## 停止所有容器
	docker stop nav-all-in-one nav-dqn-train nav-shell 2>/dev/null || true

clean: stop ## 删除容器
	docker rm nav-all-in-one nav-dqn-train nav-shell 2>/dev/null || true

clean-all: clean ## 删除镜像
	docker rmi $(IMAGE_NAME):$(IMAGE_TAG) 2>/dev/null || true
	docker builder prune -f
