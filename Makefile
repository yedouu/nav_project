IMAGE_NAME ?= nav_project
IMAGE_TAG  ?= latest

.PHONY: help build build-no-cache build-ws build-turtlebot3 build-cartographer run-shell run-all stop clean clean-all

help: ## 显示帮助
	@echo "nav_project Docker 部署命令:"
	@echo ""
	@echo "  构建:"
	@echo "    make build          构建 Docker 镜像"
	@echo "    make build-no-cache 无缓存构建"
	@echo "    make build-ws       编译全部导航 ROS 工作区"
	@echo ""
	@echo "  运行:"
	@echo "    make run-shell      进入容器交互式 Shell"
	@echo "    make run-all        启动 Gazebo、SLAM 和遥控"
	@echo ""
	@echo "  清理:"
	@echo "    make stop           停止导航容器"
	@echo "    make clean          删除导航容器"
	@echo "    make clean-all      删除导航镜像"
	@echo ""

build: ## 构建镜像
	DOCKER_BUILDKIT=1 docker build -t $(IMAGE_NAME):$(IMAGE_TAG) .

build-no-cache: ## 无缓存构建
	DOCKER_BUILDKIT=1 docker build --no-cache -t $(IMAGE_NAME):$(IMAGE_TAG) .

# ---- 编译工作区 ----
build-ws: build-turtlebot3 build-cartographer ## 编译全部导航工作区

build-turtlebot3: ## 编译 TurtleBot3、定位和导航工作区
	docker run --rm --runtime=nvidia --network host \
		-v $(PWD):/root/nav_project \
		$(IMAGE_NAME):$(IMAGE_TAG) \
		bash -c "cd /root/nav_project/catkin_turtlebot3 && catkin_make -j4"

build-cartographer: ## 编译 Cartographer 工作区
	docker run --rm --runtime=nvidia --network host \
		-v $(PWD):/root/nav_project \
		$(IMAGE_NAME):$(IMAGE_TAG) \
		bash -c "cd /root/nav_project/catkin_cartographer && catkin_make_isolated --install --use-ninja -j2"

# ---- 运行 ----
run-shell: ## 进入容器 Shell
	xhost +local:docker > /dev/null 2>&1 || true
	docker run -it --rm --runtime=nvidia --network host \
		-v $(PWD):/root/nav_project \
		-e DISPLAY=$${DISPLAY:-:0} \
		$(IMAGE_NAME):$(IMAGE_TAG) bash

run-all: ## 启动 Gazebo、SLAM 和遥控
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

# ---- 清理 ----
stop: ## 停止导航容器
	docker stop nav-gazebo nav-slam nav-navigation nav-teleop nav-rviz nav-shell 2>/dev/null || true

clean: stop ## 删除导航容器
	docker rm nav-gazebo nav-slam nav-navigation nav-teleop nav-rviz nav-shell 2>/dev/null || true

clean-all: clean ## 删除导航镜像
	docker rmi $(IMAGE_NAME):$(IMAGE_TAG) 2>/dev/null || true
	docker builder prune -f
