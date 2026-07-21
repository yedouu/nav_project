# ============================================================
# ROS Noetic + Cartographer + TurtleBot3 导航镜像
# nav_project: https://github.com/yedouu/nav_project
# ============================================================

FROM ros:noetic-robot

LABEL description="ROS Noetic + Cartographer SLAM + TurtleBot3 Navigation"

ENV DEBIAN_FRONTEND=noninteractive
ENV TERM=xterm
ENV TURTLEBOT3_MODEL=waffle_pi
ENV GAZEBO_MODEL_DATABASE_URI=""

# ============================================================
# Stage 1: 基础工具 + 编译依赖
# ============================================================
RUN apt-get update && apt-get install -y --no-install-recommends \
      sudo git wget curl vim htop \
      ca-certificates gnupg lsb-release \
      build-essential cmake ninja-build \
      python3-osrf-pycommon python3-catkin-tools \
      python3-vcstool python3-rosdep python3-pip \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# ============================================================
# Stage 2: 系统库 (Cartographer + PCL 编译依赖)
# ============================================================
RUN apt-get update && apt-get install -y --no-install-recommends \
      libgoogle-glog-dev libgflags-dev libatlas-base-dev \
      libeigen3-dev libceres-dev libprotobuf-dev protobuf-compiler \
      libpcl-dev libyaml-cpp-dev liblua5.2-dev libboost-all-dev \
      libcairo2-dev libgtest-dev libgmock-dev \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# ============================================================
# Stage 3: Cartographer_ROS 编译所需的 ROS 包 (必须在 cartographer_ros 之前)
# ============================================================
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-noetic-pcl-ros ros-noetic-pcl-conversions \
      ros-noetic-tf2-sensor-msgs ros-noetic-tf2-eigen \
      ros-noetic-nav-msgs ros-noetic-urdf ros-noetic-xacro \
      ros-noetic-robot-state-publisher \
      ros-noetic-rviz \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# ============================================================
# Stage 4: 编译 Abseil (Cartographer 官方指定版本 20211102.0)
# ============================================================
WORKDIR /tmp
RUN git clone https://github.com/abseil/abseil-cpp.git \
    && cd abseil-cpp \
    && git checkout 215105818dfde3174fe799600bb0f3cae233d0bf \
    && mkdir build && cd build \
    && cmake -G Ninja \
             -DCMAKE_BUILD_TYPE=Release \
             -DCMAKE_POSITION_INDEPENDENT_CODE=ON \
             -DCMAKE_INSTALL_PREFIX=/usr/local .. \
    && ninja && ninja install && ldconfig \
    && cd /tmp && rm -rf abseil-cpp

# ============================================================
# Stage 5: 编译 Cartographer 核心库
# ============================================================
WORKDIR /tmp
RUN git clone --depth 1 --branch master \
      https://github.com/cartographer-project/cartographer.git \
    && cd cartographer && mkdir build && cd build \
    && cmake -G Ninja -DCMAKE_BUILD_TYPE=Release \
             -DCMAKE_POSITION_INDEPENDENT_CODE=ON \
             -DCMAKE_INSTALL_PREFIX=/usr/local .. \
    && ninja && ninja install && ldconfig \
    && cd /tmp && rm -rf cartographer

# ============================================================
# Stage 6: 编译 Cartographer ROS (依赖 Stage 3 的 ROS 包)
# ============================================================
WORKDIR /tmp
RUN git clone --depth 1 --branch master \
      https://github.com/cartographer-project/cartographer_ros.git \
    && mkdir -p /tmp/crosws/src && mv cartographer_ros /tmp/crosws/src/ \
    && cd /tmp/crosws \
    && /bin/bash -c "source /opt/ros/noetic/setup.bash \
        && catkin_make_isolated --install --use-ninja -DCMAKE_BUILD_TYPE=Release" \
    && mkdir -p /opt/cartographer_ros \
    && cp -r install_isolated/* /opt/cartographer_ros/ \
    && cd /tmp && rm -rf crosws

# ============================================================
# Stage 7: 导航核心
# ============================================================
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-noetic-navigation ros-noetic-move-base \
      ros-noetic-amcl ros-noetic-map-server \
      ros-noetic-global-planner ros-noetic-dwa-local-planner \
      ros-noetic-teb-local-planner ros-noetic-explore-lite \
      ros-noetic-robot-localization ros-noetic-gmapping \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# ============================================================
# Stage 8: TurtleBot3 + Gazebo 仿真 + 可视化
# ============================================================
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-noetic-turtlebot3 \
      ros-noetic-turtlebot3-gazebo \
      ros-noetic-turtlebot3-simulations \
      ros-noetic-turtlebot3-msgs \
      ros-noetic-turtlebot3-teleop \
      ros-noetic-gazebo-ros-pkgs \
      ros-noetic-gazebo-ros-control \
      ros-noetic-ros-control ros-noetic-ros-controllers \
      ros-noetic-rqt-common-plugins \
      ros-noetic-joint-state-publisher \
      # OpenGL/Mesa 库 (Gazebo headless 渲染必需) \
      libgl1-mesa-glx libgl1-mesa-dri mesa-utils \
      xvfb \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# ============================================================
# Stage 9: 项目环境入口
# ============================================================
RUN mkdir -p /root/nav_project

ENV GAZEBO_RESOURCE_PATH=/root/nav_project/xtdrone_worlds:/root/nav_project/xtdrone_maps

COPY docker/entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
WORKDIR /root/nav_project
