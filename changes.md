## 二、文件改动记录

### 1. `turtlebot3_slam/launch/turtlebot3_cartographer.launch`

新增内容：
- **robot_state_publisher** — 发布机器人 TF 树（base_footprint → base_scan 等）
- **rviz** — 自动启动 RViz 并加载 cartographer 配置

完整文件见 `~/nav_project/catkin_turtlebot3/src/turtlebot3/turtlebot3_slam/launch/turtlebot3_cartographer.launch`

### 2. `~/.bashrc`

新增：
```bash
source ~/nav_project/catkin_cartographer/install_isolated/setup.bash --extend
```

### 3. `turtlebot3_navigation/param/move_base_params.yaml`

修改：
```yaml
planner_frequency: 5.0   →   planner_frequency: 0.0
```

**原因：** 建图过程中 Cartographer 持续更新 `/map`，5Hz 重规划会导致全局路径被反复重新计算。设为 0.0 后只在新目标或路径失效时才重规划。
