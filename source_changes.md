# Cartographer 源码改动

## 改动文件

`catkin_cartographer/src/cartographer_ros/cartographer_ros/cartographer_ros/occupancy_grid_node_main.cc`

## 改动内容

### 1. 新增命令行参数（第 42-43 行）

```cpp
DEFINE_double(occupied_space_override, 0.5,
              "Probability at which to consider cells as occupied.");
```

### 2. DrawAndPublish 中应用阈值（第 173-179 行）

```cpp
const double occ_threshold = FLAGS_occupied_space_override;
for (auto& cell : msg_ptr->data) {
  if (cell >= 0) {
    cell = (cell / 100.0 > occ_threshold) ? 100 : 0;
  }
}
```

## 作用

Cartographer 默认占用阈值 0.5 太敏感，导致地图中 0% free 细胞，
explore_lite 找不到 frontier。改为 0.7 后 free 细胞恢复正常。
