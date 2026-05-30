# Cartographer 地图无 free 细胞

## 日期
2026-05-30

## 现象
`check_map.py` 显示 free 细胞为 0，explore_lite 找不到 frontier。

```
free: 0 cells (0.0%)
frontiers: 0 cells (0.00 m²)
```

## 原因
Cartographer 默认概率阈值太敏感（等效 0.5），被激光扫过的区域几乎全标为 occupied。

`msg_conversion.cc` 第 410 行的固定转换公式：
```cpp
value = RoundToInt((1. - color / 255.) * 100.);
```

## 解决
修改 `occupancy_grid_node_main.cc`，添加 `occupied_space_override` 参数（阈值 0.7），重编译：

```bash
catkin_make_isolated --install --use-ninja --pkg cartographer_ros
```

启动时传参：
```xml
args="-resolution 0.05 -occupied_space_override 0.7"
```

## 涉及文件
`catkin_cartographer/src/.../cartographer_ros/occupancy_grid_node_main.cc`
`turtlebot3_slam/launch/turtlebot3_cartographer.launch`
