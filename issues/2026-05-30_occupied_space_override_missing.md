# occupied_space_override 参数不存在

## 日期
2026-05-30

## 现象
传入 `-occupied_space_override 0.7` 后节点崩溃：
```
ERROR: unknown command line flag 'occupied_space_override'
```

## 原因
编译的 cartographer_ros 版本较旧，`occupancy_grid_node_main.cc` 中没有对应 `DEFINE_double`。

## 解决
手动修改源码添加参数，并在 `DrawAndPublish()` 中应用阈值后重新编译。

## 涉及文件
`catkin_cartographer/src/.../cartographer_ros/occupancy_grid_node_main.cc`
