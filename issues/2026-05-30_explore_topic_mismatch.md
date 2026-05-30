# explore_lite 订阅不到 /map

## 日期
2026-05-30

## 现象
explore_lite 启动后一直打印 `Waiting for costmap to become available, topic: map`，永远收不到地图。

## 原因
`costmap_topic: "map"`（相对路径）在 explore 的私有命名空间下被解析为 `/explore/map`，但 Cartographer 发布的是全局 `/map`。

## 解决
参数改为 `/map`（加前导斜杠）：
```xml
<param name="costmap_topic" value="/map"/>
<param name="costmap_updates_topic" value="/map_updates"/>
```

## 涉及文件
`turtlebot3_slam/launch/explore.launch`
