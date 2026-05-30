# explore_lite 发不出探索目标

## 日期
2026-05-30

## 现象
地图已有 free 细胞，但 explore_lite 仍不发目标，机器人不动。

## 原因
`min_frontier_size` 设为 0.5 m²，但初始扫描区域小，frontier 只有 0.04 m²，全被过滤。

死锁：需要大 frontier → 需要移动 → 需要目标 → 需要大 frontier。

## 解决
`min_frontier_size` 从 0.5 降到 0.02：
```xml
<param name="min_frontier_size" value="0.02"/>
```

## 涉及文件
`turtlebot3_slam/launch/explore.launch`
