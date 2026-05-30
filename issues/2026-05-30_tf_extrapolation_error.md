# 导航中 TF 帧转换报错导致卡住

## 日期
2026-05-30

## 现象
探索中机器人突然停住，控制台反复输出：
```
Extrapolation Error: Lookup would require extrapolation 0.020s into the future.
Could not transform the global plan to the frame of the controller
Could not get local plan
```

## 原因
局部 costmap 用 `global_frame: odom`，全局路径在 `map` 帧，每次规划需 `map → odom` TF 转换。Cartographer 负载大时 TF 延迟从几 ms 涨到几十 ms，转换失败。

## 解决
局部 costmap `global_frame` 从 `odom` 改为 `map`，两条路径同帧，无需转换：
```yaml
local_costmap:
  global_frame: map
```

## 涉及文件
`turtlebot3_navigation/param/local_costmap_params.yaml`
