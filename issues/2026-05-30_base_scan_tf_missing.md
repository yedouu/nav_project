# Cartographer 第一次启动报 base_scan TF 不存在

## 日期
2026-05-30

## 现象
第一次启动时 Cartographer 报：
```
"base_scan" passed to lookupTransform argument source_frame does not exist
```
没地图显示，第二次启动才正常。

## 原因
`turtlebot3_cartographer.launch` 中没有设置 `robot_description` 参数，`robot_state_publisher` 启动后无 URDF 可发，`base_footprint → base_scan` TF 不存在。第二次启动时参数被缓存了所以正常。

## 解决
在 launch 文件中加载 `robot_description`：
```xml
<param name="robot_description"
       command="$(find xacro)/xacro --inorder
                '$(find turtlebot3_description)/urdf/turtlebot3_$(arg model).urdf.xacro'"/>
```

## 涉及文件
`turtlebot3_slam/launch/turtlebot3_cartographer.launch`
