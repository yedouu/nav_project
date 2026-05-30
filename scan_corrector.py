#!/usr/bin/env python3
"""
激光雷达运动畸变补偿节点
原理：激光扫描过程中车在移动，每束激光发射时车体位姿不同。
      通过里程计插值算出每束激光发射时的车体位姿，将点云变换回扫描起始时刻。

订阅：
  /scan       — 原始激光雷达数据
  /odom       — 里程计（用于插值位姿）

发布：
  /scan_corrected — 去畸变后的激光雷达数据
"""

import rospy
import math
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
import tf2_ros
import tf2_geometry_msgs

class ScanDistortionCorrector:
    def __init__(self):
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.scan_pub = rospy.Publisher('/scan_corrected', LaserScan, queue_size=10)

        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(5.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.odom_buffer = []  # 缓存里程计 (time, x, y, yaw)
        self.max_buffer = 100

        rospy.Subscriber('/odom', Odometry, self.odom_callback)

        self.target_frame = rospy.get_param('~target_frame', 'map')
        self.source_frame = rospy.get_param('~source_frame', 'base_scan')

        rospy.loginfo('Scan distortion corrector started')
        rospy.loginfo(f'  target_frame: {self.target_frame}')
        rospy.loginfo(f'  source_frame: {self.source_frame}')

    def odom_callback(self, msg):
        """缓存里程计位姿"""
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        # 四元数转偏航角
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.odom_buffer.append((msg.header.stamp, x, y, yaw))
        if len(self.odom_buffer) > self.max_buffer:
            self.odom_buffer.pop(0)

    def interpolate_pose(self, stamp):
        """线性插值里程计位姿到指定时间戳"""
        if len(self.odom_buffer) < 2:
            return None
        # 找到最近的前后两个里程计数据
        before = None
        after = None
        for i, (ts, x, y, yaw) in enumerate(self.odom_buffer):
            if ts <= stamp:
                before = (ts, x, y, yaw)
            if ts >= stamp and after is None:
                after = (ts, x, y, yaw)
        if before is None or after is None:
            return None
        # 线性插值
        dt = (after[0] - before[0]).to_sec()
        if dt <= 0:
            return before[1], before[2], before[3]
        ratio = (stamp - before[0]).to_sec() / dt
        x = before[1] + (after[1] - before[1]) * ratio
        y = before[2] + (after[2] - before[2]) * ratio
        # 角度插值（处理跨 ±π 的情况）
        dyaw = after[3] - before[3]
        if dyaw > math.pi:
            dyaw -= 2 * math.pi
        elif dyaw < -math.pi:
            dyaw += 2 * math.pi
        yaw = before[3] + dyaw * ratio
        return x, y, yaw

    def scan_callback(self, scan):
        """去畸变主处理"""
        if len(scan.ranges) == 0:
            return

        # 获取扫描起止时间
        time_increment = scan.time_increment
        if time_increment == 0:
            # 有些雷达不设 time_increment，用 scan_time 和 beam 数估算
            time_increment = scan.scan_time / len(scan.ranges) if scan.scan_time > 0 else 1e-4

        scan_start = scan.header.stamp
        ranges = list(scan.ranges)
        intensities = list(scan.intensities) if scan.intensities else None

        angle_min = scan.angle_min
        angle_increment = scan.angle_increment
        corrected_ranges = list(ranges)  # 深拷贝

        # 获取扫描起始时刻的 base_scan → target_frame 变换
        try:
            tf_start = self.tf_buffer.lookup_transform(
                self.target_frame, self.source_frame, scan_start,
                rospy.Duration(0.1))
        except Exception as e:
            rospy.logwarn_throttle(2.0, f'无法获取起始TF: {e}')
            self.scan_pub.publish(scan)
            return

        # 对每束激光计算畸变补偿
        corrected_count = 0
        for i in range(len(ranges)):
            r = ranges[i]
            if r < scan.range_min or r > scan.range_max:
                continue  # 无效点跳过

            # 这束激光的时间戳
            beam_time = rospy.Time.from_sec(
                scan_start.to_sec() + i * time_increment)

            # 插值当前时刻的里程计位姿
            pose_at_beam = self.interpolate_pose(beam_time)
            if pose_at_beam is None:
                continue

            # 获取 beam 时刻的 base_scan → target_frame 变换
            try:
                tf_beam = self.tf_buffer.lookup_transform(
                    self.target_frame, self.source_frame, beam_time,
                    rospy.Duration(0.05))
            except Exception:
                continue  # TF 不可用，跳过这束

            # 这束激光在 base_scan 坐标系下的位置
            angle = angle_min + i * angle_increment
            local_x = r * math.cos(angle)
            local_y = r * math.sin(angle)

            # 用 beam 时刻的变换转到 target_frame
            p_in = tf2_geometry_msgs.PointStamped()
            p_in.header.frame_id = self.source_frame
            p_in.header.stamp = beam_time
            p_in.point.x = local_x
            p_in.point.y = local_y
            p_in.point.z = 0.0

            try:
                p_world = tf2_geometry_msgs.do_transform_point(p_in, tf_beam)
            except Exception:
                continue

            # 再转回扫描起始时刻的 base_scan 坐标系
            p_back = tf2_geometry_msgs.do_transform_point(p_world, tf_start)
            corrected_r = math.hypot(p_back.point.x, p_back.point.y)

            if corrected_r >= scan.range_min and corrected_r <= scan.range_max:
                corrected_ranges[i] = corrected_r
                corrected_count += 1

        if corrected_count > 0:
            rospy.logdebug(f'去畸变: {corrected_count}/{len(ranges)} 束')

        # 发布校正后的 scan
        corrected_scan = LaserScan()
        corrected_scan.header = scan.header
        corrected_scan.header.frame_id = self.source_frame
        corrected_scan.angle_min = angle_min
        corrected_scan.angle_max = scan.angle_max
        corrected_scan.angle_increment = angle_increment
        corrected_scan.time_increment = time_increment
        corrected_scan.scan_time = scan.scan_time
        corrected_scan.range_min = scan.range_min
        corrected_scan.range_max = scan.range_max
        corrected_scan.ranges = corrected_ranges
        if intensities:
            corrected_scan.intensities = intensities
        self.scan_pub.publish(corrected_scan)


if __name__ == '__main__':
    rospy.init_node('scan_distortion_corrector')
    ScanDistortionCorrector()
    rospy.spin()
