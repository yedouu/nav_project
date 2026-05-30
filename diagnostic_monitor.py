#!/usr/bin/env python3
"""
诊断脚本：监控 turtlebot3 + cartographer + explore_lite 的关键话题
输出到日志文件 ~/nav_project/diagnostic_log.txt
"""

import rospy
import os
from datetime import datetime
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from actionlib_msgs.msg import GoalStatusArray
from move_base_msgs.msg import MoveBaseActionGoal

log_file = os.path.expanduser("~/nav_project/diagnostic_log.txt")
log_fp = open(log_file, "w", buffering=1)

def log_msg(msg):
    print(msg)
    log_fp.write(msg + "\n")

def timestamp():
    return datetime.now().strftime('%H:%M:%S.%f')[:12]

log_msg("=" * 60)
log_msg(f"诊断监控启动 — {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
log_msg("=" * 60)

class Monitor:
    def __init__(self):
        self.last_cmd_vel = None
        self.cmd_vel_time = None
        self.last_goal = None
        self.last_goal_time = None
        self.last_map_time = None
        self.last_odom = None
        self.goal_status = None
        self.no_move_start = None
        self.last_scan_time = None

        rospy.Subscriber('/cmd_vel', Twist, self.cb_cmd_vel)
        rospy.Subscriber('/move_base/goal', MoveBaseActionGoal, self.cb_goal)
        rospy.Subscriber('/move_base/status', GoalStatusArray, self.cb_status)
        rospy.Subscriber('/map', OccupancyGrid, self.cb_map)
        rospy.Subscriber('/odom', Odometry, self.cb_odom)
        rospy.Subscriber('/scan', rospy.AnyMsg, self.cb_scan)

        rospy.Timer(rospy.Duration(2.0), self.timer_check)
        rospy.Timer(rospy.Duration(10.0), self.timer_summary)

        log_msg("监控已启动 — 每2秒检查一次，每10秒汇总一次")

    def cb_cmd_vel(self, msg):
        speed = (msg.linear.x**2 + msg.linear.y**2)**0.5
        turn = abs(msg.angular.z)
        now = rospy.Time.now()
        self.last_cmd_vel = (msg.linear.x, msg.angular.z, speed)
        self.cmd_vel_time = now
        if speed < 0.01 and turn < 0.01:
            if self.no_move_start is None:
                self.no_move_start = rospy.Time.now()
        else:
            self.no_move_start = None

    def cb_goal(self, msg):
        pos = msg.goal.target_pose.pose.position
        self.last_goal = (pos.x, pos.y)
        self.last_goal_time = rospy.Time.now()
        log_msg(f"[{timestamp()}] 新探索目标: ({pos.x:.2f}, {pos.y:.2f})")

    def cb_status(self, msg):
        if msg.status_list:
            s = msg.status_list[-1]
            texts = {1: "ACTIVE", 3: "SUCCEEDED", 4: "ABORTED", 5: "REJECTED"}
            if s.status in texts:
                self.goal_status = texts[s.status]
                if s.status != 1:
                    log_msg(f"[{timestamp()}] move_base 目标状态: {texts[s.status]}")

    def cb_map(self, msg):
        self.last_map_time = rospy.Time.now()

    def cb_odom(self, msg):
        self.last_odom = msg

    def cb_scan(self, msg):
        self.last_scan_time = rospy.Time.now()

    def timer_check(self, event):
        now = rospy.Time.now()
        t = timestamp()
        issues = []

        if self.no_move_start is not None:
            stalled = (now - self.no_move_start).to_sec()
            if stalled > 5.0:
                issues.append(f"⚠ 机器人 {stalled:.0f}秒未移动")
                if self.last_cmd_vel:
                    issues.append(f"   最后 cmd_vel: v={self.last_cmd_vel[0]:.2f} w={self.last_cmd_vel[1]:.2f}")
                if self.last_goal:
                    issues.append(f"   当前探索目标: ({self.last_goal[0]:.2f}, {self.last_goal[1]:.2f})")
                if self.goal_status:
                    issues.append(f"   move_base 状态: {self.goal_status}")

        if self.last_map_time and (now - self.last_map_time).to_sec() > 5.0:
            issues.append("⚠ 地图超过5秒未更新")

        if self.last_scan_time and (now - self.last_scan_time).to_sec() > 2.0:
            issues.append("⚠ 激光雷达超过2秒无数据")

        if self.goal_status in ("ABORTED", "REJECTED"):
            issues.append(f"⚠ move_base 目标被 {self.goal_status}")

        if issues:
            log_msg(f"[{t}] {' | '.join(issues)}")

    def timer_summary(self, event):
        t = timestamp()
        log_msg(f"\n[{t}] == 状态汇总 ==")
        if self.last_cmd_vel:
            log_msg(f"  cmd_vel: 线速度={self.last_cmd_vel[0]:.2f}, 角速度={self.last_cmd_vel[1]:.2f}")
        if self.last_goal:
            log_msg(f"  当前目标: ({self.last_goal[0]:.2f}, {self.last_goal[1]:.2f})")
        if self.goal_status:
            log_msg(f"  move_base: {self.goal_status}")
        if self.last_odom:
            p = self.last_odom.pose.pose.position
            log_msg(f"  里程计位置: ({p.x:.2f}, {p.y:.2f})")
        log_msg("")

if __name__ == '__main__':
    rospy.init_node('diagnostic_monitor', anonymous=True)
    m = Monitor()
    rospy.spin()
