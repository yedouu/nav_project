#!/bin/bash
# 诊断脚本 v2 — 用 rostopic 命令行抓取关键话题
LOG=~/nav_project/diagnostic_log2.txt

echo "诊断开始 — $(date)" > $LOG
echo "==============================" >> $LOG

# 1. 检查话题列表
echo "--- 活跃话题 ---" >> $LOG
rostopic list -v 2>&1 | head -30 >> $LOG

# 2. 检查 cmd_vel (5秒窗口)
echo "--- cmd_vel (5秒) ---" >> $LOG
timeout 5 rostopic echo /cmd_vel -n 20 --noarr 2>&1 >> $LOG

# 3. 检查 map 是否发布
echo "--- map 信息 ---" >> $LOG
timeout 3 rostopic echo /map -n 1 --noarr 2>&1 | head -15 >> $LOG

# 4. 检查 move_base 目标状态
echo "--- move_base 状态 (5秒) ---" >> $LOG
timeout 5 rostopic echo /move_base/status -n 10 --noarr 2>&1 >> $LOG

# 5. 检查 move_base 是否有当前目标
echo "--- move_base 当前目标 ---" >> $LOG
timeout 3 rostopic echo /move_base/current_goal -n 1 --noarr 2>&1 | head -15 >> $LOG

# 6. 检查 scan 频率
echo "--- scan 频率 (5秒) ---" >> $LOG
timeout 5 rostopic hz /scan 2>&1 >> $LOG

# 7. 检查 Gazebo 是否发布了机器人位置
echo "--- odom 最后位置 ---" >> $LOG
timeout 3 rostopic echo /odom -n 1 --noarr 2>&1 | grep -A3 "position" >> $LOG

echo "==============================" >> $LOG
echo "诊断结束 — $(date)" >> $LOG
echo "日志已保存到 $LOG"
