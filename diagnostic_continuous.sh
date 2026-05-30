#!/bin/bash
# 持续诊断脚本 — 从启动就开始记录，每10秒抓一次快照
LOG=~/nav_project/diagnostic_log2.txt
PIDFILE=/tmp/diagnostic_monitor.pid

# 防止重复启动
if [ -f $PIDFILE ] && kill -0 $(cat $PIDFILE) 2>/dev/null; then
    echo "诊断脚本已在运行 (PID $(cat $PIDFILE))"
    exit 1
fi

echo "$$" > $PIDFILE
echo "持续诊断启动 — $(date)" > $LOG
echo "========================================" >> $LOG
echo "每10秒抓取一次快照，直到按Ctrl+C停止" >> $LOG
echo "========================================" >> $LOG

cleanup() {
    echo "" >> $LOG
    echo "诊断停止 — $(date)" >> $LOG
    rm -f $PIDFILE
    exit 0
}
trap cleanup INT TERM

while true; do
    echo "--- $(date +%H:%M:%S) ---" >> $LOG

    # cmd_vel（最近一次）
    CMD=$(timeout 2 rostopic echo /cmd_vel -n 1 --noarr 2>/dev/null | grep -E "linear|angular" | head -4)
    if [ -n "$CMD" ]; then
        echo "cmd_vel: $CMD" >> $LOG
    else
        echo "cmd_vel: (无数据)" >> $LOG
    fi

    # move_base 状态
    STATUS=$(timeout 2 rostopic echo /move_base/status -n 1 --noarr 2>/dev/null | grep "status:" | head -1)
    if [ -n "$STATUS" ]; then
        echo "move_base 状态: $STATUS" >> $LOG
    else
        echo "move_base 状态: (无数据)" >> $LOG
    fi

    # 当前探索目标
    GOAL=$(timeout 2 rostopic echo /move_base/current_goal -n 1 --noarr 2>/dev/null | grep -E "position" | head -2)
    if [ -n "$GOAL" ]; then
        echo "当前目标: $GOAL" >> $LOG
    else
        echo "当前目标: (无目标)" >> $LOG
    fi

    # odom 位置
    ODOM=$(timeout 2 rostopic echo /odom -n 1 --noarr 2>/dev/null | grep -A3 "position" | head -4)
    if [ -n "$ODOM" ]; then
        echo "里程计: $ODOM" >> $LOG
    fi

    # map 更新情况
    MAP_INFO=$(timeout 2 rostopic echo /map -n 1 --noarr 2>/dev/null | grep -E "width|height|resolution" | head -3)
    if [ -n "$MAP_INFO" ]; then
        echo "地图: $MAP_INFO" >> $LOG
    else
        echo "地图: (未发布)" >> $LOG
    fi

    echo "" >> $LOG
    sleep 10
done
