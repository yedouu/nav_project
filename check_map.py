#!/usr/bin/env python3
"""检查 Cartographer 地图是否有 frontier 细胞"""
import rospy
from nav_msgs.msg import OccupancyGrid
import numpy as np

def cb_map(msg):
    w, h = msg.info.width, msg.info.height
    res = msg.info.resolution
    data = np.array(msg.data, dtype=np.int8).reshape(h, w)

    unknown = np.sum(data < 0)
    free = np.sum(data == 0)
    occ = np.sum(data > 50)

    # 找 frontier: free 细胞旁边有 unknown
    frontier = 0
    for y in range(1, h-1):
        for x in range(1, w-1):
            if data[y,x] == 0:
                # 检查4邻域是否有 unknown
                if (data[y-1,x] < 0 or data[y+1,x] < 0 or
                    data[y,x-1] < 0 or data[y,x+1] < 0):
                    frontier += 1

    print(f"地图 {w}x{h} @ {res:.2f}m/px")
    print(f"  unknown: {unknown} cells ({100*unknown/(w*h):.1f}%)")
    print(f"  free:    {free} cells ({100*free/(w*h):.1f}%)")
    print(f"  occ:     {occ} cells ({100*occ/(w*h):.1f}%)")
    print(f"  frontiers: {frontier} cells ({frontier*res*res:.2f} m²)")
    rospy.signal_shutdown("done")

rospy.init_node('check_map')
rospy.Subscriber('/map', OccupancyGrid, cb_map)
rospy.spin()
