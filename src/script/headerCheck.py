#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import PointCloud2

def callback(msg):
    print("Header:")
    print(msg.header)

rospy.init_node('pointcloud_header_viewer')
rospy.Subscriber('/xv_sdk/xv_dev/tof_camera/point_cloud', PointCloud2, callback)
rospy.spin()
