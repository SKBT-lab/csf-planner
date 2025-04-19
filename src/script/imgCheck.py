#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

def image_callback(msg):
    try:
        # 使用cv_bridge将ROS图像消息转换为OpenCV图像
        bridge = CvBridge()
        # 深度图通常是单通道的，使用 "passthrough" 或 "32FC1" 格式
        cv_image = bridge.imgmsg_to_cv2(msg, "passthrough")
        
        # 获取图像的长宽尺寸
        height, width = cv_image.shape
        rospy.loginfo("Depth image dimensions: {}x{}".format(width, height))
        
        # 打印深度图的一些信息（可选）
        rospy.loginfo("Depth range: min = {}, max = {}".format(cv_image.min(), cv_image.max()))
        
    except Exception as e:
        rospy.logerr("Error processing depth image: %s", str(e))

def main():
    # 初始化ROS节点
    rospy.init_node('depth_image_subscriber', anonymous=True)
    
    # 订阅深度图话题
    rospy.Subscriber("/xv_sdk/xv_dev/tof_camera/image", Image, image_callback)
    
    # 保持节点运行
    rospy.spin()

if __name__ == '__main__':
    main()