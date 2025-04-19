#!/usr/bin/env python

import rospy
import numpy as np
import tf2_ros
import geometry_msgs.msg
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped, TransformStamped
from odom_fusion.msg import OdomStamp

# 全局变量存储当前里程计信息和轨迹数据
current_odom = None
current_path = None

# 获取阈值d、n和Display_dis
d = 5
n = 100
Display_dis = 5

def odom_callback(odom_msg):
    global current_odom
    current_odom = odom_msg

    # 发布 TF 变换
    publish_tf(odom_msg)

def path_callback(path_msg):
    global current_path
    current_path = path_msg

def publish_tf(odom_msg):
    # 创建 TF 广播器
    tf_broadcaster = tf2_ros.TransformBroadcaster()

    # 创建 TransformStamped 消息
    tf_msg = TransformStamped()

    # 设置 TF 消息的 header
    tf_msg.header.stamp = rospy.Time.now()
    tf_msg.header.frame_id = "world"  # 父坐标系
    tf_msg.child_frame_id = "body"   # 子坐标系

    # 设置 TF 消息的平移（位置）
    tf_msg.transform.translation.x = odom_msg.pose.pose.pose.position.x
    tf_msg.transform.translation.y = odom_msg.pose.pose.pose.position.y
    tf_msg.transform.translation.z = odom_msg.pose.pose.pose.position.z

    # 设置 TF 消息的旋转（姿态）
    tf_msg.transform.rotation = odom_msg.pose.pose.pose.orientation

    # 发布 TF 变换
    tf_broadcaster.sendTransform(tf_msg)

def timer_callback(event):
    global current_odom, current_path, d, n, Display_dis

    # 如果没有里程计信息或轨迹数据，直接返回
    if current_odom is None or current_path is None:
        rospy.logwarn("No odometry or path received yet!")
        return

    # 获取当前里程计的位置
    current_position = current_odom.pose.pose.pose.position

    # 如果没有轨迹点，直接返回
    if len(current_path.poses) == 0:
        rospy.logwarn("Received an empty path!")
        return

    # 找到轨迹上距离当前位置最近的点
    min_distance = float('inf')
    closest_index = 0

    for i, pose in enumerate(current_path.poses):
        # 计算当前位置与轨迹点之间的距离
        distance = np.sqrt((current_position.x - pose.pose.position.x) ** 2 +
                           (current_position.y - pose.pose.position.y) ** 2 +
                           (current_position.z - pose.pose.position.z) ** 2)

        # 更新最近点和索引
        if distance < min_distance:
            min_distance = distance
            closest_index = i

    # 获取轨迹的终点
    end_pose = current_path.poses[-1]

    # 计算当前位置与轨迹终点之间的距离
    end_distance = np.sqrt((current_position.x - end_pose.pose.position.x) ** 2 +
                           (current_position.y - end_pose.pose.position.y) ** 2 +
                           (current_position.z - end_pose.pose.position.z) ** 2)

    # 创建新的轨迹
    new_path = Path()
    new_path.header = current_path.header

    # 提取当前点之后的n个点
    extracted_poses = []
    for i in range(closest_index, min(closest_index + n, len(current_path.poses))):
        # 获取当前提取点
        extracted_pose = current_path.poses[i]

        # 计算提取点与当前点的距离
        extract_distance = np.sqrt((current_position.x - extracted_pose.pose.position.x) ** 2 +
                                   (current_position.y - extracted_pose.pose.position.y) ** 2 +
                                   (current_position.z - extracted_pose.pose.position.z) ** 2)

        # 如果提取点与当前点的距离大于Display_dis，则停止提取
        if extract_distance > Display_dis:
            break

        # 将提取点添加到新轨迹中
        extracted_poses.append(extracted_pose)

    # 将提取到的点添加到新轨迹中
    new_path.poses = extracted_poses

    # 发布新的轨迹
    new_path_pub.publish(new_path)

if __name__ == '__main__':
    # 初始化节点
    rospy.init_node('path_processor')

    # 订阅Odometry话题
    rospy.Subscriber('/fusion/odom', OdomStamp, odom_callback)

    # 订阅Path话题
    rospy.Subscriber('/console_node/trajectory', Path, path_callback)

    # 发布新的Path话题
    new_path_pub = rospy.Publisher('/console_node/trans_trajectory', Path, queue_size=10)

    # 设置定时器，周期为0.2秒
    rospy.Timer(rospy.Duration(0.2), timer_callback)

    # 进入ROS循环
    rospy.spin()