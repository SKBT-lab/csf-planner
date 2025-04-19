#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <fstream>
#include <string>
#include <iomanip> // 用于 std::fixed 和 std::setprecision
#include "odom_fusion/OdomStamp.h"


std::ofstream outFile;
ros::Publisher path_pub;

nav_msgs::Path path;
geometry_msgs::PoseStamped pose;


void odomCallback(const odom_fusion::OdomStamp::ConstPtr& msg)
{
    // 提取时间戳
    double timestamp = msg->pose.header.stamp.toSec();

    // 提取位置信息
    double x = pose.pose.position.x = msg->pose.pose.pose.position.x;
    double y = pose.pose.position.y = msg->pose.pose.pose.position.y;
    double z = pose.pose.position.z = msg->pose.pose.pose.position.z;

    
    // 提取速度信息
    double vx = msg->pose.twist.twist.linear.x;
    double vy = msg->pose.twist.twist.linear.y;
    double vz = msg->pose.twist.twist.linear.z;

    // 将数据写入文件，时间戳保留到小数点后六位，其他数据保留到小数点后三位
    outFile << std::fixed << std::setprecision(6) << timestamp << " "
            << std::setprecision(6) << x << " " << y << " " << z << " "
            << vx << " " << vy << " " << vz << std::endl;

    path.poses.push_back(pose);

    path.header.stamp = msg->pose.header.stamp;
    path_pub.publish(path);
}

int main(int argc, char** argv)
{
    // 初始化ROS节点
    ros::init(argc, argv, "odom_listener");
    ros::NodeHandle nh;

    path.header.frame_id = "world";
    pose.header.frame_id = "world";
    path_pub = nh.advertise<nav_msgs::Path>("/realtime_trajectory", 10);
    // 打开文件
    outFile.open("/home/skbt2/SKBT_Drone/src/planner/Utils/csf_utils/traj_record/output/trj_data.txt");
    if (!outFile.is_open()) {
        ROS_ERROR("Failed to open file!");
        return -1;
    }

    // 订阅odometry话题
    ros::Subscriber sub = nh.subscribe<odom_fusion::OdomStamp>("/fusion/odom", 1000, odomCallback);

    // 进入ROS事件循环
    ros::spin();

    // 关闭文件
    outFile.close();

    return 0;
}