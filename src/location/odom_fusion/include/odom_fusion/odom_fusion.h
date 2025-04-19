#ifndef _ODOM_FUSION_H_
#define _ODOM_FUSION_H_

#include <iostream>
#include <ros/ros.h>
#include <ros/console.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Imu.h>
#include <vector>
#include <thread>
#include <serial/serial.h>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <std_msgs/Float64.h>
#include <tf/tf.h>
#include "xv_sdk/PoseStampedConfidence.h"
#include "odom_fusion/OdomStamp.h"

using std::cout;
using std::endl;

class ODOM_FUSION{
private:
    odom_fusion::OdomStamp last_pose;
    bool first_pose = true;

    ros::Subscriber odom_sub;
    ros::Publisher odom_fusion_pub;
    ros::Timer TF_timer;

    odom_fusion::OdomStamp odom_withH;    //最终发布出去的定位信息
    bool flag;
    bool have_altitude;

    double tmp_altitude;

    serial::Serial serial_port;

    Eigen::Matrix3d rotation_matrix_CtoB;  //相机系相对于Body系的旋转矩阵
    Eigen::Quaterniond q_CtoB;             //相机系相对于Body系的四元数

    Eigen::Quaterniond Qc;                   //记录当前相机测量的原始姿态
    Eigen::Quaterniond Qb;                   //转换到body系下的姿态
  
    void OdomCallback(const xv_sdk::PoseStampedConfidence msg);
   
public:
    double altitude;
    void readLaser();


    ODOM_FUSION(){
        have_altitude = false;
        tmp_altitude = 0.0;
        altitude = 0.0;
        rotation_matrix_CtoB << 0, 0, 1,
                                -1, 0, 0,
                                0, -1, 0;
        Eigen::Quaterniond q(rotation_matrix_CtoB);
        q_CtoB = q;

        last_pose.pose.pose.pose.position.x = 0.0;
        last_pose.pose.pose.pose.position.y = 0.0;
        last_pose.pose.pose.pose.position.z = 0.0;

        last_pose.pose.pose.pose.orientation.x = 0.0;
        last_pose.pose.pose.pose.orientation.y = 0.0;
        last_pose.pose.pose.pose.orientation.z = 0.0;
        last_pose.pose.pose.pose.orientation.w = 1.0;

        odom_withH.pose.twist.twist.linear.x = 0.0;
        odom_withH.pose.twist.twist.linear.y = 0.0;
        odom_withH.pose.twist.twist.linear.z = 0.0;

        odom_withH.pose.twist.twist.angular.x = 0.0;
        odom_withH.pose.twist.twist.angular.y = 0.0;
        odom_withH.pose.twist.twist.angular.z = 0.0;


        
        while (true){
            try
            {
                // 打开串口设备
                serial_port.setPort("/dev/ttyUSB0");
                serial_port.setBaudrate(115200);
                serial_port.setBytesize(serial::eightbits);  // 设置数据位为8
                serial_port.setStopbits(serial::stopbits_one);  // 设置停止位为1
                serial_port.setParity(serial::parity_none);  // 设置奇偶校验位为None
                serial::Timeout timeout = serial::Timeout::simpleTimeout(100000);
                serial_port.setTimeout(timeout);
                serial_port.open();
                if (serial_port.isOpen())  // 检查串口是否已经成功打开
                {
                    ROS_INFO("Successfully opened the serial port.");
                    break;  // 成功打开串口，退出循环
                }
            }
            catch (serial::IOException& e)
            {
                ROS_ERROR_STREAM("Failed to open the serial port: " << e.what());
            }

            // 等待一段时间后重试
            ROS_INFO("Retrying to open the serial port in 1 seconds...");
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    }

    ~ODOM_FUSION(){}
    void init(ros::NodeHandle& nh);
};



#endif