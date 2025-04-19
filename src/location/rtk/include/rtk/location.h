#ifndef __LOCATION_H
#define __LOCATION_H

#include <iostream>
#include <std_msgs/String.h>
#include <ros/ros.h>

#include <thread>
#include <serial/serial.h>
#include <mavros_msgs/Altitude.h>
#include <sensor_msgs/Imu.h>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/TwistStamped.h>

#include <vector>

#include <cmath>

#include <Eigen/Geometry>


// typdef struct RTK_DATA{
//     double float Latitude;
//     double float Longitude;
//     double float Altitude;
// };


typedef struct{
    double pitch;
    double yaw;
    double roll;
}Euler;



class LOCATION{
private:

    int flag = 0;

    Euler euler;

    ros::Timer processData_timer_;

    nav_msgs::Odometry          odom_msg_;


    geometry_msgs::Point world_point;

    //用于发布里程计信息
    ros::Publisher odom_pub;

    //订阅
    ros::Subscriber baro_alti_sub, PX4_odom_data_sub;

    //RTK读取到的经纬高信息
    double lat = 40;
    double lon = 40;
    double alt = 40;

    //激光测距得到的距离值
    double laser_altitude = 0;

    /*
    **RTK模块的串口参数设置
    */
    void read_rtk_via_uart();


    /*
    **用于解析RTK串口读回的gpgga数据流
    */
    void parse_gpgga(const std::string& gpgga_data);



    /*
    **激光测距（TFmini Plus）模块的串口参数设置及数据解析
    */
    void read_laser_via_uart();

    /*
    **完成所有传感器数据的融合，并发布odometry
    */
    void processDataCallback(const ros::TimerEvent &e);
    

    void processBaroCallback(const mavros_msgs::Altitude::ConstPtr &msg);

    /*
    **获取来自飞控的里程计信息，但是需要补偿位置信息
    */
    void processPX4Odometry(const nav_msgs::Odometry::ConstPtr &msg);


    /*
    **四元数转欧拉角
    */
    void Quaternion2Euler(const geometry_msgs::Quaternion &q, Euler& euler);


    /*
    **从RTK读取到的WGS84转换到世界坐标系，函数输入是经纬高，输出是世界坐标系的（x,y,z）
    */
    void WGS84_2_world();



public:

    

    //构造函数可以传递参数进来
    LOCATION(/* argvs */){

    }
    ~LOCATION(){

    }

    void init(ros::NodeHandle& nh);
};

#endif