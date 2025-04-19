#ifndef APRILTAG_HPP
#define APRILTAG_HPP

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include "opencv2/opencv.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

class APRILTAG{
private:

    /***just for rosbag record***/
    image_transport::Publisher cam_pub; 
    sensor_msgs::ImagePtr image;
    ros::Timer imageTimer;
    void imageCallback(const ros::TimerEvent &e);
    /****************************/

    //相机流
    cv::VideoCapture cap;
    cv::Mat frame;

    //相机初始化
    void camInit();

public:
    APRILTAG(/* args */){

    }
    ~APRILTAG(){

    }
    void init(ros::NodeHandle &nh);

};



#endif 