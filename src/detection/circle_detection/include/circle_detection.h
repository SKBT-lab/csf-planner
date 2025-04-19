#ifndef __CIRCLE_DETECTION_H
#define __CIRCLE_DETECTION_H

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/Imu.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <vector>
#include <algorithm>
#include <queue>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <numeric>
#include <math.h>
#include <fstream>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/Imu.h>
#include <edcircles/EDCircles.h>
#include <float.h>
// #include "circle_detection_node/Circle_Center.h"


#include <edcircles/EDPF.h>
#include <edcircles/EDCircles.h>
#include <edcircles/EDColor.h>
#include <edcircles/EDLib.h>
#include <edcircles/EDLines.h>
#include <edcircles/EDLib.h>
#include <edcircles/ED.h>


using namespace std;
using namespace cv;
using namespace Eigen;


struct distance_i_j
{
    double distance;
    int i;
    int j;
    double radius;
};


class CIRCLE //circle
{
public:
    CIRCLE() {
        have_circle_ = false;
        have_Ellipse = false;
    }
    ~CIRCLE() {}
    
    void init(ros::NodeHandle& nh);
    typedef shared_ptr<CIRCLE> Ptr;
    std::pair<bool, cv::Point3d> DetectionCallback(const sensor_msgs::Image::ConstPtr& msg);
        
private:
    cv::Mat color_pic;
    cv::Mat img_gray;               
    

    int ring_width;

    bool  have_circle_ , have_Ellipse;   
    // circle_detection_node::Circle_Center point_msg;       
    cv::Point des_cen , decE_cen; 

    //圆属性           
    int des_r;
    cv::Size axes;
    double theta;

    //椭圆半长轴
    double decE_r;

    vector<distance_i_j> ED_lib_dis;
    vector<distance_i_j> ED_lib_disE;



    
    inline double distance_cv_points(const cv::Point& a , const cv::Point& b);
    inline static bool cv_point_cmp(const distance_i_j& a , const distance_i_j& b);
    inline static bool radius_cmp(const distance_i_j &a , const distance_i_j &b);

};


#endif