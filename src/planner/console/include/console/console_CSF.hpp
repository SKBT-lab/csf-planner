#ifndef _CONSOLE_HPP_
#define _CONSOLE_HPP_

#include <tf/tf.h>
#include <stdio.h>
#include <string>
#include <thread>
#include <iostream>
#include <fstream>
#include <cmath>
#include <random>
#include <limits>
#include <chrono>
#include <ros/ros.h>
#include <ros/console.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TwistStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

#include <quadrotor_msgs/PolynomialTrajectory.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <quadrotor_msgs/TakeoffLand.h>

#include <sensor_msgs/Joy.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <algorithm>
#include <Eigen/Eigen>
#include <Eigen/Geometry>

#include <vector>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>

#include <visual_utils/planning_visualization.hpp>
#include <tf/transform_broadcaster.h>

#include <apriltag_ros/AprilTagDetectionArray.h>
#include <apriltag_ros/AprilTagDetection.h>

//自定义的控制台状态消息
#include <console/ConsoleState.h>

#include "CSF.h"
#include "odom_fusion/OdomStamp.h"

#include <mutex>
#include <thread>

#include <csf/Cloth.h>
#include <csf/Trajectory.h>
#include <csf/Waypoints.h>

using namespace std;
using namespace Eigen;




enum Main_STATE{             
    LEVEL1 = 0,             //第n关
    LEVEL2,
    LEVEL3,
    LEVEL4,
    LEVEL5,
    LANDING,                 //降落
    INIT,                   //初始状态
    WAIT_TAKE_OFF_FINISHED //等待起飞完成
};

enum Sub_STATE{             
    Stage_1 = 0,
    Stage_2,
    Stage_3,
    Stage_4,
    Stage_5,
    Stage_6,
    Stage_0
};



class CONSOLE{
private:

    std::mutex csf_mutex;

    C_S_FILTER cs_f;
    csf::Trajectory traj_msg;
    //当前跟踪的轨迹
    MiniSnapTraj Cur_Traj;
    MiniSnapTraj Cur_Traj_cp;

    cv_bridge::CvImagePtr cv_ptr;

    cv::Mat Depth_image;
    cv::Mat Depth_image_cp;

    sensor_msgs::PointCloud2 Cloud_msg;
    
    double TF_height;
    
    bool rviz_delay; //是否按照实际的时间节奏去可视化轨迹
    int Hardwaypoints_num;

   

    int stagenum1, stagenum2, stagenum3, stagenum4, stagenum5;


    // 用于存储每一关是否被启用
    std::vector<bool> levelenable_list;

    // 每一关的阶段数
    std::vector<int> stagenum_list;

    // 每一关的每一阶段的规划结束标志和飞行结束标志
    
    std::vector<std::vector<bool>> planfinishflag_list;
    std::vector<std::vector<bool>> flyfinishflag_list;

    bool task_begin = false;

    double time_forward;
    
    //每一关的起点
    Eigen::Vector3d level1_start = Vector3d::Zero();
    Eigen::Vector3d level2_start = Vector3d::Zero();
    Eigen::Vector3d level3_start = Vector3d::Zero();
    Eigen::Vector3d level4_start = Vector3d::Zero();
    Eigen::Vector3d level5_start = Vector3d::Zero();
    std::vector<Eigen::Vector3d> start_list;

    //每一关的yaw角
    double yaw1, yaw2, yaw3, yaw4, yaw5;
    std::vector<double> yaw_list;

    double fly_altitude;
    
    ros::Timer ControlCmdCallback_Timer;
    ros::Timer Transformer_Timer;
    ros::Timer FSM_Timer;
    ros::Timer CSF_Timer;
    tf::TransformBroadcaster broadcaster;

    // 状态机当前状态
    Main_STATE main_state = INIT;
    Sub_STATE sub_state = Stage_0;

    //轨迹开始时间初始化标志
    bool Traj_time_init = false;
    //每一条轨迹的开始时间
    ros::Time tra_start_time;


    bool rotate_finished = true;
    double desired_yaw = 0.0;

    bool traj_rotate = false;

    bool replan_flag = false;

    //Apriltag相关
    Eigen::Matrix3d CtB_R; //相机系到机体系的姿态
    Eigen::Vector3d CtB_t;

    std::vector<Eigen::Vector3d> tag_to_center_list; //存储每个码相对于降落点中心的位置偏置
    bool apriltag_detected = false;
    std::vector<Eigen::Vector3d> CenterCalbyTag_list;

    Eigen::Vector3d landing_center;
    double last_yaw_;
    //时间放缩
    double tv1, tv2;
    double v_thresh, v_desire;
    double tv, dtv;
    bool zoom_enable;
    double zoom_factor;

    PlanningVisualization::Ptr trajVisual_;
    // nav_msgs::Path trajPath;
    ros::Publisher trajPath_pub; //用于轨迹可视化
    ros::Publisher controlCmd_pub;
    ros::Publisher vel_cmd_pub;
    ros::Publisher land_pub;
    ros::Publisher pose_pub;
    ros::Publisher traj_pub;

    ros::Publisher obj_pos_pub;
    nav_msgs::Odometry obj_odom;

    ros::Publisher Bpc_pub;
    ros::Publisher Hpc_pub;
    
    ros::Subscriber odom_sub;
    ros::Subscriber pointcloud_sub;
    ros::Subscriber apriltag_sub;
    ros::Subscriber depth_sub;

    ros::Subscriber csf_sub;
    ros::Subscriber wps_sub;

    Eigen::Quaterniond odom_q;
    Eigen::Quaterniond odom_q_cp;

    Eigen::Vector3d odom_pos, odom_vel;
    Eigen::Vector3d odom_pos_cp, odom_vel_cp;

    Eigen::Matrix3d x_PI_rotation;
  
    bool has_odom;
    bool has_depth;
    bool has_csf;

    quadrotor_msgs::PositionCommand posiCmd;

    
    double landing_height;
    bool is_land;

    bool traj_ready;

    pcl::PointCloud<pcl::PointXYZ>::Ptr barrier_cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr access_cloud;


    std::pair<double, double> calculate_yaw(const Traj_info& t_info);
    double clamp(double value, double min, double max);
    
    void ApriltagCallback(const apriltag_ros::AprilTagDetectionArrayPtr& msg);
    void ControlCmdCallback(const ros::TimerEvent &e);
    void OdomCallback(const odom_fusion::OdomStamp::ConstPtr& msg);
    void DepthCallback(const sensor_msgs::Image::ConstPtr& depth_msg);
    void CSFCallback(const csf::Cloth::ConstPtr& csf_msg);
    void wpsCallback(const csf::Waypoints::ConstPtr& wps_msg);

    /*TF树坐标变换关系*/
    void TransformerCallback(const ros::TimerEvent &e);
    /*控制状态机*/
    void ControlFSM(const ros::TimerEvent &e);
    /*根据轨迹解析控制指令*/
    bool ControlParse(MiniSnapTraj trajectory, ros::Time start_time, bool init, bool isAdjustYaw);

    void CSF_Update(const ros::TimerEvent &e);
    // void CSF_Update();

    void getVisual(MiniSnapTraj trajectory);


    void delay(int milliseconds);

    void nextLevel(Main_STATE& m);
    void nextStage(Sub_STATE& m);

    Eigen::MatrixXd vectorToMatrix(const std::vector<Eigen::Vector3d>& vec);
    Eigen::Vector3d start_forward(double vel);

    double Correction_z(double z);

    Eigen::Vector3d body_to_world(const Eigen::Vector3d& p_b);

    double getYawfromQuaternion(const Eigen::Quaterniond& q);
    void pub_cloud();
    void zoom_time(double t0);

    
public:
    
    
    CONSOLE(int width, int height): cs_f(width, height){
        TF_height = 0.0;
        has_odom = false;
        is_land = false;
        traj_ready = false;
        has_depth = false;
        has_csf = false;

        tra_start_time = ros::Time(0);

        odom_q.x() = 0.0;
        odom_q.y() = 0.0;
        odom_q.z() = 0.0;
        odom_q.w() = 1.0;

        odom_pos.x() = 0.0;
        odom_pos.y() = 0.0;
        odom_pos.z() = 0.0;

        CtB_R << 0, -1,  0,
                -1,  0,  0,
                 0,  0, -1;
        CtB_t << 0.067, 0, -0.16;
        barrier_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
        access_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
        Hardwaypoints_num = 0;
        x_PI_rotation = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX());  // 绕X轴旋转180度
        dtv = 0.0;
        zoom_enable = false;

    }
    ~CONSOLE(){

    }
    void init(ros::NodeHandle& nh);
};

#endif
