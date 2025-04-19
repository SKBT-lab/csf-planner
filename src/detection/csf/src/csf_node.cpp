#include "ros/ros.h"
#include <ros/console.h>
#include "sensor_msgs/Image.h"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/highgui/highgui.hpp"
#include "CSF.h"
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Point.h>
#include <mutex>

MiniSnapTraj traj;


C_S_FILTER cs_f(320, 240);
Eigen::Quaterniond odom_q;
Eigen::Vector3d odom_pos;

ros::Subscriber odom_sub;
ros::Subscriber depth_sub;
ros::Subscriber traj_sub;

csf::Cloth csf_msg;
ros::Publisher csf_pub;

csf::Waypoints wps_msg;

ros::Publisher Bpc_pub;
ros::Publisher Hpc_pub;

ros::Publisher wps_pub;

pcl::PointCloud<pcl::PointXYZ>::Ptr barrier_cloud;
pcl::PointCloud<pcl::PointXYZ>::Ptr access_cloud;

bool has_traj;


void pub_cloud(){
  pcl::PointXYZ point;
  barrier_cloud->clear();
  access_cloud->clear();
  // 布料转化为点云，用于可视化

  // cv::Point2f cen(0.5 * csf.width * csf.Kspars, 0.5 * csf.height * csf.Kspars);
  // Eigen::Vector3d cen_pos = csf.Get_3Dpos(cen, csf.Depth_Cloth(int(0.5 * csf.height), int(0.5 * csf.width)), odom_q, odom_pos);
  // std::cout << "center: " <<  cen_pos << std::endl;

  for(int i = 0; i < cs_f.height; i++){
      for(int j = 0; j < cs_f.width; j++){
          cv::Point2f center(j * cs_f.Kspars, i * cs_f.Kspars);
          Eigen::Vector3d point3d = cs_f.Get_3Dpos(center, cs_f.Depth_Cloth(i, j), odom_q, odom_pos);
          point.x = point3d.x();
          point.y = point3d.y();
          point.z = point3d.z();
          if(cs_f.Coll_Flag(i, j) == 0.0){
              barrier_cloud->points.push_back(point);
          }
          else{
              access_cloud->points.push_back(point);
          }
      }
  }
  barrier_cloud->width = barrier_cloud->points.size();
  barrier_cloud->height = 1; // 表示无序点云
  barrier_cloud->is_dense = true;
  access_cloud->width = access_cloud->points.size();
  access_cloud->height = 1; // 表示无序点云
  access_cloud->is_dense = true;

  sensor_msgs::PointCloud2 pc_ros;
  pcl::toROSMsg(*barrier_cloud, pc_ros);
  pc_ros.header.stamp = ros::Time::now();
  pc_ros.header.frame_id = "world";
  Bpc_pub.publish(pc_ros);
  pcl::toROSMsg(*access_cloud, pc_ros);
  pc_ros.header.stamp = ros::Time::now();
  pc_ros.header.frame_id = "world";
  Hpc_pub.publish(pc_ros);
}

void depthImageCallback(const sensor_msgs::Image::ConstPtr& msg)
{   
    cv::Mat depth_image;
    cv::Mat depth_image_noinf;  
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    depth_image = cv_ptr->image.clone();
    depth_image_noinf = depth_image.clone(); // 先克隆一份，避免直接修改原始数据

    // 获取图像的尺寸
    int rows = depth_image_noinf.rows;
    int cols = depth_image_noinf.cols;

    // 遍历每个像素点
    for (int y = 0; y < rows; ++y) {
        for (int x = 0; x < cols; ++x) {
            float& pixel = depth_image_noinf.at<float>(y, x); // 获取当前像素的引用
            if (std::isinf(pixel)) {
                pixel = 100.0f; // 将 inf 替换为 100
            }
        }
    }


    // // 处理深度图像中的无穷大值
    // cv::MatIterator_<float> it, end;
    // for (it = cv_ptr->image.begin<float>(), end = cv_ptr->image.end<float>(); it != end; ++it)
    // {
    //     if (std::isinf(*it))
    //     {
    //         *it = 100.0f;
    //     }
    // }
    // depth_image_noinf = cv_ptr->image.clone();
    // std::cout << depth_image_noinf.rows << ", " << depth_image_noinf.cols << std::endl;
    cs_f.Input_Depth(depth_image_noinf);
    cs_f.Reset_Cloth();
    cs_f.Shot();
    pub_cloud();
    if(cs_f.commu_mode == 0){
        csf_msg.header = msg -> header;
        csf_msg.rows = cs_f.Coll_Flag.rows();
        csf_msg.cols = cs_f.Coll_Flag.cols();

        csf_msg.Coll_Flag.assign(cs_f.Coll_Flag.data(), cs_f.Coll_Flag.data() + cs_f.Coll_Flag.size());
        csf_msg.Depth_Image.assign(cs_f.Depth_Image.data(), cs_f.Depth_Image.data() + cs_f.Depth_Image.size());
        csf_msg.Depth_Cloth.assign(cs_f.Depth_Cloth.data(), cs_f.Depth_Cloth.data() + cs_f.Depth_Cloth.size());

        csf_pub.publish(csf_msg);
    }
    if(cs_f.commu_mode == 1 && has_traj){
        Waypoints wps = cs_f.Update_Waypoints(traj, odom_q, odom_pos);
        wps_msg.hard_waypoint.x = wps.hard_waypoint.x();
        wps_msg.hard_waypoint.y = wps.hard_waypoint.y();
        wps_msg.hard_waypoint.z = wps.hard_waypoint.z();

        wps_msg.soft_waypoints.clear();
        geometry_msgs::Point point_msg;
        for(Eigen::Vector3d point : wps.soft_waypoints){
            point_msg.x = point.x();
            point_msg.y = point.y();
            point_msg.z = point.z();
            wps_msg.soft_waypoints.push_back(point_msg);
        }
        wps_pub.publish(wps_msg);
    }
    
}


void odomCallback(const odom_fusion::OdomStamp::ConstPtr& msg){
    odom_pos(0) = msg->pose.pose.pose.position.x;
    odom_pos(1) = msg->pose.pose.pose.position.y;
    odom_pos(2) = msg->pose.pose.pose.position.z;

    odom_q.x() = msg->pose.pose.pose.orientation.x;
    odom_q.y() = msg->pose.pose.pose.orientation.y;
    odom_q.z() = msg->pose.pose.pose.orientation.z;
    odom_q.w() = msg->pose.pose.pose.orientation.w;
}

void trajCallback(const csf::Trajectory::ConstPtr& msg){
    traj = cs_f.Extract_Traj_fromMsg(msg);
    has_traj = true;
}

int main(int argc, char **argv)
{
    barrier_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
    access_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
    has_traj = false;
    
    // 初始化ROS节点
    ros::init(argc, argv, "csf_node");
    
    // 创建节点句柄
    ros::NodeHandle nh;
    odom_sub = nh.subscribe("odom", 1, &odomCallback, ros::TransportHints().tcpNoDelay());
    depth_sub = nh.subscribe("depth", 1, &depthImageCallback, ros::TransportHints().tcpNoDelay());
    if(cs_f.commu_mode == 1){
        traj_sub = nh.subscribe("/console_trajectory", 1, &trajCallback, ros::TransportHints().tcpNoDelay());
    }
    

    csf_pub = nh.advertise<csf::Cloth>("csf_topic", 1);
    Bpc_pub = nh.advertise<sensor_msgs::PointCloud2>("/barrier_pointcloud", 1);
    Hpc_pub = nh.advertise<sensor_msgs::PointCloud2>("/hole_pointcloud", 1);
    wps_pub = nh.advertise<csf::Waypoints>("/csf_waypoints", 1);
    // // 创建一个空白窗口
    // cv::namedWindow("Depth Image");

    // // 设置鼠标事件回调函数
    // cv::setMouseCallback("Depth Image", onMouse, NULL);

    //  // 创建一个空白窗口
    // cv::namedWindow("Marked Columns");

    // 循环等待回调函数
    ros::spin();

    return 0;
}