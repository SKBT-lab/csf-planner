#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "odom_fusion/OdomStamp.h"
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>

class DepthToPointCloud
{
public:
    DepthToPointCloud() : nh_("~")
    {
        // 订阅深度图话题
        image_transport::ImageTransport it(nh_);
        depth_sub_ = nh_.subscribe("/xv_sdk/xv_dev/tof_camera/image", 1, &DepthToPointCloud::depthCallback, this);
        odom_sub_ = nh_.subscribe("/fusion/odom", 1, &DepthToPointCloud::odomCallback, this);
        // 发布点云话题
        point_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/local_point_cloud", 1);
        global_point_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/global_point_cloud", 1);

        // 内参矩阵 (fx, fy, cx, cy)
        // K_ << 258, 0.0, 160,
        //       0.0, 258, 120,
        //       0.0, 0.0, 1.0;

        K_ <<   107.40475463867188, 0.0, 128.0,
                0.0, 107.40475463867188, 72.0,
                0.0, 0.0, 1.0;

        // 外参位姿 (姿态和平移)
        q_ = Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0); // 初始化为单位四元数
        t_ = Eigen::Vector3d(0.0, 0.0, 0.0);         // 初始化为零平移

        // 初始化全局点云
        global_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);
    }

// private:
    void odomCallback(const odom_fusion::OdomStamp::ConstPtr& msg) {
        // 获取位置
        t_<< msg->pose.pose.pose.position.x,
             msg->pose.pose.pose.position.y,
             msg->pose.pose.pose.position.z;

        // 获取姿态（四元数）
        
        q_.w() = msg->pose.pose.pose.orientation.w;
        q_.x() = msg->pose.pose.pose.orientation.x;
        q_.y() = msg->pose.pose.pose.orientation.y;
        q_.z() = msg->pose.pose.pose.orientation.z;
    }
    void depthCallback(const sensor_msgs::ImageConstPtr& msg)
    {
        // 将ROS图像消息转换为OpenCV图像
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

        // 获取深度图
        cv::Mat depth_image = cv_ptr->image;


        // 创建当前帧点云
        pcl::PointCloud<pcl::PointXYZ>::Ptr current_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        current_cloud->width = depth_image.cols;
        current_cloud->height = depth_image.rows;
        current_cloud->is_dense = false;

        // 遍历深度图，将有效像素点转换为点云
        for (int v = 0; v < depth_image.rows; ++v)
        {
            for (int u = 0; u < depth_image.cols; ++u)
            {
                float depth = depth_image.at<float>(v, u);

                // 忽略无效点
                if (std::isinf(depth)){
                    continue;
                    // std::cout << "valid!" << std::endl;
                }
                // std::cout << "no valid!" << std::endl;
                // 将像素坐标转换为相机坐标
                Eigen::Vector3d point_camera;
                point_camera(0) = (u - K_(0, 2)) * depth / K_(0, 0);
                point_camera(1) = (v - K_(1, 2)) * depth / K_(1, 1);
                point_camera(2) = depth;

                Eigen::Vector3d point_body(point_camera(2), -point_camera(0), -point_camera(1));

                // 将相机坐标转换为世界坐标
                Eigen::Vector3d point_world = q_.toRotationMatrix() * point_body + t_;

                // 将点添加到当前帧点云
                pcl::PointXYZ point;
                point.x = point_world(0);
                point.y = point_world(1);
                point.z = point_world(2);
                current_cloud->points.push_back(point);
            }
        }

        // 发布当前帧点云
        sensor_msgs::PointCloud2 current_cloud_msg;
        pcl::toROSMsg(*current_cloud, current_cloud_msg);
        current_cloud_msg.header = msg->header;
        current_cloud_msg.header.frame_id = "world";
        point_cloud_pub_.publish(current_cloud_msg);

        // 将当前帧点云合并到全局点云
        *global_cloud_ += *current_cloud;

        // 对合并后的点云进行降采样
        pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
        voxel_filter.setInputCloud(global_cloud_);
        voxel_filter.setLeafSize(0.03f, 0.03f, 0.03f); // 设置体素大小，单位：米
        voxel_filter.filter(*downsampled_cloud);

        // 发布全局点云地图
        sensor_msgs::PointCloud2 global_cloud_msg;
        pcl::toROSMsg(*downsampled_cloud, global_cloud_msg);
        global_cloud_msg.header.stamp = msg->header.stamp;
        global_cloud_msg.header.frame_id = "world";
        global_point_cloud_pub_.publish(global_cloud_msg);

    }

    void saveMap(const std::string& filename) {
        if (global_cloud_->empty()) {
            ROS_WARN("No point cloud map to save.");
            return;
        }

        // 保存点云地图为 PCD 文件
        pcl::io::savePCDFileBinary(filename, *global_cloud_);
        ROS_INFO("Point cloud map saved to %s", filename.c_str());
    }

    ros::NodeHandle nh_;
    ros::Subscriber depth_sub_;
    ros::Publisher point_cloud_pub_;
    ros::Publisher global_point_cloud_pub_;
    ros::Subscriber odom_sub_;

    Eigen::Matrix3d K_;              // 内参矩阵
    Eigen::Quaterniond q_;          // 外参位姿中的姿态 (四元数)
    Eigen::Vector3d t_;             // 外参位姿中的平移

    pcl::PointCloud<pcl::PointXYZ>::Ptr global_cloud_; // 全局点云地图
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "depth_to_point_cloud");
    DepthToPointCloud node;
    ros::spin();

    // 可选：保存点云地图
    node.saveMap("/home/skbt2/SKBT_Drone/src/planner/Utils/csf_utils/pc_map/output/global_map.pcd");

    return 0;
}