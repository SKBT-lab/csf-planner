#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <Eigen/Dense>
#include <pcl/io/pcd_io.h>
#include "odom_fusion/OdomStamp.h"

class PointCloudMapper {
public:
    PointCloudMapper() {
        // 订阅点云和里程计话题
        pc_sub_ = nh_.subscribe("/xv_sdk/xv_dev/tof_camera/point_cloud", 1, &PointCloudMapper::pointCloudCallback, this);
        odom_sub_ = nh_.subscribe("/fusion/odom", 1, &PointCloudMapper::odomCallback, this);

        // 发布累积的点云地图
        globalmap_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/global_point_cloud_map", 1);
        localmap_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/local_point_cloud_map", 1);
        // 初始化全局点云地图
        global_map_.reset(new pcl::PointCloud<pcl::PointXYZRGB>);

        T_c_to_b.setIdentity();
        T_c_to_b.block<3, 3>(0, 0) << 0, 0, 1,
                                      -1, 0, 0,
                                      0, -1, 0;
        scale_factor = 0.5;
        scale_M(0, 0) = scale_M(1, 1) = scale_M(2, 2) = scale_factor;
        scale_M(3, 3) = 1;
        ROS_INFO("Point Cloud Mapper initialized.");
    }

    void odomCallback(const odom_fusion::OdomStamp::ConstPtr& msg) {
        // 获取位置
        Eigen::Vector3d translation(
            msg->pose.pose.pose.position.x,
            msg->pose.pose.pose.position.y,
            msg->pose.pose.pose.position.z
        );

        // 获取姿态（四元数）
        Eigen::Quaterniond quaternion(
            msg->pose.pose.pose.orientation.w,
            msg->pose.pose.pose.orientation.x,
            msg->pose.pose.pose.orientation.y,
            msg->pose.pose.pose.orientation.z
        );

        // 构建 4x4 变换矩阵
        current_pose_.setIdentity();
        current_pose_.block<3, 3>(0, 0) = quaternion.toRotationMatrix();
        current_pose_.block<3, 1>(0, 3) = translation;
        current_pose_ = scale_M * current_pose_ * T_c_to_b;
        // current_pose_ = scale_M * T_c_to_b * current_pose_;
    }

    void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& pc_msg) {
        if (current_pose_.isIdentity()) {
            ROS_WARN("No valid pose received yet.");
            return;
        }

        // 将 ROS PointCloud2 转换为 PCL PointCloud
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::fromROSMsg(*pc_msg, *cloud);

        // 将点云从机体坐标系转换到世界坐标系
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::transformPointCloud(*cloud, *transformed_cloud, current_pose_);

        // 将转换后的点云累积到全局地图中
        *global_map_ += *transformed_cloud;

        sensor_msgs::PointCloud2 map_msg;
        pcl::toROSMsg(*transformed_cloud, map_msg);
        map_msg.header.stamp = ros::Time::now();
        map_msg.header.frame_id = "world";  // 设置世界坐标系

        // 发布点云地图
        localmap_pub_.publish(map_msg);
        // 发布累积的点云地图
        publish_globalMap();
    }

    void publish_globalMap() {
        if (global_map_->empty()) {
            ROS_WARN("Global map is empty.");
            return;
        }

        // 将 PCL PointCloud 转换为 ROS PointCloud2
        sensor_msgs::PointCloud2 map_msg;
        pcl::toROSMsg(*global_map_, map_msg);
        map_msg.header.stamp = ros::Time::now();
        map_msg.header.frame_id = "world";  // 设置世界坐标系

        // 发布点云地图
        globalmap_pub_.publish(map_msg);
    }

    void saveMap(const std::string& filename) {
        if (global_map_->empty()) {
            ROS_WARN("No point cloud map to save.");
            return;
        }

        // 保存点云地图为 PCD 文件
        pcl::io::savePCDFileBinary(filename, *global_map_);
        ROS_INFO("Point cloud map saved to %s", filename.c_str());
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber pc_sub_;
    ros::Subscriber odom_sub_;
    ros::Publisher globalmap_pub_;
    ros::Publisher localmap_pub_;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr global_map_;  // 全局点云地图
    Eigen::Matrix4d current_pose_;                      // 当前位姿（4x4 变换矩阵）
    Eigen::Matrix4d T_c_to_b;
    Eigen::Matrix4d scale_M;
    double scale_factor;

};

int main(int argc, char** argv) {
    ros::init(argc, argv, "point_cloud_mapper");

    PointCloudMapper mapper;

    ros::spin();

    // 可选：保存点云地图
    mapper.saveMap("/home/skbt2/SKBT_Drone/src/planner/Utils/csf_utils/PC_map/output/global_map.pcd");

    return 0;
}