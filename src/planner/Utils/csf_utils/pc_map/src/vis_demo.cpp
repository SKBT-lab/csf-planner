#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <sensor_msgs/PointCloud2.h>

int main(int argc, char** argv)
{
    // 初始化ROS节点
    ros::init(argc, argv, "pcd_to_pointcloud_publisher");
    ros::NodeHandle nh;

    // 检查命令行参数
    if (argc != 2)
    {
        ROS_ERROR("用法: %s <pcd文件>", argv[0]);
        return -1;
    }

    // 创建点云对象
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // 读取PCD文件
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(argv[1], *cloud) == -1)
    {
        ROS_ERROR("无法读取文件 %s", argv[1]);
        return -1;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
    voxel_filter.setInputCloud(cloud);
    voxel_filter.setLeafSize(0.03f, 0.03f, 0.03f); // 设置体素大小，单位：米
    voxel_filter.filter(*downsampled_cloud);

    // 创建发布者，发布点云消息到话题 "/point_cloud"
    ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2>("/global_point_cloud2", 1);

    // 设置发布频率
    ros::Rate rate(1); // 1 Hz

    while (ros::ok())
    {
        // 将PCL点云转换为ROS消息
        sensor_msgs::PointCloud2 output;
        pcl::toROSMsg(*downsampled_cloud, output);

        // 设置消息头
        output.header.stamp = ros::Time::now();
        output.header.frame_id = "world"; // 设置坐标系，例如 "map"

        // 发布点云消息
        pub.publish(output);

        ROS_INFO("num of points: %ld", cloud->size());

        // 按照指定频率休眠
        rate.sleep();
    }

    return 0;
}