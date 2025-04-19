#include <iostream>
#include <ros/ros.h>
#include <livox_ros_driver2/CustomMsg.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <thread>  
#include <chrono>  
#include <Eigen/Dense>
#include "CSF_Lidar.h"

int main()
{
    // 指定 PCD 文件路径
    std::string pcd_file = "/home/skbt/SKBT_Drone/Dataset/output.pcd";

    // 创建一个 PointCloud 变量
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // 读取 PCD 文件
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_file, *cloud) == -1) // 这里解引用指针
    {
        PCL_ERROR("Couldn't read file %s \n", pcd_file.c_str());
        return (-1);
    }

    std::cout << "Loaded " << cloud->width * cloud->height << " data points from " << pcd_file << std::endl;

    // 创建一个PCL的可视化对象
    pcl::visualization::PCLVisualizer viewer("Cloud Viewer");

    viewer.setBackgroundColor(0, 0, 0);
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
    viewer.addCoordinateSystem(1.0);
    viewer.initCameraParameters();

    viewer.addPointCloud<pcl::PointXYZ>(cloud, "sample cloud");

    // 保持可视化窗口打开，直到用户关闭它
    while (!viewer.wasStopped()) {}

    return 0;
}