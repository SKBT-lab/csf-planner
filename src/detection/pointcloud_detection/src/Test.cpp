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

pcl::visualization::PCLVisualizer viewer("Livox Point Cloud Viewer");
// 创建一个PCL点云对象



void applyPassThroughFilter(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
                            pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_filtered,
                            float x_min, float x_max,
                            float y_min, float y_max,
                            float z_min, float z_max) {
    // 创建直通滤波器对象
    pcl::PassThrough<pcl::PointXYZI> pass;

    // 设置X轴的过滤范围并进行过滤
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(x_min, x_max);
    pass.filter(*cloud_filtered);

    // 设置Y轴的过滤范围并进行过滤
    pass.setInputCloud(cloud_filtered);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(y_min, y_max);
    pass.filter(*cloud_filtered);

    // 设置Z轴的过滤范围并进行过滤
    pass.setInputCloud(cloud_filtered);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(z_min, z_max);
    pass.filter(*cloud_filtered);
}

void cloudCallback(const livox_ros_driver2::CustomMsgConstPtr& custom_msg) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    // 遍历Livox自定义消息中的点，将其转换为PCL点云格式
    for (const auto& point : custom_msg->points){
        pcl::PointXYZI pcl_point;
        pcl_point.x = point.x;
        pcl_point.y = point.y;
        pcl_point.z = point.z;
        pcl_point.intensity = point.reflectivity;
        cloud->points.push_back(pcl_point);

    }

    cloud->width = cloud->points.size();
    cloud->height = 1;
    cloud->is_dense = true;

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>);
    applyPassThroughFilter(cloud, cloud_filtered, 0.0, 4.0, -1.5, 1.5, -1.5, 1.5);

    // 清空viewer中的所有点云和形状
    viewer.removeAllPointClouds();
    viewer.removeAllShapes();



    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> original_color(cloud, 0, 255, 0);
    viewer.addPointCloud<pcl::PointXYZI>(cloud, original_color, "original cloud");

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> filtered_color(cloud_filtered, 255, 0, 0);
    viewer.addPointCloud<pcl::PointXYZI>(cloud_filtered, filtered_color, "filtered cloud");

    viewer.spinOnce(0);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "livox_pointcloud_processor");
    ros::NodeHandle nh;

    viewer.setBackgroundColor(0, 0, 0);

    // 添加自定义坐标系
    // Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    // transform.translation() << 0.0, 0.0, 0.0;  // 偏移量
    // transform.rotate(Eigen::AngleAxisf(M_PI/2, Eigen::Vector3f::UnitY()));  
    // transform.rotate(Eigen::AngleAxisf(-M_PI/2, Eigen::Vector3f::UnitX()));  
    viewer.addCoordinateSystem(1.0);
    viewer.initCameraParameters();
    // 设置相机位置，调换X轴和Y轴的方向
    viewer.setCameraPosition(-2.0, 0.0, 0.0,  // 相机位置
                              0.0, 0.0, 0.0,  // 目标位置
                              0.0, 0.0, 1.0); // 上方向
    
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_init(new pcl::PointCloud<pcl::PointXYZI>);
    // 添加初始的空点云到可视化器，并使用单一颜色处理程序（绿色）
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> single_color(cloud_init, 0, 255, 0);
    viewer.addPointCloud<pcl::PointXYZI>(cloud_init, single_color, "dynamic cloud");

    // 订阅Livox自定义点云消息
    ros::Subscriber sub = nh.subscribe("/livox/lidar", 1, cloudCallback);

    ros::spin();
    return 0;
}