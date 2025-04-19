#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <boost/foreach.hpp>

int main()
{
    // 指定要读取的 ROS bag 文件路径和要保存的 PCD 文件路径
    std::string bag_file = "/home/skbt/SKBT_Drone/Dataset/circle_Lidar1.bag";
    std::string pcd_file = "/home/skbt/SKBT_Drone/Dataset/output.pcd";

    // Open the bag file
    rosbag::Bag bag;
    try
    {
        bag.open(bag_file, rosbag::bagmode::Read);
    }
    catch (rosbag::BagIOException const& ex)
    {
        ROS_ERROR("Failed to open bag file %s: %s", bag_file.c_str(), ex.what());
        return -1;
    }

    // Define the topic to read from

    std::string topic = "/your/pointcloud/topic";

    // Set up the view to read messages of type sensor_msgs::PointCloud2
    rosbag::View view(bag, rosbag::TopicQuery(topic));

    // Accumulate point clouds
    pcl::PointCloud<pcl::PointXYZ>::Ptr accumulated_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    BOOST_FOREACH(rosbag::MessageInstance const m, view)
    {
        sensor_msgs::PointCloud2::ConstPtr cloud_msg = m.instantiate<sensor_msgs::PointCloud2>();
        if (cloud_msg != nullptr)
        {
            // Convert to PCL point cloud
            pcl::PointCloud<pcl::PointXYZ> cloud;
            pcl::fromROSMsg(*cloud_msg, cloud);

            // Check if cloud has data
            if (!cloud.empty())
            {
                *accumulated_cloud += cloud;
            }
            else
            {
                ROS_WARN("Empty point cloud received, skipping...");
            }
        }
    }

    // Check if accumulated_cloud has data
    if (accumulated_cloud->empty())
    {
        ROS_ERROR("No valid point cloud data found, unable to save to PCD file.");
        bag.close();
        return -1;
    }

    // Save accumulated point cloud to PCD file
    pcl::io::savePCDFileASCII(pcd_file, *accumulated_cloud);
    ROS_INFO("Saved %s", pcd_file.c_str());

    bag.close();
    return 0;
}