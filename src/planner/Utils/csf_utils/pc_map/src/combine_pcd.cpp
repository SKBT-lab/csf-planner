#include <iostream>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>

int main(int argc, char** argv)
{
    // 检查命令行参数
    if (argc != 4)
    {
        std::cerr << "Usage: " << argv[0] << " <input_pcd1> <input_pcd2> <output_pcd>" << std::endl;
        return -1;
    }

    // 获取命令行参数
    std::string input_file1 = argv[1]; // 第一个输入PCD文件路径
    std::string input_file2 = argv[2]; // 第二个输入PCD文件路径
    std::string output_file = argv[3]; // 降采样后点云输出路径

    // 创建点云对象
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2_trans(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    // 读取第一个PCD文件
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(input_file1, *cloud1) == -1)
    {
        std::cerr << "Error: Could not read file " << input_file1 << std::endl;
        return -1;
    }

    // 读取第二个PCD文件
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(input_file2, *cloud2) == -1)
    {
        std::cerr << "Error: Could not read file " << input_file2 << std::endl;
        return -1;
    }

    Eigen::Vector3d axis(0, 0, 1);

    // 定义旋转角度（以弧度为单位）
    double angle = 33.0 * M_PI / 180.0; // 将角度转换为弧度

    // 通过轴角生成旋转矩阵
    Eigen::AngleAxisd rotation_vector(angle, axis);
    Eigen::Matrix3d rotation_matrix = rotation_vector.toRotationMatrix();

    Eigen::Matrix4d transform_matrix = Eigen::Matrix4d::Identity(); // 初始化为单位矩阵
    transform_matrix.block<3, 3>(0, 0) = rotation_matrix; // 将旋转矩阵填入左上角 3x3 部分

    pcl::transformPointCloud(*cloud2, *cloud2_trans, transform_matrix);

    // 创建直通滤波对象
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud2_trans);              // 设置输入点云
    pass.setFilterFieldName("x");           // 设置过滤字段为x坐标
    pass.setFilterLimits(5.0, std::numeric_limits<float>::max()); // 设置x坐标的滤波范围
    pass.filter(*cloud2_filtered);           // 执行滤波

    // 合并两个点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr merged_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    *merged_cloud = *cloud1 + *cloud2_filtered; // PCL 提供了 "+" 操作符用于合并点云

    // 对合并后的点云进行降采样
    pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
    voxel_filter.setInputCloud(merged_cloud);
    voxel_filter.setLeafSize(0.03f, 0.03f, 0.03f); // 设置体素大小，单位：米
    voxel_filter.filter(*downsampled_cloud);

    // 保存降采样后的点云到输出文件
    if (pcl::io::savePCDFile<pcl::PointXYZ>(output_file, *downsampled_cloud) == -1)
    {
        std::cerr << "Error: Could not save file " << output_file << std::endl;
        return -1;
    }

    std::cout << "Downsampled point cloud saved to " << output_file << std::endl;
    std::cout << "Downsampled point cloud size: " << downsampled_cloud->size() << std::endl;

    return 0;
}