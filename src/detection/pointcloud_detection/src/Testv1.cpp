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

pcl::visualization::PCLVisualizer viewer("Livox Point Cloud Viewer");
// 创建一个PCL点云对象

C_S_FILTER_LIDAR csf;
std::vector<int> R;
std::vector<int> G;
std::vector<int> B;

double Get_right_distance(pcl::PointCloud<pcl::PointXYZI>::Ptr Cloud){
  pcl::PointCloud<pcl::PointXYZI>::Ptr r_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  csf.applyPassThroughFilter(Cloud, r_cloud, -0.5, 0.5, -10.0, 0.0, -0.5, 0.5);
  double r_dis = std::numeric_limits<double>::max();

  for (const auto& point : r_cloud->points)
  {
    if (point.y < r_dis){
          r_dis = point.y;
    }
  }
  return r_dis;
}

void cloudCallback(const livox_ros_driver2::CustomMsgConstPtr& custom_msg) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);

    // 遍历Livox自定义消息中的点，将其转换为PCL点云格式
    for (const auto& point : custom_msg->points) {
        pcl::PointXYZI pcl_point;
        // std::cout << point.x << std::endl;
        pcl_point.x = point.x;
        pcl_point.y = point.y;
        pcl_point.z = point.z;
        pcl_point.intensity = point.reflectivity;
        cloud->points.push_back(pcl_point);
    }

    cloud->width = cloud->points.size();
    cloud->height = 1;
    cloud->is_dense = true;

  
   

    viewer.removeAllPointClouds();
    viewer.removeAllShapes();

    //针对柱子的测试
    if(csf.state == 1){
        csf.Input_Cloud(cloud, 0.2, csf.x_debug, -3, 3, 0, 2);
        csf.Reset_Cloth();

        csf.DownSampling();
        csf.Euclidean_Clustering();

        std::vector<Eigen::Vector3d> sort_list;
        sort_list.push_back(csf.Clusters_info[0].min_y);
        sort_list.push_back(csf.Clusters_info[0].max_y);
        sort_list.push_back(csf.Clusters_info[1].min_y);
        sort_list.push_back(csf.Clusters_info[1].max_y);
        std::sort(sort_list.begin(), sort_list.end(), [](const Eigen::Vector3d& a, const Eigen::Vector3d& b) {
        return a.y() < b.y();
        });
        Eigen::Vector3d p(0, 0, 0);
        
        p.x() = (sort_list[1].x() + sort_list[2].x()) / 2;
        p.y() = (sort_list[1].y() + sort_list[2].y()) / 2;
        p.z() = 1.5;
        // cout << csf.clusters_num << std::endl;
        for (size_t i = 0; i < csf.Clusters_info.size(); ++i)
        {   
            //if(csf.Clusters_info[i].span_y > 0.1 && csf.Clusters_info[i].span_y < 1.5 && csf.Clusters_info[i].span_z > 0.2 && csf.Clusters_info[i].span_z < 2.0){
            if(csf.Clusters_info[i].span_y > 0.1 && csf.Clusters_info[i].span_y < 1.0 && csf.Clusters_info[i].span_z > 0 && csf.Clusters_info[i].span_z < 2.2){  
                pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI>::Ptr color_handler(new pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI>(csf.Clusters_info[i].cloud, R[i], G[i], B[i]));
                viewer.addPointCloud<pcl::PointXYZI>(csf.Clusters_info[i].cloud, *color_handler, "cluster_" + std::to_string(i));
                viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cluster_" + std::to_string(i));
                viewer.addSphere(pcl::PointXYZ(csf.Clusters_info[i].center.x(), csf.Clusters_info[i].center.y(), csf.Clusters_info[i].center.z()), 0.1, "sphere" + std::to_string(i));
                viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.5, 0.0, "sphere" + std::to_string(i));
            }

        }

        viewer.addSphere(pcl::PointXYZ(p.x(), p.y(), p.z()), 0.1, "waypoint");
        viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 0.0, "waypoint");
    
    //     if(csf.Clusters_info.size() > 1){
    //         csf.process_level1();
        
    //         for (size_t i = 0; i < csf.level1_waypoints.size(); ++i){
    //             viewer.addSphere(pcl::PointXYZ(csf.level1_waypoints[i].x(), csf.level1_waypoints[i].y(), csf.level1_waypoints[i].z()), 0.1, "waypoint" + std::to_string(i));
    //             viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 0.0, "waypoint" + std::to_string(i));
    //         }
    //     }
    // }
    }
    
    

    if(csf.state == 2){
        // 针对隧道的测试
        csf.Input_Cloud(cloud);
        csf.Reset_Cloth();
        
        csf.DownSampling();
        csf.Euclidean_Clustering();
        cout << csf.clusters_num << std::endl;

        for (size_t i = 0; i < csf.Clusters_info.size(); ++i)
        {   
            if(csf.Clusters_info[i].span_y > 1.0 && csf.Clusters_info[i].span_y < 2.0 && csf.Clusters_info[i].span_z > 0.5 && csf.Clusters_info[i].span_z < 2.0){
                pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI>::Ptr color_handler(new pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI>(csf.Clusters_info[i].cloud, R[i], G[i], B[i]));
                viewer.addPointCloud<pcl::PointXYZI>(csf.Clusters_info[i].cloud, *color_handler, "cluster_" + std::to_string(i));
                viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cluster_" + std::to_string(i));
                csf.Find_suidao(csf.Clusters_info[i]);
                viewer.addSphere(pcl::PointXYZ(csf.Suidao_center.x(), csf.Suidao_center.y(), csf.Suidao_center.z()), 0.1, "sphere" + std::to_string(i));
                viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.5, 0.0, "sphere" + std::to_string(i));
                // 设置箭头的起点和终点
                pcl::PointXYZ start(csf.Suidao_center.x(), csf.Suidao_center.y(), csf.Suidao_center.z());
                pcl::PointXYZ end(csf.Suidao_center.x() + 1.0 * sin(csf.Suidao_angle), csf.Suidao_center.y() - 1.0 * cos(csf.Suidao_angle), csf.Suidao_center.z());
                viewer.addArrow(end, start, 0.0, 1.0, 1.0, false, "arrow"+ std::to_string(i));
            }

        }
    }

    
    if(csf.state == 3){
        //针对窗口的测试
        csf.Input_Cloud(cloud, 0.01, 2.0, -3, 3, -3, 3);
        csf.Reset_Cloth();

        csf.Find_wall();
        
        if(csf.Get_Wallimg(csf.Cloud)){
            pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI>::Ptr color_handler(new pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI>(csf.Wall_Cloud, 255, 0, 0));
            viewer.addPointCloud<pcl::PointXYZI>(csf.Wall_Cloud, *color_handler, "wall" );
            viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "wall");
            viewer.addSphere(pcl::PointXYZ(csf.Window_center.x(), csf.Window_center.y(), csf.Window_center.z()), 0.1, "center");
            viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.5, 0.0, "center");
        }

        cv::imshow("Wall", csf.result_img);
        cv::waitKey(10);
    
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> B_color(csf.Barrier_Cloud, 255, 0, 0);
        viewer.addPointCloud<pcl::PointXYZI>(csf.Barrier_Cloud, B_color, "B_Cloth");

        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> H_color(csf.Hole_Cloud, 0, 0, 255);
        viewer.addPointCloud<pcl::PointXYZI>(csf.Hole_Cloud, H_color, "H_Cloth");

        viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "B_Cloth");
        viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "H_Cloth");
    }
    
    if(csf.state == 4){
        //针对右侧点的测试
        double r_dis = Get_right_distance(cloud);
        viewer.addSphere(pcl::PointXYZ(0, r_dis + 0.2, 1.0), 0.1, "right");
        viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.5, 0.0, "right");
        
       
    }

    if(csf.state == 5){
         //针对圆环的测试
        csf.Input_Cloud(cloud);
        csf.Reset_Cloth();

        csf.DownSampling();
        csf.Euclidean_Clustering();
        cout << csf.clusters_num << std::endl;

        for (size_t i = 0; i < csf.Clusters_info.size(); ++i)
        {   
            if(csf.Clusters_info[i].span_y > 1.0 && csf.Clusters_info[i].span_y < 2.0 && csf.Clusters_info[i].span_z > 0.5 && csf.Clusters_info[i].span_z < 2.0){
                pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI>::Ptr color_handler(new pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI>(csf.Clusters_info[i].cloud, R[i], G[i], B[i]));
                viewer.addPointCloud<pcl::PointXYZI>(csf.Clusters_info[i].cloud, *color_handler, "cluster_" + std::to_string(i));
                viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cluster_" + std::to_string(i));
                viewer.addSphere(pcl::PointXYZ(csf.Clusters_info[i].center.x(), csf.Clusters_info[i].center.y(), csf.Clusters_info[i].center.z()), 0.1, "sphere" + std::to_string(i));
                viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.5, 0.0, "sphere" + std::to_string(i));
            }

        }
    }
    
    // int iteration = 0;
    // while (iteration < csf.MaxIterations)
    // {
    //     iteration ++;
    //     csf.Collision_Detection();
    //     csf.Cal_force();
    //     csf.Update_Cloth();
    //     // std::cout << Vel_Cloth << std::endl;
    //     csf.Get_Visualization_Cloth();
    //     // 清空viewer中的所有点云和形状
    //     viewer.removeAllPointClouds();
    //     viewer.removeAllShapes();

    //     // 设置红色和蓝色渲染器
        

    //     pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> B_color(csf.Barrier_Cloud, 255, 0, 0);
    //     viewer.addPointCloud<pcl::PointXYZI>(csf.Barrier_Cloud, B_color, "B_Cloth");

    //     pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> H_color(csf.Hole_Cloud, 0, 0, 255);
    //     viewer.addPointCloud<pcl::PointXYZI>(csf.Hole_Cloud, H_color, "H_Cloth");

    //     viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "B_Cloth");
    //     viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "H_Cloth");

    //     pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> original_color(cloud, 0, 255, 0);
    //     viewer.addPointCloud<pcl::PointXYZI>(cloud, original_color, "original cloud");
    //     viewer.spinOnce(100);
    //     if (csf.CalculateAverageNorm(csf.Vel_Grid) < csf.thresh){
    //         break;
    //     }
    // }

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> original_color(csf.Cloud, 0, 255, 0);
    viewer.addPointCloud<pcl::PointXYZI>(csf.Cloud, original_color, "original cloud");

    viewer.spinOnce(0);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "livox_pointcloud_processor");
    ros::NodeHandle nh;
    R.push_back(255);
    G.push_back(0);
    B.push_back(0);

    R.push_back(0);
    G.push_back(0);
    B.push_back(255);

    R.push_back(95);
    G.push_back(0);
    B.push_back(191);
    
    for (size_t i = 0; i < 10; ++i)
    {
        int r = (rand() % 256);
        int g = (rand() % 256);
        int b = (rand() % 256);
        R.push_back(r);
        G.push_back(g);
        B.push_back(b);
    }
    

    viewer.setBackgroundColor(0, 0, 0);

    // 添加自定义坐标系
    // Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    // transform.translation() << 0.0, 0.0, 0.0;  // 偏移量
    // transform.rotate(Eigen::AngleAxisf(M_PI/2, Eigen::Vector3f::UnitY()));  
    // transform.rotate(Eigen::AngleAxisf(-M_PI/2, Eigen::Vector3f::UnitX()));  
    viewer.addCoordinateSystem(1.0);
    viewer.initCameraParameters();
    // 设置相机位置，调换X轴和Y轴的方向
    viewer.setCameraPosition(-5.0, 0.0, 1.0,  // 相机位置
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