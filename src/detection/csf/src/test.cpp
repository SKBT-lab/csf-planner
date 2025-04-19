#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/highgui/highgui.hpp"
#include "CSF.h"

#include <mutex>


cv::Mat depth_image;
cv::Mat depth_image_noinf;
bool barrier_flag;
C_S_FILTER csf(320, 240);
Eigen::Quaterniond Q;
Eigen::Vector3d P;
//pcl::visualization::PCLVisualizer viewer("Point Cloud Viewer");
MiniSnapTraj traj;

PlanningVisualization::Ptr trajVisual_;

ros::Publisher Bpc_pub;
ros::Publisher Hpc_pub;

nav_msgs::Path trajPath;
ros::Publisher trajPath_pub;

ros::Timer* timer_ptr;
bool start_flag;

std::mutex csf_mutex;


void getVisual(const MiniSnapTraj& trajectory){
    trajVisual_->displayWaypoints(trajectory.waypoints);
    // std::cout << "WPS_nums: " << trajectory.waypoints.size() << std::endl;
    
    if(trajectory.hard_waypoints_ID != -1){
        std::vector<Eigen::Vector3d> hwps;
        hwps.push_back(trajectory.hard_waypoints);
        trajVisual_->displayHardWaypoints(hwps);
    }
    
    std::vector<Eigen::Vector3d> swps = trajectory.soft_waypoints.points;
    for(int i = 0; i <  trajectory.soft_waypoints.points.size(); i++){
        Traj_info tj_info  = csf.tg.getTrajInfo(trajectory, trajectory.soft_waypoints.times[i]);
        swps.push_back(tj_info.position);
    }
    trajVisual_->displaySoftWaypoints(swps);
    // for (size_t i = 0; i < trajectory.soft_waypoints.times.size(); i++)
    // {
    //     std::cout << trajectory.soft_waypoints.times[i] << " ";
    // }
    // std::cout << std::endl;

    double traj_len = 0.0;
    int count = 1;
    Eigen::Vector3d cur, vel;
    cur.setZero();
    geometry_msgs::PoseStamped poses;
    trajPath.header.frame_id = poses.header.frame_id = "world";
    trajPath.header.stamp    = poses.header.stamp    = ros::Time::now();
    double yaw = 0.0;
    poses.pose.orientation   = tf::createQuaternionMsgFromYaw(yaw);
    trajPath.poses.clear();
    ros::Rate loop_rate(1000);
    for(double t = 0.0; t < trajectory.time_duration; t += 0.1, count++)   // go through each segment
    {   
        auto info = csf.tg.getTrajInfo(trajectory, t);
        cur = info.position;
        // vel = info.velocity;
        poses.pose.position.x = cur[0];
        poses.pose.position.y = cur[1];
        poses.pose.position.z = cur[2];
        trajPath.poses.push_back(poses);
        // std::cout << cur << std::endl;
        // std::cout << '###############' << std::endl;
        // loop_rate.sleep();
    } 
    trajPath_pub.publish(trajPath);
}

void depthImageCallback(const sensor_msgs::Image::ConstPtr& msg)
{   
    // 将ROS图像消息转换为OpenCV格式
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
   
    // 处理深度图像中的无穷大值
    cv::MatIterator_<float> it, end;
    for (it = cv_ptr->image.begin<float>(), end = cv_ptr->image.end<float>(); it != end; ++it)
    {
        if (std::isinf(*it))
        {
            *it = 100.0f;
        }
    }
    depth_image_noinf = cv_ptr->image;
    start_flag = true;


}

void timerCallback(const ros::TimerEvent&)
{   
    if(start_flag){
        pcl::PointCloud<pcl::PointXYZ>::Ptr barrier_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr access_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        
        // auto start_t = std::chrono::high_resolution_clock::now();
        csf.Input_Depth(depth_image_noinf);
        csf.Reset_Cloth();
        csf.Shot();
        // auto end_t = std::chrono::high_resolution_clock::now();
        // std::chrono::duration<double, std::milli> duration = end_t - start_t;
        // std::cout << "Cloth Update Time: " << duration.count() << " ms" << std::endl;
        if(P[2] > 1){
            std::pair<MiniSnapTraj, bool> traj_info;
            traj_info = csf.Update_Trajectory(traj, Q, P, 0);
            traj = traj_info.first;
        }
            
        getVisual(traj);

        cv::Point2f cen(0.5 * csf.width * csf.Kspars, 0.5 * csf.height * csf.Kspars);
        Eigen::Vector3d cen_pos = csf.Get_3Dpos(cen, csf.Depth_Cloth(int(0.5 * csf.height), int(0.5 * csf.width)), Q, P);
        std::cout << "center: " <<  cen_pos << std::endl;
        
        // 布料转化为点云，用于可视化
        for(int i = 0; i < csf.height; i++){
            for(int j = 0; j < csf.width; j++){
                // std::cout << i << ", " << j << ": " << std::endl;
                // std::cout << "depth: " << csf.Depth_Cloth(i, j) << std::endl; 
                cv::Point2f center(j * csf.Kspars, i * csf.Kspars);
                Eigen::Vector3d point3d = csf.Get_3Dpos(center, csf.Depth_Cloth(i, j), Q, P);
                // std::cout << "point: " << point3d << std::endl; 
                pcl::PointXYZ point;
                point.x = point3d.x();
                point.y = point3d.y();
                point.z = point3d.z();
                if(csf.Coll_Flag(i, j) == 0.0){
                    barrier_cloud->points.push_back(point);
                }
                else{
                    access_cloud->points.push_back(point);
                }
            }
        }
        std::cout << "111111" << std::endl;
        barrier_cloud->width = barrier_cloud->points.size();
        barrier_cloud->height = 1; // 表示无序点云
        barrier_cloud->is_dense = true;
        access_cloud->width = access_cloud->points.size();
        access_cloud->height = 1; // 表示无序点云
        access_cloud->is_dense = true;
        std::cout << "222222" << std::endl;
        

        sensor_msgs::PointCloud2 pc_ros;
        pcl::toROSMsg(*barrier_cloud, pc_ros);
        pc_ros.header.stamp = ros::Time::now();
        pc_ros.header.frame_id = "world";
        Bpc_pub.publish(pc_ros);
        pcl::toROSMsg(*access_cloud, pc_ros);
        pc_ros.header.stamp = ros::Time::now();
        pc_ros.header.frame_id = "world";
        Hpc_pub.publish(pc_ros);

        

        // viewer.removeAllPointClouds();
        // viewer.removeAllShapes();

        // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>::Ptr access_color_handler(new pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(access_cloud, 0, 255, 0));
        // viewer.addPointCloud<pcl::PointXYZ>(access_cloud, *access_color_handler, "access_cloth");
        // viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "access_cloth");

        // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>::Ptr barrier_color_handler(new pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(barrier_cloud, 255, 0, 0));
        // viewer.addPointCloud<pcl::PointXYZ>(barrier_cloud, *barrier_color_handler, "barrier_cloth");
        // viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "barrier_cloth");

        // viewer.addSphere(pcl::PointXYZ(wp.hard_waypoint.x(), wp.hard_waypoint.y(), wp.hard_waypoint.z()), 0.05, "hwp");
        // viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.5, 0.0, "hwp");

        // for(int i = 0; i < wp.soft_waypoints.size(); i++){
        //     viewer.addSphere(pcl::PointXYZ(wp.soft_waypoints[i].x(), wp.soft_waypoints[i].y(), wp.soft_waypoints[i].z()), 0.05, "swp" + std::to_string(i));
        //     viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 0.0, 1.0, "swp" + std::to_string(i));
        // }

        
        // // viewer.addLine(pcl::PointXYZ(box_list[0].x(), box_list[0].y(), box_list[0].z()), pcl::PointXYZ(box_list[1].x(), box_list[1].y(), box_list[1].z()), 0.0, 0.0, 1.0, "line1");
        // // viewer.addLine(pcl::PointXYZ(box_list[1].x(), box_list[1].y(), box_list[1].z()), pcl::PointXYZ(box_list[2].x(), box_list[2].y(), box_list[2].z()), 0.0, 0.0, 1.0, "line2");
        // // viewer.addLine(pcl::PointXYZ(box_list[2].x(), box_list[2].y(), box_list[2].z()), pcl::PointXYZ(box_list[3].x(), box_list[3].y(), box_list[3].z()), 0.0, 0.0, 1.0, "line3");
        // // viewer.addLine(pcl::PointXYZ(box_list[0].x(), box_list[0].y(), box_list[0].z()), pcl::PointXYZ(box_list[3].x(), box_list[3].y(), box_list[3].z()), 0.0, 0.0, 1.0, "line4");
        
        // // viewer.addLine(pcl::PointXYZ(0.0, 1.0, 1.2), pcl::PointXYZ(5.0, 1.0, 1.2), 0.0, 0.0, 1.0, "tra");
        // viewer.spinOnce(0);

        // csf.Barrier_Check();
        // csf.Find_Hole(Q, P);
        csf.Show_Result('H');

        // Collision_info c_info = csf.Barrier_Collision_Detection(traj, Q, P);
        // Eigen::Vector3d UV1 = c_info.UV1;
        // // for(int i = 0; i < box_list.size(); i++){
        // //     if(box_list[i][0] > 0 && box_list[i][0] < csf.width_ori && box_list[i][1] > 0 && box_list[i][1] < csf.height_ori){
        // //         cv::Point center(box_list[i][0], box_list[i][1]);
        // //         cv::circle(csf.CV_Cloth, center, 1, cv::Scalar(0, 0, 255), -1);
        // //     }
        // // }
        // if(UV1[0] != -1){
        //     cv::Point center(UV1.x() * csf.Kspars, UV1.y() * csf.Kspars);
        //     cv::circle(csf.CV_Cloth, center, 2, cv::Scalar(255, 0, 255), -1);
        // }
        // cv::imshow("cvCloth", csf.CV_Cloth);
        cv::waitKey(5);
    }
}


void odomCallback(const odom_fusion::OdomStamp& msg){
    P[0] = msg.pose.pose.pose.position.x;
    P[1] = msg.pose.pose.pose.position.y;
    P[2] = msg.pose.pose.pose.position.z;


    Q.coeffs()[0] = msg.pose.pose.pose.orientation.x;  // 设置虚部 x 
    Q.coeffs()[1] = msg.pose.pose.pose.orientation.y;  // 设置虚部 y
    Q.coeffs()[2] = msg.pose.pose.pose.orientation.z;  // 设置虚部 z
    Q.coeffs()[3] = msg.pose.pose.pose.orientation.w;  // 设置实部 w

}

void onMouse(int event, int x, int y, int flags, void* userdata)
{
    if (event == cv::EVENT_LBUTTONDOWN)
    {
        // 输出点击位置处的像素值
        if (!depth_image.empty())
        {
            float depth_value = depth_image_noinf.at<float>(y, x);
            ROS_INFO("Depth value at (%d, %d): %f", y, x, depth_value);
        }
    }
}

int main(int argc, char **argv)
{
    Eigen::Vector3d p(0, 0.0, 1.1);
    std::vector<Eigen::Vector3d> p_list;
    p_list.push_back(p);
    p << 3.0, 0.0, 1.1;
    p_list.push_back(p);

    
    // 初始化ROS节点
    ros::init(argc, argv, "test");
    
    // 创建节点句柄
    ros::NodeHandle nh;
    traj = csf.tg.trajGeneration(p_list, ros::Time::now());
    Bpc_pub = nh.advertise<sensor_msgs::PointCloud2>("/barrier_pointcloud", 1);
    Hpc_pub = nh.advertise<sensor_msgs::PointCloud2>("/hole_pointcloud", 1);
    trajPath_pub  = nh.advertise<nav_msgs::Path>("trajectory", 10);

    trajVisual_.reset(new PlanningVisualization(nh));

    ros::Subscriber sub = nh.subscribe("/xv_sdk/xv_dev/tof_camera/image", 1, depthImageCallback);

    ros::Subscriber odom_sub = nh.subscribe("/fusion/odom", 1, odomCallback);

    ros::Timer timer = nh.createTimer(ros::Duration(0.1), timerCallback);
    timer_ptr = &timer;


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