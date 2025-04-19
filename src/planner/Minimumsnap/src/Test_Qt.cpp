#include <ros/ros.h>
#include <interactive_markers/interactive_marker_server.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include "Minimumsnap.hpp"

Eigen::Vector3d wp1(20.1894, 0, 1.74196);
Eigen::Vector3d wp2(24.3049, -0.704177, 2.11733);
Eigen::Vector3d wp3(50, 0.0, 1.8);
Eigen::Vector3d wp1_3;
// Eigen::Vector3d vel(0.0500125, 0.00018275, 0.000710986);
// Eigen::Vector3d acc(0.0825418, 0.000301615, 0.00117343);

Eigen::Vector3d vel(0, 0.0, 0.0);
Eigen::Vector3d acc(0.0, 0.0, 0.0);

// Eigen::Vector3d wp4(2.0, 4.0, 0.0);
// Eigen::Vector3d wp5(2.0, 6.0, 1.0);
Eigen::Vector3d wp_in(0.0, 0.0, 0.0);
Eigen::Vector3d Swp1(0.0, 2.0, 0.0);
Eigen::Vector3d Swp2(0.0, 4.0, 0.0);


Eigen::MatrixXd waypoints;
TRAJECTORY_GENERATOR XY;
MiniSnapTraj traj;
nav_msgs::Path trajPath;
ros::Publisher trajPath_pub;

std::vector<Eigen::Vector3d> Wps_list;

PlanningVisualization::Ptr trajVisual_;

std::vector<Eigen::Vector3d> P_list;

bool insert_Flag;
double insert_t;

std::shared_ptr<interactive_markers::InteractiveMarkerServer> server;

Soft_waypoints swps;    

Eigen::MatrixXd vectorToMatrix(const std::vector<Eigen::Vector3d>& vec) {
    int n = vec.size();
    Eigen::MatrixXd mat(n, 3);

    for (int i = 0; i < n; ++i) {
        mat.row(i) = vec[i];
    }

    return mat;
}

void GetWaypoints(){
    Wps_list.clear();
    Wps_list.push_back(wp1);
    Wps_list.push_back(wp2);
    // Wps_list.push_back(wp1_3);
    Wps_list.push_back(wp3);
    // Wps_list.push_back(wp4);
    // Wps_list.push_back(wp5);
    // waypoints = vectorToMatrix(Wps_list);
}

void getVisual(MiniSnapTraj& trajectory, std::vector<Eigen::Vector3d>& Add_points){
    trajVisual_->displayWaypoints(Add_points);
    double traj_len = 0.0;
    int count = 1;
    Eigen::Vector3d cur, vel;
    cur.setZero();
    geometry_msgs::PoseStamped poses;
    trajPath.header.frame_id = poses.header.frame_id = "world";
    trajPath.header.stamp    = poses.header.stamp    = ros::Time::now();
    double yaw = 0.0;
    poses.pose.orientation   = tf::createQuaternionMsgFromYaw(yaw);

    ros::Rate loop_rate(1000);
    for(double t = 0.0; t < trajectory.time_duration; t += 0.1, count++)   // go through each segment
    {   
        auto info = XY.getTrajInfo(trajectory, t);
        cur = info.position;
        vel = info.velocity;
        poses.pose.position.x = cur[0];
        poses.pose.position.y = cur[1];
        poses.pose.position.z = cur[2];
        trajPath.poses.push_back(poses);
        // std::cout << cur << std::endl;getTrajID
        // std::cout << '###############' << std::endl;
        // loop_rate.sleep();
    } 
    trajPath_pub.publish(trajPath);
}
void processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
void createInteractiveMarker(interactive_markers::InteractiveMarkerServer &server,
                             const std::string &name, const std::string &description,
                             double x, double y, double z, double r, double g, double b) {
    // 创建交互式标记
    visualization_msgs::InteractiveMarker int_marker;
    int_marker.header.frame_id = "world";
    int_marker.header.stamp = ros::Time::now();
    int_marker.name = name;
    int_marker.description = description;
    int_marker.pose.position.x = x;
    int_marker.pose.position.y = y;
    int_marker.pose.position.z = z;

    // 创建一个实际可见的 Marker
    visualization_msgs::Marker marker;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.scale.x = marker.scale.y = marker.scale.z = 0.2;
    marker.color.r = r;  // 红色
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = 1.0;  // 不透明

    // 将 Marker 添加到交互式标记
    visualization_msgs::InteractiveMarkerControl control;
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_3D;
    control.always_visible = true;
    control.markers.push_back(marker);
    int_marker.controls.push_back(control);

    // 将交互式标记添加到服务器
    server.insert(int_marker, &processFeedback);
}
void processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback) {
    ROS_INFO_STREAM("Marker " << feedback->marker_name << " is now at: "
                    << feedback->pose.position.x << ", "
                    << feedback->pose.position.y << ", "
                    << feedback->pose.position.z);
    if(feedback->marker_name == "waypoint_1"){
        wp1 << feedback->pose.position.x, feedback->pose.position.y, feedback->pose.position.z;
        GetWaypoints();
        traj = XY.trajGeneration(Wps_list, ros::Time::now(), -1, vel, Eigen::Vector3d(0.0, 0.0, 0.0), acc, Eigen::Vector3d(0.0, 0.0, 0.0));
    }
    else if(feedback->marker_name == "waypoint_2"){
        wp2 << feedback->pose.position.x, feedback->pose.position.y, feedback->pose.position.z;
        GetWaypoints();
        traj = XY.trajGeneration(Wps_list, ros::Time::now(), -1, vel, Eigen::Vector3d(0.0, 0.0, 0.0), acc, Eigen::Vector3d(0.0, 0.0, 0.0));
    }
    else if(feedback->marker_name == "waypoint_3"){
        wp3 << feedback->pose.position.x, feedback->pose.position.y, feedback->pose.position.z;
        GetWaypoints();
        traj = XY.trajGeneration(Wps_list, ros::Time::now(), -1, vel, Eigen::Vector3d(0.0, 0.0, 0.0), acc, Eigen::Vector3d(0.0, 0.0, 0.0));
    }
    // if(feedback->marker_name == "waypoint_4"){
    //     wp4 << feedback->pose.position.x, feedback->pose.position.y, feedback->pose.position.z;
    // }
    // // if(feedback->marker_name == "waypoint_5"){
    //     wp5 << feedback->pose.position.x, feedback->pose.position.y, feedback->pose.position.z;
    // }

    else if(feedback->marker_name == "waypoint_insert"){
        wp_in << feedback->pose.position.x, feedback->pose.position.y, feedback->pose.position.z;
    }

    else if(feedback->marker_name == "Softwaypoint_1"){
        Swp1<< feedback->pose.position.x, feedback->pose.position.y, feedback->pose.position.z;
        swps.points.clear();
        swps.times.clear();
        swps.points.push_back(Swp1);
        // swps.points.push_back(Swp2);
        double t1 = 0.5 * traj.time_duration;
        double t2 = 0.25 * traj.time_duration + 0.5;
        swps.times.push_back(t1);
        // swps.times.push_back(t2);
        swps.enable = true;
        // GetWaypoints();
        traj = XY.trajGeneration(Wps_list, ros::Time::now(), -1, Vector3d::Zero(), Vector3d::Zero(), Vector3d::Zero(), Vector3d::Zero(), swps);
        P_list.clear();
        Traj_info t_info = XY.getTrajInfo(traj, t1);
        P_list.push_back(t_info.position);
        t_info = XY.getTrajInfo(traj, t2);
        P_list.push_back(t_info.position);
    }
    else if(feedback->marker_name == "Softwaypoint_2"){
        Swp2<< feedback->pose.position.x, feedback->pose.position.y, feedback->pose.position.z;
        swps.points.clear();
        swps.times.clear();
        swps.points.push_back(Swp1);
        swps.points.push_back(Swp2);
        double t1 = 0.25 * traj.time_duration;
        double t2 = 0.25 * traj.time_duration + 0.5;
        swps.times.push_back(t1);
        swps.times.push_back(t2);
        swps.enable = true;
        // GetWaypoints();
        traj = XY.trajGeneration(Wps_list, ros::Time::now(), -1, Vector3d::Zero(), Vector3d::Zero(), Vector3d::Zero(), Vector3d::Zero(), swps);
        P_list.clear();
        Traj_info t_info = XY.getTrajInfo(traj, t1);
        P_list.push_back(t_info.position);
        t_info = XY.getTrajInfo(traj, t2);
        P_list.push_back(t_info.position);
    }
    
    if(wp1.z() > 3.0){  
        if(!insert_Flag){
            Traj_info insert_info = XY.getTrajInfo(traj, 0.5 * traj.time_duration);
            Eigen::Vector3d insert_pos = insert_info.position;
            createInteractiveMarker(*server, "waypoint_insert", "Waypoint insert", insert_pos[0], insert_pos[1], insert_pos[2], 0, 1, 0);
            server->applyChanges();
            wp_in = insert_pos;
            insert_Flag = true;
        }
        std::pair<MiniSnapTraj, bool> traj_info;

        traj_info = XY.waypointInsert(traj, wp_in, 0.5 * traj.time_duration, 0,1.0);
        traj = traj_info.first;
    }
    trajPath.poses.clear();
    getVisual(traj, P_list);

}


int main(int argc, char** argv) {
    
    ros::init(argc, argv, "interactive_waypoint");
    
    ros::NodeHandle nh("~");
    trajPath_pub  = nh.advertise<nav_msgs::Path>("trajectory", 10);
    trajVisual_.reset(new PlanningVisualization(nh));
    insert_Flag = false;
    // vel = 0.5 * vel.normalized();
    wp1_3 = (wp1 + wp3) / 2;
    // interactive_markers::InteractiveMarkerServer server("waypoint_marker");
     // 添加多个航点
    
    server = std::make_shared<interactive_markers::InteractiveMarkerServer>("waypoint_marker");
    createInteractiveMarker(*server, "waypoint_1", "Waypoint 1", wp1[0], wp1[1], wp1[2], 1, 0, 0);
    createInteractiveMarker(*server, "waypoint_2", "Waypoint 2", wp2[0], wp2[1], wp2[2], 1, 0, 0);
    createInteractiveMarker(*server, "waypoint_3", "Waypoint 3", wp3[0], wp3[1], wp3[2], 1, 0, 0);
    // createInteractiveMarker(*server, "waypoint_4", "Waypoint 4", wp4[0], wp4[1], wp4[2], 1, 0, 0);
    // createInteractiveMarker(server, "waypoint_5", "Waypoint 5", wp5[0], wp5[1], wp5[2]);
    
    createInteractiveMarker(*server, "Softwaypoint_1", "Softwaypoint 1", Swp1[0], Swp1[1], Swp1[2], 0, 0, 1);
    createInteractiveMarker(*server, "Softwaypoint_2", "Softwaypoint 2", Swp2[0], Swp2[1], Swp2[2], 0, 0, 1);
    server->applyChanges();
    ros::spin();
}