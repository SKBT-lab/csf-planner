#include <visual_utils/planning_visualization.hpp>

using std::cout;
using std::endl;


PlanningVisualization::PlanningVisualization(ros::NodeHandle &nh){
    traj_pub        = nh.advertise<visualization_msgs::Marker>("mini_snap_traj", 1);
    waypoints_pub   = nh.advertise<visualization_msgs::Marker>("waypoints", 1);   
    pillars_pub     = nh.advertise<visualization_msgs::Marker>("pillars", 1);   
    Hard_waypoints_pub = nh.advertise<visualization_msgs::Marker>("Hard_waypoints", 1);;
    Soft_waypoints_pub= nh.advertise<visualization_msgs::Marker>("Soft_waypoints", 1);;

}

void PlanningVisualization::displayMarkerList(  ros::Publisher &pub,
                                                const vector<Eigen::Vector3d> &list, 
                                                double scale,
                                                Eigen::Vector4d color, 
                                                int id, 
                                                int type)
{
    visualization_msgs::Marker sphere, line_strip, pillar;
    pillar.header.frame_id  = sphere.header.frame_id = line_strip.header.frame_id = "world";
    pillar.header.stamp     = sphere.header.stamp    = line_strip.header.stamp    = ros::Time::now();
    sphere.type     = visualization_msgs::Marker::SPHERE_LIST;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    pillar.type     = visualization_msgs::Marker::CUBE_LIST;
    pillar.action = sphere.action = line_strip.action = visualization_msgs::Marker::ADD;
    sphere.id       = id;
    line_strip.id   = id + 1000;
    pillar.id       = id + 2000;

    pillar.pose.orientation.w = sphere.pose.orientation.w = line_strip.pose.orientation.w = 1.0;
    pillar.color.r = sphere.color.r = line_strip.color.r = color(0);
    pillar.color.g = sphere.color.g = line_strip.color.g = color(1);
    pillar.color.b = sphere.color.b = line_strip.color.b = color(2);
    pillar.color.a = sphere.color.a = line_strip.color.a = color(3) > 1e-5 ? color(3) : 1.0;
    sphere.scale.x = scale;
    sphere.scale.y = scale;
    sphere.scale.z = scale;

    pillar.scale.x = scale;
    pillar.scale.y = scale;
    pillar.scale.z = 3;    

    line_strip.scale.x = scale / 2;
    geometry_msgs::Point pt;
    ros::Rate loop_rate(1);
    for (int i = 0; i < int(list.size()); i++)
    {
        pt.x = list[i](0);
        pt.y = list[i](1);
        pt.z = list[i](2);
        sphere.points.push_back(pt);
        pillar.points.push_back(pt);
        line_strip.points.push_back(pt);
    }
    if (type == visualization_msgs::Marker::CUBE_LIST) {
            loop_rate.sleep();
            pub.publish(pillar);
    }
    //if (show_sphere) pub.publish(sphere);
    if (type == visualization_msgs::Marker::LINE_STRIP) pub.publish(line_strip);
    if (type == visualization_msgs::Marker::SPHERE_LIST) {
        pub.publish(sphere);
    }
}


void PlanningVisualization::displayTraj(vector<Eigen::Vector3d> list){
    // if (traj_pub.getNumSubscribers() == 0){
    //   return;
    // }
    Eigen::Vector4d color(1, 0, 0, 1);
    displayMarkerList(traj_pub, list, 0.15, color, 0, visualization_msgs::Marker::LINE_STRIP);
}

 void PlanningVisualization::displayWaypoints(vector<Eigen::Vector3d> list, Eigen::Vector4d color){
    displayMarkerList(waypoints_pub, list, 0.15, color, 0, visualization_msgs::Marker::SPHERE_LIST);
 }

 void PlanningVisualization::displaySoftWaypoints(vector<Eigen::Vector3d> list, Eigen::Vector4d color){
    displayMarkerList(Soft_waypoints_pub, list, 0.15, color, 0, visualization_msgs::Marker::SPHERE_LIST);
 }

 void PlanningVisualization::displayHardWaypoints(vector<Eigen::Vector3d> list, Eigen::Vector4d color){
    displayMarkerList(Hard_waypoints_pub, list, 0.15, color, 0, visualization_msgs::Marker::SPHERE_LIST);
 }

 void PlanningVisualization::displayPillars(vector<Eigen::Vector3d> list){
    Eigen::Vector4d color(0, 0, 0, 0);
    displayMarkerList(pillars_pub, list, 0.15, color, 0, visualization_msgs::Marker::CUBE_LIST);
 }





