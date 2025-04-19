#ifndef _PLANNING_VISUALIZATION_HPP_
#define _PLANNING_VISUALIZATION_HPP_


#include <Eigen/Eigen>
#include <algorithm>
#include <iostream>
#include <ros/ros.h>
#include <vector>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <stdlib.h>

using std::vector;

class PlanningVisualization{
private:

    //可视化显示类型
    enum VISUAL_TYPE{
        LINE = 0,
        SPHERE,
        PILLAR,
    };

    ros::Publisher traj_pub;
    ros::Publisher waypoints_pub;
    ros::Publisher pillars_pub;
    ros::Publisher Hard_waypoints_pub;
    ros::Publisher Soft_waypoints_pub;

public:
    typedef std::shared_ptr<PlanningVisualization> Ptr;
    PlanningVisualization(/* args */) {}
    ~PlanningVisualization() {}
    //构造函数
    PlanningVisualization(ros::NodeHandle &nh);

    void displayMarkerList( ros::Publisher &pub, 
                            const vector<Eigen::Vector3d> &list, 
                            double scale,
                            Eigen::Vector4d color, 
                            int id,  
                            int type);

    void displayTraj(vector<Eigen::Vector3d> list);
    void displayWaypoints(vector<Eigen::Vector3d> list, Eigen::Vector4d color = Eigen::Vector4d(255, 109, 21, 0));
    void displayPillars(vector<Eigen::Vector3d> list);
    void displaySoftWaypoints(vector<Eigen::Vector3d> list, Eigen::Vector4d color = Eigen::Vector4d(1, 1, 0, 0));
    void displayHardWaypoints(vector<Eigen::Vector3d> list, Eigen::Vector4d color = Eigen::Vector4d(1, 0, 0, 0));

};




#endif