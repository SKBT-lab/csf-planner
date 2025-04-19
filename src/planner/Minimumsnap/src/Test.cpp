#include "Minimumsnap.hpp"

nav_msgs::Path trajPath;
ros::Publisher trajPath_pub;
ros::Publisher pose_pub;
TRAJECTORY_GENERATOR XY;
PlanningVisualization::Ptr trajVisual_;
bool insert_Flag;

double clamp(double value, double min, double max) {
    return std::max(min, std::min(value, max));
}


std::pair<double, double> calculate_yaw(Traj_info& t_info){
    Vector3d pos = t_info.position;
    Vector3d vel = t_info.velocity;
    Vector3d acc = t_info.acceleration;

    double dx = vel.x();
    double dy = vel.y();

    double ddx = acc.x();
    double ddy = acc.y();

    double yaw = atan(dy / dx);
    double dyaw =  ((ddy - dy * ddx) / dx) / (1 + (dy / dx) * (dy / dx));
        
    if(dx < 0){               //由于反正切的值域是-PI/2到PI/2,而dx<0的部分在该范围之外，故额外叠加个PI
        yaw = yaw - M_PI;
    }
    
    std::pair<double, double> yaw_dyaw;
    yaw_dyaw.first = yaw;
    yaw_dyaw.second = dyaw;
    return yaw_dyaw;  
}

void getVisual(MiniSnapTraj& trajectory, MatrixXd wps){
    std::vector<Eigen::Vector3d> waypoint_list;
    std::vector<Eigen::Vector3d> traj;

    for (int i = 0; i < wps.rows(); ++i) {
        Eigen::Vector3d vec = wps.row(i);
        waypoint_list.push_back(vec);
    }
    trajVisual_->displayWaypoints(waypoint_list);

    double traj_len = 0.0;
    int count = 1;
    Eigen::Vector3d cur, pre, vel;
    cur.setZero();
    pre.setZero();
    geometry_msgs::PoseStamped poses;
    trajPath.header.frame_id = poses.header.frame_id = "world";
    trajPath.header.stamp    = poses.header.stamp    = ros::Time::now();
    double yaw = 0.0;
    poses.pose.orientation   = tf::createQuaternionMsgFromYaw(yaw);

    ros::Rate loop_rate(1000);
    for(double t = 0.0; t < trajectory.time_duration; t += 0.001, count++)   // go through each segment
    {   
        auto info = XY.getTrajInfo(trajectory, t);
        cur = info.position;
        vel = info.velocity;
        auto yaw_dyaw = calculate_yaw(info);
        yaw = yaw_dyaw.first;
        std::cout << vel.x() <<" " << vel.y() << " " << vel.z() << std::endl;
        std::cout << yaw_dyaw.second << std::endl;
        poses.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
        poses.pose.position.x = cur[0];
        poses.pose.position.y = cur[1];
        poses.pose.position.z = cur[2];
        trajPath.poses.push_back(poses);
        traj.push_back(cur);
        // std::cout << cur << std::endl;
        // std::cout << '###############' << std::endl;
        trajPath_pub.publish(trajPath);
        pose_pub.publish(poses);
        if (count % 1000 == 0) traj_len += (pre - cur).norm();
        pre = cur;
        
        loop_rate.sleep();
    }
    trajVisual_->displayTraj(traj);  
}
int main(int argc, char** argv){
    double dis_1 = 1.5;
    double dis_1to2 = 2.0;
    double r = 1.0;
    double h = 1.5;
    int dir = - 1;
    insert_Flag = false;
    ros::init(argc, argv, "Test_node");
    ros::NodeHandle nh("~");
    trajPath_pub  = nh.advertise<nav_msgs::Path>("trajectory", 10);
    pose_pub = nh.advertise<geometry_msgs::PoseStamped>("XYpose", 10);
    trajVisual_.reset(new PlanningVisualization(nh));
    MatrixXd waypoints(9,3);
    
    waypoints << 0, 0, h,
                 dis_1, dir * r, h,
                 dis_1 + dis_1to2 / 2, 0, h,
                 dis_1 + dis_1to2, - dir * r, h,
                 dis_1 + dis_1to2 + r, 0, h,
                 dis_1 + dis_1to2, dir * r, h,
                 dis_1 + dis_1to2 / 2, 0, h,
                 dis_1, - dir * r, h,
                 0, 0, h;

    // 遍历每一行，将其转换为 Eigen::Vector3d 并添加到 vectorOfRows 中
    MiniSnapTraj traj;
    traj = XY.trajGeneration(waypoints, ros::Time::now(), -1, Eigen::Vector3d(0.0, dir * 0.5, 0), Eigen::Vector3d(0.0, dir * 0.5, 0));
    std::this_thread::sleep_for(std::chrono::seconds(3));
    getVisual(traj, waypoints);
    ros::spin();
    return 0;
}

