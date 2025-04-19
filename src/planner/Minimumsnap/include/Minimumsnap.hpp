#ifndef _MINIMUMSNAP_HPP_
#define _MINIMUMSNAP_HPP_

#include <stdio.h>
#include <iostream>
#include <cmath>
#include <Eigen/Eigen>
#include <visual_utils/planning_visualization.hpp>
#include <ros/ros.h>
#include <ros/console.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>
#include <chrono>
#include <thread>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

#include <yaml-cpp/yaml.h>

using namespace Eigen;

struct Soft_waypoints{
    std::vector<Eigen::Vector3d> points;
    std::vector<double> times;
    bool enable = false;
};
struct MiniSnapTraj{
    ros::Time       start_time;         //轨迹起始时间
    int             traj_nums;          //分段轨迹数量
    double          time_duration;      //轨迹总时间
    std::vector<double> poly_time;          //每段轨迹的结束时间点，相对轨迹起始点来说
    std::vector<Eigen::VectorXd> poly_coeff;         //多项式轨迹系数矩阵
    std::vector<Eigen::Vector3d> waypoints;
    Eigen::Vector3d hard_waypoints;
    int hard_waypoints_ID;
    double hard_waypoints_time;
    Soft_waypoints soft_waypoints;
    bool replan_flag;

    MiniSnapTraj(){};
    MiniSnapTraj(ros::Time _start_time, std::vector<double> _poly_time, std::vector<Eigen::VectorXd> _poly_coeff, std::vector<Eigen::Vector3d> wps){
        start_time  = _start_time;
        traj_nums   = _poly_time.size();
        poly_time.resize(traj_nums);
        waypoints = wps;
        double sum = 0;
        for (int i = 0; i < _poly_time.size(); i++){
            sum += _poly_time[i];
            poly_time[i] = sum;
        }
        poly_coeff  = _poly_coeff;
        time_duration = poly_time[traj_nums - 1];
        hard_waypoints_ID = -1;
        hard_waypoints_time = -1.0;
        replan_flag = true;
    };
    ~MiniSnapTraj(){};
};

struct Traj_info{
    Eigen::Vector3d position;
    Eigen::Vector3d velocity;
    Eigen::Vector3d acceleration;

    Traj_info(){}
    Traj_info(Eigen::Vector3d& p, Eigen::Vector3d& v, Eigen::Vector3d& a)
    : position(p), velocity(v), acceleration(a) {}
};


class TRAJECTORY_GENERATOR  
{
private:

    double _vis_traj_width;
    double _Vel, _Acc;
    int _dev_order, _min_order;
    int _poly_num1D, _poly_order;
    int Ksoft, Ktime;
    double h, dt;
    int max_iters;
    double start_vel_lim;
    double insert_time_thresh;

    Vector3d _startPos  = Vector3d::Zero();
    Vector3d _startVel  = Vector3d::Zero();
    Vector3d _startAcc  = Vector3d::Zero();
    Vector3d _endVel    = Vector3d::Zero();
    Vector3d _endAcc    = Vector3d::Zero();

    std::vector<Eigen::Vector3d> traj;
    // double _qp_cost;
    Eigen::MatrixXd _Q;
    Eigen::MatrixXd _M;
    Eigen::MatrixXd _Ct;

    Eigen::MatrixXd _G;
    Eigen::MatrixXd _S;
    
    Eigen::VectorXd _Px, _Py, _Pz;

    bool height_fix = true;
    int height_fix_num;
    double fixed_height = 1.5;


    Eigen::MatrixXd getQ(const Eigen::VectorXd &Time, 
                         const int seg_index);

    Eigen::MatrixXd getM(const Eigen::VectorXd &Time, 
                         const int seg_index);

    Eigen::MatrixXd getCt(const int seg_num);

    void get_G_S(const Soft_waypoints& Softwps, const std::vector<double>& Time);

    Eigen::VectorXd closedFormCalCoeff1D(const Eigen::VectorXd &WayPoints1D,
                                         const Eigen::VectorXd &StartState1D,
                                         const Eigen::VectorXd &EndState1D,
                                         const int seg_num);
    Eigen::VectorXd closedFormCalCoeff1D(const Eigen::VectorXd &WayPoints1D,
                                                            const Eigen::VectorXd &StartState1D,
                                                            const Eigen::VectorXd &EndState1D,
                                                            const Eigen::VectorXd &S1D,
                                                            const int seg_num);
    double closedFormCalLoss(const Eigen::VectorXd &WayPoints1D,
                                                            const Eigen::VectorXd &StartState1D,
                                                            const Eigen::VectorXd &EndState1D,
                                                            const int seg_num);

public:
    YAML::Node config;
    TRAJECTORY_GENERATOR(){
        config = YAML::LoadFile("/home/skbt2/SKBT_Drone/src/planner/Minimumsnap/config/minimumsnap_Para.yaml");
        _Vel = config["Vel"].as<double>();
        _Acc = config["Acc"].as<double>();
        _dev_order = config["dev_order"].as<int>();
        _min_order = config["min_order"].as<int>();
        _vis_traj_width = config["vis_traj_width"].as<double>();
        Ksoft = config["K_soft"].as<int>();
        Ktime = config["K_time"].as<int>();
        max_iters = config["Max_Iter"].as<int>();
        h = config["h_"].as<double>();
        dt = config["dt_"].as<double>();
        start_vel_lim = config["start_vel_lim"].as<double>();
        insert_time_thresh = config["insert_time_thresh"].as<double>();
        fixed_height = config["fixed_height"].as<double>();
        std::cout << fixed_height << std::endl;
        height_fix_num = config["height_fix"].as<int>();
        std::cout << height_fix_num << std::endl;
        if(height_fix_num == 1){
            height_fix = true;
        }
        else{
            height_fix = false;
        }
        _poly_num1D = 2 * _dev_order;
        _poly_order = _poly_num1D - 1;
    };

    ~TRAJECTORY_GENERATOR(){};

    std::vector<Eigen::VectorXd> PolyQPGeneration(
        const std::vector<Eigen::Vector3d> &Path_,
        const std::vector<Eigen::Vector3d> &Vel_,
        const std::vector<Eigen::Vector3d> &Acc_,
        const std::vector<double>& Time_,
        const Soft_waypoints& Softwps_ = Soft_waypoints{});

    int Factorial(int x);

    std::vector<double> timeAllocation(std::vector<Eigen::Vector3d>& Path, double vel = -1, Eigen::Vector3d start_vel_ = Vector3d::Zero());
    double GetLoss_withT(const Eigen::MatrixXd &StartState, // waypoints coordinates (3d)
                        const Eigen::MatrixXd &EndState,  // boundary velocity
                        const Eigen::MatrixXd &Path,
                        const Eigen::VectorXd& Time);
    std::vector<double> timeRefine(const std::vector<Eigen::Vector3d> &Path_,
                                                    const std::vector<Eigen::Vector3d> &Vel_,
                                                    const std::vector<Eigen::Vector3d> &Acc_,
                                                    const std::vector<double>& Init_time);
    Traj_info getTrajInfo(const MiniSnapTraj& traj, double time);
    std::pair<int, bool> getTrajID(const MiniSnapTraj& traj, double time);
    MiniSnapTraj trajGeneration(std::vector<Eigen::Vector3d>& path, ros::Time Time_start, double vel_desire = -1, Eigen::Vector3d startVel = Vector3d::Zero(), Eigen::Vector3d endVel = Vector3d::Zero(), Eigen::Vector3d startAcc = Vector3d::Zero(), Eigen::Vector3d endAcc = Vector3d::Zero(), const Soft_waypoints& Softwps = Soft_waypoints{});
    std::pair<MiniSnapTraj, bool> waypointInsert(MiniSnapTraj Traj, Eigen::Vector3d& point_insert, double time_insert, double time_start, int& hwps_num, Soft_waypoints Softwps = Soft_waypoints{});  //time_insert仅用于确定插入点在原路标点序列中的次序位置，而非用于时间分配

    std::pair<double, double> calculate_yaw(Traj_info& t_info);
};

#endif // _MINIMUMSNAP_HPP_