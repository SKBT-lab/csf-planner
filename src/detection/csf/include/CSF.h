#include <iostream>
#include <limits>
#include <chrono>
#include <vector>
#include <queue>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include "odom_fusion/OdomStamp.h"
#include <yaml-cpp/yaml.h>

#include "Minimumsnap.hpp"

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>

#include <csf/Cloth.h>
#include <csf/Waypoints.h>
#include <csf/Trajectory.h>


using namespace Eigen;

enum Sliding_mode {
    FIND_POINT,  // 0
    FILL_HOLE,  // 1
};

struct Waypoints{
    Eigen::Vector3d hard_waypoint;
    std::vector<Eigen::Vector3d> soft_waypoints;
    Waypoints(){};
    Waypoints(Eigen::Vector3d& hwps, std::vector<Eigen::Vector3d>& swps){
        hard_waypoint = hwps;
        soft_waypoints = swps;
    };
    ~Waypoints(){};
};

struct Collision_info{
    Eigen::Vector3d UV1;
    Eigen::Vector3d n;
    double t;
    Collision_info(){};
    Collision_info(Eigen::Vector3d& uv1, Eigen::Vector3d& n_, double t_){
        UV1 = uv1;
        n = n_;
        t = t_;
    };
    ~Collision_info(){};
};

class C_S_FILTER{
public:
    
    int width_ori;       //原始深度图的宽和高
    int height_ori;
    cv::Mat CV_Depth_ori;
    
    int width;           //稀疏后的宽和高
    int height;

    double Min_depth;

    double m;            //布料节点质量
    double dt;           //时间步长

    double Kt;           //Traction劲度系数
    double Ks;           //Shear劲度系数
    double Kf;           //Flexion劲度系数

    double K_drag;       //阻力系数

    double Kspars;          //稀疏系数

    int Cnt_Area_Threshold;

    double Kerase;       //地面擦除系数

    double thresh;
    double init_vel;

    int MaxIterations;

    double va, vt;

    double coll_thresh;
    double coll_dis;

    double safetybox_w;
    double safetybox_h;

    double safetybox_pixelw;
    double safetybox_pixelh;

    double K_Coll_tolerance;

    int Soft_points_num;
    double Soft_seg_len;
    double Soft_enable_thresh;

    double guidance_enable_thresh;

    int slid_num_thresh;

    int update_num_thresh;

    int commu_mode;

    double time_forward;

    Eigen::Vector3d safetybox_LU;
    Eigen::Vector3d safetybox_RU;
    Eigen::Vector3d safetybox_LD;
    Eigen::Vector3d safetybox_RD;

    TRAJECTORY_GENERATOR tg;

    Eigen::Matrix3d  Intri_Matrix;           //相机内参
    Eigen::Matrix3d  Intri_Matrix_Inverse;   //内参的逆

    Eigen::Matrix3d Rotation_Matrix_CtoB;  //相机系相对于Body系的旋转矩阵
    Eigen::Quaterniond q_CtoB;             //相机系相对于Body系的四元数
    Eigen::Quaterniond q_BtoC; 

    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> Depth_Cloth;
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> Depth_Cloth0;    //用于备份Depth_Cloth以进行帧间比较
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> Vel_Cloth;

    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> Depth_Image; 
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> Coll_Distance;    //存储布和每个实际深度的距离，用于碰撞检测
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> Coll_Flag;
    Eigen::MatrixXd Coll_Flag_exp;

    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> UtD_Cloth;
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> DtU_Cloth;
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> LtR_Cloth;
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> RtL_Cloth;

    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> LUtRD_Cloth;
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> RUtLD_Cloth;
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> RDtLU_Cloth;
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> LDtRU_Cloth;

    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> UtD2_Cloth;
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> DtU2_Cloth;
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> LtR2_Cloth;
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> RtL2_Cloth;

    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> U_Traction;
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> D_Traction;
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> L_Traction;
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> R_Traction;

    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> LU_Shear;
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> RU_Shear;
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> RD_Shear;
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> LD_Shear;

    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> U2_Flexion;
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> D2_Flexion;
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> L2_Flexion;
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> R2_Flexion;

    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> Internal_Force;
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> External_Force;
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> Force_Matrix;

    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> U_matrix;
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> D_matrix;
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> L_matrix;
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> R_matrix;

    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> U2_matrix;
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> D2_matrix;
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> L2_matrix;
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> R2_matrix;

    Eigen::MatrixXd Row_Identity;
    Eigen::MatrixXd Col_Identity;

    Eigen::MatrixXd Row_trans_matrix;
    Eigen::MatrixXd Col_trans_matrix;

    Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic> Spread_Flag;

    Eigen::Vector3d PC_bias;

    cv::Mat Barrier_Mask;
    cv::Mat Barrier_Mask_OriSize;
    cv::Mat Access_Mask;
    cv::Mat Hole_Mask;
    cv::Mat CV_Depth;
    cv::Mat CV_Cloth;
    cv::Mat Safety_Filter; //用于擦掉上下边缘区域
    cv::Mat CV_Cloth_Mask;

    cv::Size New_Size;
    cv::Size Ori_Size;

    cv::Mat element;   //用于闭操作

    cv::Mat sld_p_img;

    cv::Mat Filter_kernel;
    
    std::vector<std::vector<cv::Point>> Hole_Cnts;
    std::vector<cv::Point2f> Hole_Centers;
    std::vector<Eigen::Vector3d> Hole_3Dpos;

    std::vector<std::vector<cv::Point>> Barrier_Cnts;
    std::vector<cv::Point2f> Barrier_Centers;
    std::vector<Eigen::Vector3d> Barrier_3Dpos;

    std::vector<std::vector<Eigen::Vector3d>> Swps_list;
    std::vector<Eigen::Vector4d> Swps_sum;

    const std::vector<std::pair<int, int>> directions = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}};

    void Input_Depth(cv::Mat Depth);
    void Reset_Cloth();
    void Cal_force();
    void Cloth_Collision_Detection();
    void Update_Cloth();
    void Shot();
    void Barrier_Check();
    void Clear_BG();
    void Find_Hole(const Eigen::Quaterniond& q = Eigen::Quaterniond::Identity(), const Eigen::Vector3d& p = Eigen::Vector3d::Zero());
    void Find_Barrier(const Eigen::Quaterniond& q = Eigen::Quaterniond::Identity(), const Eigen::Vector3d& p = Eigen::Vector3d::Zero());
    void Show_Result(char X);
    void Cloth_Visualization();
    Eigen::MatrixXd Process_Coll_Flag();
    void Point_Protect(cv::Point& point);
    //Eigen::Vector3d Get_3Dpos_Body(cv::Point2f& pix_point, double& depth);
    Eigen::Vector3d Get_3Dpos(const cv::Point2f& pix_point, const double& depth, const Eigen::Quaterniond& q = Eigen::Quaterniond::Identity(), const Eigen::Vector3d& p = Eigen::Vector3d::Zero());
    Waypoints Sliding_Point(cv::Point init_point, const Eigen::Quaterniond &q, const Eigen::Vector3d &p);
    Collision_info Barrier_Collision_Detection(MiniSnapTraj traj, const Eigen::Quaterniond& q, const Eigen::Vector3d& p, double t_start = 0.0);
    Eigen::Vector3d fitLine(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2);
    Eigen::Vector4d fitPlane(const Eigen::Vector3d& p, const Eigen::Vector3d& a);
    void Get_SoftPoints_spread(const cv::Point& p, const cv::Point& init_p, double radius, double d_min, Sliding_mode mode);
    double Cal_node_traj_plane(const Eigen::Vector4d& coeff, MiniSnapTraj traj, double t_start = 0.0);
    Soft_waypoints Get_Softwps_t(const Eigen::Vector3d& n, MiniSnapTraj traj, const std::vector<Eigen::Vector3d>& soft_waypoints, double t_start = 0.0);
    std::pair<MiniSnapTraj, bool> Update_Trajectory(MiniSnapTraj traj, const Eigen::Quaterniond& q, const Eigen::Vector3d& p, int hwps_num);
    bool Point_Check(const cv::Point& point);
    double projectionLength(const Eigen::Vector3d& A, const Eigen::Vector3d& B, const Eigen::Vector3d& C); //计算AB在AC方向上的投影长度
    MiniSnapTraj Replan_Trajectory(MiniSnapTraj traj, const Eigen::Quaterniond& q, const Eigen::Vector3d& p, const Eigen::Vector3d& v);
    void Extract_Cloth_fromMsg(const csf::Cloth::ConstPtr& msg);
    Waypoints Update_Waypoints(MiniSnapTraj traj, const Eigen::Quaterniond& q, const Eigen::Vector3d& p);
    MiniSnapTraj Extract_Traj_fromMsg(const csf::Trajectory::ConstPtr& msg);
    Waypoints Extract_Wps_fromMsg(const csf::Waypoints::ConstPtr& msg);

    YAML::Node config;
    C_S_FILTER(int W, int H): width_ori(W), height_ori(H) {
        config = YAML::LoadFile("/home/skbt2/airsim_ros/src/detection/csf/config/Cloth.yaml");
        m = config["m"].as<double>();
        Kt = config["Kt"].as<double>();
        Ks = config["Ks"].as<double>();
        Kf = config["Kf"].as<double>();
        Kspars = config["Kspars"].as<double>();
        Kerase = config["Kerase"].as<double>();
        dt = config["dt"].as<double>();
        MaxIterations = config["MaxIterations"].as<int>();
        thresh = config["converg_thresh"].as<double>();
        K_drag = config["K_drag"].as<double>();
        init_vel = config["init_vel"].as<double>();
        va = config["va"].as<double>();
        vt = config["vt"].as<double>();

        coll_thresh = config["coll_thresh"].as<double>();
        coll_dis = config["coll_dis"].as<double>();

        safetybox_w = config["safetybox_w"].as<double>();
        safetybox_h = config["safetybox_h"].as<double>();

        K_Coll_tolerance = config["K_Coll_tolerance"].as<double>();

        Soft_points_num = config["Soft_points_num"].as<int>();
        Soft_enable_thresh = config["Soft_enable_thresh"].as<double>();

        guidance_enable_thresh = config["guidance_enable_thresh"].as<double>();

        slid_num_thresh = config["slid_num_thresh"].as<int>();

        update_num_thresh = config["update_num_thresh"].as<int>();
        time_forward = config["time_forward"].as<double>();

        commu_mode = config["commu_mode"].as<int>();

        PC_bias << config["PC_bias"][0].as<double>(), config["PC_bias"][1].as<double>(), config["PC_bias"][2].as<double>();

        safetybox_LU << 0.0, 0.5 * safetybox_w, 0.5 * safetybox_h;
        safetybox_LD << 0.0, 0.5 * safetybox_w, -0.5 * safetybox_h;
        safetybox_RU << 0.0, -0.5 * safetybox_w, 0.5 * safetybox_h;
        safetybox_RD << 0.0, -0.5 * safetybox_w, -0.5 * safetybox_h;

        // 读取矩阵元素并赋值给 Eigen 矩阵
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                Intri_Matrix(i, j) = config["Intri_Matrix"][i][j].as<double>();
            }
        }
        std::cout << Intri_Matrix << std::endl;

    
        // Intri_Matrix << 516.7852783203125, 0.0, 320.07366943359375,
        //                 0.0, 516.7852783203125, 239.31646728515625,
        //                 0.0, 0.0, 1.0;
        Intri_Matrix_Inverse = Intri_Matrix.inverse();

        Rotation_Matrix_CtoB << 0, 0, 1,
                                -1, 0, 0,
                                0, -1, 0;

        Eigen::Quaterniond q(Rotation_Matrix_CtoB);
        q_CtoB = q;
        q_BtoC = q.inverse();

        Cnt_Area_Threshold = int((width_ori / 8) * (height_ori / 8)) ;

        Ori_Size.width = W;
        Ori_Size.height = H;

        width = int(W / Kspars);
        height = int(H / Kspars);
        New_Size.width = width;
        New_Size.height = height;

        Depth_Cloth.resize(height, width);
        Depth_Cloth0.resize(height, width);
        Vel_Cloth.resize(height, width);

        Depth_Image.resize(height, width);
        Coll_Distance.resize(height, width);
        Coll_Flag.resize(height, width);
        Coll_Flag_exp.resize(height, width);
        
        UtD_Cloth.resize(height, width);
        DtU_Cloth.resize(height, width);
        LtR_Cloth.resize(height, width);
        RtL_Cloth.resize(height, width);

        LUtRD_Cloth.resize(height, width);
        RUtLD_Cloth.resize(height, width);
        RDtLU_Cloth.resize(height, width);
        LDtRU_Cloth.resize(height, width);

        UtD2_Cloth.resize(height, width);
        DtU2_Cloth.resize(height, width);
        LtR2_Cloth.resize(height, width);
        RtL2_Cloth.resize(height, width);

        Row_trans_matrix.resize(height, height);
        Col_trans_matrix.resize(width, width);

        U_matrix = Eigen::MatrixXd::Zero(height, height);
        D_matrix = Eigen::MatrixXd::Zero(height, height);

        L_matrix = Eigen::MatrixXd::Zero(width, width);
        R_matrix = Eigen::MatrixXd::Zero(width, width);

        U2_matrix = Eigen::MatrixXd::Zero(height, height);
        D2_matrix = Eigen::MatrixXd::Zero(height, height);

        L2_matrix = Eigen::MatrixXd::Zero(width, width);
        R2_matrix = Eigen::MatrixXd::Zero(width, width);

        Row_Identity = Eigen::MatrixXd::Identity(height, height);
        Col_Identity = Eigen::MatrixXd::Identity(width, width);

        U_matrix.block(1, 0, height - 1, height - 1) = Eigen::MatrixXd::Identity(height - 1, height - 1);
        U_matrix(0, 0) = 1;
        D_matrix.block(0, 1, height - 1, height - 1) = Eigen::MatrixXd::Identity(height - 1, height - 1);
        D_matrix(height - 1, height - 1) = 1;

        U2_matrix.block(2, 0, height - 2, height - 2) = Eigen::MatrixXd::Identity(height - 2, height - 2);
        U2_matrix(0, 0) = 1;
        U2_matrix(1, 1) = 1;
        D2_matrix.block(0, 2, height - 2, height - 2) = Eigen::MatrixXd::Identity(height - 2, height - 2);
        D2_matrix(height - 2, height - 2) = 1;
        D2_matrix(height - 1, height - 1) = 1;

        L_matrix.block(0, 1, width - 1, width - 1) = Eigen::MatrixXd::Identity(width - 1, width - 1);
        L_matrix(0, 0) = 1;
        R_matrix.block(1, 0, width - 1, width - 1) = Eigen::MatrixXd::Identity(width - 1, width - 1);
        R_matrix(width - 1, width - 1) = 1;

        L2_matrix.block(0, 2, width - 2, width - 2) = Eigen::MatrixXd::Identity(width - 2, width - 2);
        L2_matrix(0, 0) = 1;
        L2_matrix(1, 1) = 1;
        R2_matrix.block(2, 0, width - 2, width - 2) = Eigen::MatrixXd::Identity(width - 2, width - 2);
        R2_matrix(width - 2, width - 2) = 1;
        R2_matrix(width - 1, width - 1) = 1;

        Row_trans_matrix = Kt * (U_matrix + D_matrix - 2.0 * Row_Identity) + Kf * (U2_matrix + D2_matrix - 2.0 * Row_Identity);
        Col_trans_matrix = Kt * (L_matrix + R_matrix - 2.0 * Col_Identity) + Kf * (L2_matrix + R2_matrix - 2.0 * Col_Identity);

        U_Traction.resize(height, width);
        D_Traction.resize(height, width);
        L_Traction.resize(height, width);
        R_Traction.resize(height, width);

        LU_Shear.resize(height, width);
        RU_Shear.resize(height, width);
        RD_Shear.resize(height, width);
        LD_Shear.resize(height, width);

        U2_Flexion.resize(height, width);
        D2_Flexion.resize(height, width);
        L2_Flexion.resize(height, width);
        R2_Flexion.resize(height, width);

        Internal_Force.resize(height, width);
        External_Force.resize(height, width);
        Force_Matrix.resize(height, width);

        Spread_Flag.resize(height, width);

        Swps_list.resize(Soft_points_num);
        Swps_sum.resize(Soft_points_num);

        External_Force.fill(9.8);
        External_Force = m * External_Force;

        Barrier_Mask.create(height, width, CV_8UC1);
        Barrier_Mask_OriSize.create(H, W, CV_8UC1);
        Access_Mask.create(height_ori, width_ori, CV_8UC1);
        Hole_Mask.create(height_ori, width_ori, CV_8UC1);
        CV_Cloth.create(height_ori, width_ori, CV_8UC3);
        CV_Cloth_Mask.create(height, width, CV_8UC1);
        sld_p_img.create(height, width, CV_8UC3);

        element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
        Filter_kernel = cv::Mat::ones(3, 3, CV_64F) / 9.0;

        int borderSize = int(height_ori * Kerase);
        cv::Rect centerRect(0, 0, W, H - borderSize);
        Safety_Filter = cv::Mat::zeros(H, W, CV_8UC1);
        // 在黑色图像中绘制白色的中心区域
        cv::rectangle(Safety_Filter, centerRect, cv::Scalar(255), -1); // -1 表示填充矩形

    }
      
};

