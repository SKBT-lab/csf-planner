#ifndef _CSF_LIDAR_H_
#define _CSF_LIDAR_H_
#include <iostream>
#include <chrono>
#include <vector>
#include <thread> 
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "odom_fusion/OdomStamp.h"

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>

#include <opencv2/opencv.hpp>

#include <yaml-cpp/yaml.h>

#include <cmath>


typedef Eigen::Vector3d Point3d;
typedef Eigen::Matrix<Point3d, Eigen::Dynamic, Eigen::Dynamic> Matrix3P;



struct ClusterInfo
{
    
    double distance;                 // 聚类的距离（例如重心到原点的距离）
    Point3d center;         // 聚类的重心
    
    double span_x;                   // x 方向的跨度
    double span_y;                   // y 方向的跨度
    double span_z;                   // z 方向的跨度
    
    Point3d min_x;
    Point3d max_x;

    Point3d min_y;
    Point3d max_y;

    Point3d min_z;
    Point3d max_z;

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud; // 聚类的点云

    // 自定义比较函数，用于按照聚类的距离从小到大排序
    bool operator<(const ClusterInfo& other) const
    {
        return distance < other.distance;
    }
    // 构造函数
    ClusterInfo() : cloud(new pcl::PointCloud<pcl::PointXYZI>) {}
};

class C_S_FILTER_LIDAR{
public:

    double x_debug;
    int state;
    double X_min;     //预处理阶段直通滤波范围 
    double X_max;
    double Y_min;
    double Y_max;
    double Z_min; 
    double Z_max;
    
    double Cloth_Width;       // 布料初始尺寸及同原点的距离
    double Cloth_Height;
    double Cloth_Depth;       //等价于min_x
    double max_x;

    double Cloth_Density;           // 布料密度，表示每隔多少米取一个点

    Matrix3P Cloth_Grid;

    int Grid_Width;
    int Grid_Height;

    double m;            //布料节点质量
    double dt;           //时间步长

    double Kt;           //Traction劲度系数
    double Ks;           //Shear劲度系数
    double Kf;           //Flexion劲度系数

    Eigen::MatrixXd F_a_scale; //系数矩阵,用于动力学更新过程中的数乘运算
    Eigen::MatrixXd dt_scale;

    int MaxIterations;
    double thresh;

    double Coll_thresh;

    Point3d V0;

    Matrix3P UtD_Grid;
    Matrix3P DtU_Grid;
    Matrix3P LtR_Grid;
    Matrix3P RtL_Grid;

    Matrix3P LUtRD_Grid;
    Matrix3P RUtLD_Grid;
    Matrix3P RDtLU_Grid;
    Matrix3P LDtRU_Grid;

    Matrix3P UtD2_Grid;
    Matrix3P DtU2_Grid;
    Matrix3P LtR2_Grid;
    Matrix3P RtL2_Grid;

    Matrix3P Internal_Force;
    Matrix3P External_Force;
    Matrix3P Force_Matrix;

    Matrix3P Vel_Grid;

    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> Coll_Flag;

    
    pcl::PointCloud<pcl::PointXYZI>::Ptr Cloud;          //主点云
    pcl::PointCloud<pcl::PointXYZI>::Ptr Vis_Cloud;      //用于可视化的点云

    pcl::PointCloud<pcl::PointXYZI>::Ptr Barrier_Cloud;  //CSF的障碍点云
    pcl::PointCloud<pcl::PointXYZI>::Ptr Hole_Cloud;     //CSF的洞点云


    //欧式聚类相关
    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree;

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
    pcl::VoxelGrid<pcl::PointXYZI> vg;
    

    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> clusters_list;

    std::vector<Point3d> Center_list;

    std::vector<ClusterInfo> Clusters_info;

    int clusters_num;

    double Cluster_thresh;
    int Cluster_minnum;
    int Cluster_maxnum;

    double x_vg;
    double y_vg;
    double z_vg;

    int x_seg_num;
    int y_seg_num;
    std::vector<std::pair<int, std::vector<pcl::PointXYZI>>> bins_x;
    std::vector<std::pair<int, std::vector<pcl::PointXYZI>>> bins_y;

    //隧道相关
    Point3d Suidao_center;
    double Suidao_angle;
    double Suidao_lenth; 
    
    //-------迷宫相关
    pcl::PointCloud<pcl::PointXYZI>::Ptr Wall_Cloud;     //墙点云
    double Wall_minY;     //墙点云的Y下限
    double Wall_maxY;     //墙点云的Y上限
    Eigen::Vector3d Window_center;

    cv::Mat wall_image;
    cv::Mat result_img;
    int image_width;
    int image_height;

    int block_len;
    int projection_coeff;

    // 用于闭操作
    int morph_size;
    cv::Mat element;
    //用于平面提取
    pcl::SACSegmentation<pcl::PointXYZI> seg;
    // 创建用于存储平面模型系数的对象
    pcl::ModelCoefficients::Ptr coefficients;
    pcl::PointIndices::Ptr inliers;

    pcl::ExtractIndices<pcl::PointXYZI> extract;

    std::vector<std::vector<cv::Point>> window_contours;

    //用于level1
    double level1_miny;
    double level1_maxy;
    int interval_num;
    std::vector<Eigen::Vector3d> level1_waypoints;


    double CalculateAverageNorm(const Matrix3P& matrix);
    void applyPassThroughFilter(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
                            pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_filtered,
                            double x_min, double x_max,
                            double y_min, double y_max,
                            double z_min, double z_max) ;
    void Input_Cloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, 
                    double x_min_ = -1, double x_max_ = -1, 
                    double y_min_ = -1, double y_max_ = -1, 
                    double z_min_ = -1, double z_max_ = -1);
    void Reset_Cloth();
    void Cal_force();
    void Collision_Detection();
    Matrix3P Cwiseproduct(Matrix3P matrix, Eigen::MatrixXd scale);
    void Update_Cloth();
    void Shot();
    void Get_Visualization_Cloth();
    void DownSampling();
    void Euclidean_Clustering();
    void Find_suidao(ClusterInfo c_info);
    void Find_wall();
    void Find_window_center();
    void process_level1();
    bool Get_Wallimg(pcl::PointCloud<pcl::PointXYZI>::Ptr& c);
    //static bool compare(const std::pair<int, std::vector<pcl::PointXYZI>>& a, const std::pair<int, std::vector<pcl::PointXYZI>>& b);

    //Eigen::Vector3d Get_3Dpos_Body(cv::Point2f& pix_point, double& depth);
    //Eigen::Vector3d Get_3Dpos(cv::Point2f& pix_point, double& depth, const Eigen::Quaterniond& q = Eigen::Quaterniond::Identity(), const Eigen::Vector3d& p = Eigen::Vector3d::Zero());

    YAML::Node config;
    
    C_S_FILTER_LIDAR(){
        config = YAML::LoadFile("/home/skbt/SKBT_Drone/src/detection/pointcloud_detection/config/Cloth_Para.yaml");
        X_min = config["X_min"].as<double>();
        X_max = config["X_max"].as<double>();
        Y_min = config["Y_min"].as<double>();
        Y_max = config["Y_max"].as<double>();
        Z_min = config["Z_min"].as<double>();
        Z_max = config["Z_max"].as<double>();

        m = config["m"].as<double>();
        Kt = config["Kt"].as<double>();
        Ks = config["Ks"].as<double>();
        Kf = config["Kf"].as<double>();
        dt = config["dt"].as<double>();

        std::vector<double> v0;
        v0 = config["V0"].as<std::vector<double>>();
        V0 = Point3d(v0[0], v0[1], v0[2]);

        MaxIterations = config["MaxIterations"].as<int>();
        thresh = config["thresh"].as<double>();
        Coll_thresh = config["Coll_thresh"].as<double>();

        Cloth_Width = config["Cloth_Width"].as<double>();
        Cloth_Height = config["Cloth_Height"].as<double>();
        Cloth_Depth = config["Cloth_Depth"].as<double>();

        Cloth_Density = config["Cloth_Density"].as<double>();

        Cluster_thresh = config["Cluster_thresh"].as<double>();
        Cluster_minnum = config["Cluster_minnum"].as<int>();
        Cluster_maxnum = config["Cluster_maxnum"].as<int>();

        x_vg = config["x_vg"].as<double>();
        y_vg = config["y_vg"].as<double>();
        z_vg = config["z_vg"].as<double>();

        Suidao_lenth = config["Suidao_lenth"].as<double>();

        Grid_Width = int((Y_max - Y_min) / Cloth_Density) + 1;
        Grid_Height = int((Z_max - Z_min) / Cloth_Density) + 1;

        Cloud.reset(new pcl::PointCloud<pcl::PointXYZI>);
        Vis_Cloud.reset(new pcl::PointCloud<pcl::PointXYZI>);

        Barrier_Cloud.reset(new pcl::PointCloud<pcl::PointXYZI>);
        Hole_Cloud.reset(new pcl::PointCloud<pcl::PointXYZI>);

        Wall_Cloud.reset(new pcl::PointCloud<pcl::PointXYZI>);

        tree.reset(new pcl::search::KdTree<pcl::PointXYZI>);
        
        Cloth_Grid.resize(Grid_Height, Grid_Width);
        Vel_Grid.resize(Grid_Height, Grid_Width);
        Vel_Grid.fill(V0);
        
        UtD_Grid.resize(Grid_Height, Grid_Width);
        DtU_Grid.resize(Grid_Height, Grid_Width);
        LtR_Grid.resize(Grid_Height, Grid_Width);
        RtL_Grid.resize(Grid_Height, Grid_Width);

        LUtRD_Grid.resize(Grid_Height, Grid_Width);
        RUtLD_Grid.resize(Grid_Height, Grid_Width);
        RDtLU_Grid.resize(Grid_Height, Grid_Width);
        LDtRU_Grid.resize(Grid_Height, Grid_Width);

        UtD2_Grid.resize(Grid_Height, Grid_Width);
        DtU2_Grid.resize(Grid_Height, Grid_Width);
        LtR2_Grid.resize(Grid_Height, Grid_Width);
        RtL2_Grid.resize(Grid_Height, Grid_Width);

        Internal_Force.resize(Grid_Height, Grid_Width);
        External_Force.resize(Grid_Height, Grid_Width);
        External_Force.fill(Point3d(m * 9.8, 0.0, 0.0));

        Force_Matrix.resize(Grid_Height, Grid_Width);

        F_a_scale.resize(Grid_Height, Grid_Width);
        F_a_scale.fill(dt / m);

        dt_scale.resize(Grid_Height, Grid_Width);
        dt_scale.fill(dt);

        Coll_Flag.resize(Grid_Height, Grid_Width);
        Coll_Flag.fill(1.0);

        ec.setClusterTolerance(Cluster_thresh); // 聚类的距离阈值，请根据需要调整
        ec.setMinClusterSize(Cluster_minnum);   // 最小的聚类点数，请根据需要调整
        ec.setMaxClusterSize(Cluster_maxnum); // 最大的聚类点数，请根据需要调整

        vg.setLeafSize(x_vg, y_vg, z_vg); // 请根据需要调整体素大小

        x_seg_num = config["x_seg_num"].as<int>();
        y_seg_num = config["y_seg_num"].as<int>();

        block_len = config["block_len"].as<int>();
        projection_coeff = config["projection_coeff"].as<int>();
        
        
        bins_x.resize(x_seg_num);
        bins_y.resize(y_seg_num);
        

        Wall_minY = std::numeric_limits<double>::max();
        Wall_maxY = std::numeric_limits<double>::min();
        Window_center = Eigen::Vector3d::Zero();

        image_width = int((Y_max - Y_min) * projection_coeff) + (block_len - 1);
        image_height = int((Z_max - Z_min) * projection_coeff) + (block_len - 1);
        wall_image = cv::Mat::zeros(image_height, image_width, CV_8UC1);
        result_img = cv::Mat::zeros(image_height, image_width * 4, CV_8UC3);

        // 定义结构元素
        morph_size = 5; // 可以调整结构元素的大小
        element = cv::getStructuringElement(cv::MORPH_RECT,
                                                    cv::Size(2 * morph_size + 1, 2 * morph_size + 1),
                                                    cv::Point(morph_size, morph_size));

        for (auto& bin : bins_x) {
            bin.first = 0;
            bin.second = std::vector<pcl::PointXYZI>();
        }

        for (auto& bin : bins_y) {
            bin.first = 0;
            bin.second = std::vector<pcl::PointXYZI>();
        }

        seg.setOptimizeCoefficients(true); // 可选项
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setDistanceThreshold(0.1); // 设置距离阈值

        coefficients.reset(new pcl::ModelCoefficients);
        inliers.reset(new pcl::PointIndices);

        interval_num = config["interval_num"].as<int>();
        level1_miny = config["level1_miny"].as<double>();
        level1_maxy = config["level1_maxy"].as<double>();

        state = config["state"].as<int>();
        x_debug = config["x_debug"].as<double>();
        

    } 
};

#endif