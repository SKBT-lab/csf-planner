#include "CSF_Lidar.h"



double C_S_FILTER_LIDAR::CalculateAverageNorm(const Matrix3P& matrix) {
    double sumNorms = 0.0;
    int numElements = matrix.rows() * matrix.cols();

    // 遍历矩阵的每个元素
    for (int i = 0; i < matrix.rows(); ++i) {
        for (int j = 0; j < matrix.cols(); ++j) {
            sumNorms += matrix(i, j).norm();
        }
    }

    // 计算平均值
    return sumNorms / numElements;
}

void C_S_FILTER_LIDAR::applyPassThroughFilter(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
                            pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_filtered,
                            double x_min, double x_max,
                            double y_min, double y_max,
                            double z_min, double z_max) {
    // 创建直通滤波器对象
    pcl::PassThrough<pcl::PointXYZI> pass;
    
    // 设置X轴的过滤范围并进行过滤
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(x_min, x_max);
    pass.filter(*cloud_filtered);

    // 设置Y轴的过滤范围并进行过滤
    pass.setInputCloud(cloud_filtered);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(y_min, y_max);
    pass.filter(*cloud_filtered);

    // 设置Z轴的过滤范围并进行过滤
    pass.setInputCloud(cloud_filtered);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(z_min, z_max);
    pass.filter(*cloud_filtered);
}

void C_S_FILTER_LIDAR::Input_Cloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
                    double x_min_, double x_max_, 
                    double y_min_, double y_max_, 
                    double z_min_, double z_max_){
    if(x_min_ == -1){
        x_min_ = X_min;
    }

    if(x_max_ == -1){
        x_max_ = X_max;
    }

    if(y_min_ == -1){
        y_min_ = Y_min;
    }

    if(y_max_ == -1){
        y_max_ = Y_max;
    }

    if(z_min_ == -1){
        z_min_ = Z_min;
    }

    if(z_max_ == -1){
        z_max_ = Z_max;
    }
    pcl::PointCloud<pcl::PointXYZI>::Ptr Cloud_(new pcl::PointCloud<pcl::PointXYZI>);
    Cloud_ = cloud;
    applyPassThroughFilter(Cloud_, Cloud, x_min_, x_max_, y_min_, y_max_, z_min_, z_max_);
    double min_x = std::numeric_limits<double>::max();
    max_x = std::numeric_limits<double>::min();
    for (const auto& point : *Cloud) {
        //std::cout << point.x << ", " << point.y << ", " << point.z << std::endl;
        if (point.x < min_x && point.x != 0) {
            min_x = point.x;
        }
        if (point.x > max_x && point.x != 0) {
            max_x = point.x;
        }
    }

    Cloth_Depth = min_x;
}

void C_S_FILTER_LIDAR::Reset_Cloth(){
    for(size_t i = 0; i < Grid_Height; i++){
        for(size_t j = 0; j < Grid_Width; j++){
            Cloth_Grid(i, j) = Point3d(Cloth_Depth, j * Cloth_Density - Cloth_Width / 2, i * Cloth_Density - Cloth_Height / 2);
        }
    }
    Vel_Grid.fill(V0);
    Coll_Flag.fill(1.0);
    clusters_list.clear();
    cluster_indices.clear();
    Center_list.clear();
    Clusters_info.clear();
}

void C_S_FILTER_LIDAR::Cal_force(){

    // Up_traction
    UtD_Grid = Cloth_Grid;
    UtD_Grid.block(0, 0, Grid_Height - 1, Grid_Width) = Cloth_Grid.block(1, 0, Grid_Height - 1, Grid_Width);
    UtD_Grid = UtD_Grid - Cloth_Grid;
    for(size_t i = 0; i < Grid_Height; i++){
        for(size_t j = 0; j < Grid_Width; j++){
            UtD_Grid(i, j) = Kt * (UtD_Grid(i, j).norm() - Cloth_Density) * UtD_Grid(i, j).normalized();
        }
    }

    // Down_traction
    DtU_Grid = Cloth_Grid;
    DtU_Grid.block(1, 0, Grid_Height - 1, Grid_Width) = Cloth_Grid.block(0, 0, Grid_Height - 1, Grid_Width);
    DtU_Grid = DtU_Grid - Cloth_Grid;
    for(size_t i = 0; i < Grid_Height; i++){
        for(size_t j = 0; j < Grid_Width; j++){
            DtU_Grid(i, j) = Kt * (DtU_Grid(i, j).norm() - Cloth_Density) * DtU_Grid(i, j).normalized();
        }
    }

    // Left_traction
    LtR_Grid = Cloth_Grid;
    LtR_Grid.block(0, 0, Grid_Height, Grid_Width - 1) = Cloth_Grid.block(0, 1, Grid_Height, Grid_Width - 1);
    LtR_Grid = LtR_Grid - Cloth_Grid;
    for(size_t i = 0; i < Grid_Height; i++){
        for(size_t j = 0; j < Grid_Width; j++){
            LtR_Grid(i, j) = Kt * (LtR_Grid(i, j).norm() - Cloth_Density) * LtR_Grid(i, j).normalized();
        }
    }

    // Right_traction
    RtL_Grid = Cloth_Grid;
    RtL_Grid.block(0, 1, Grid_Height, Grid_Width - 1) = Cloth_Grid.block(0, 0, Grid_Height, Grid_Width - 1);
    RtL_Grid = RtL_Grid - Cloth_Grid;
    for(size_t i = 0; i < Grid_Height; i++){
        for(size_t j = 0; j < Grid_Width; j++){
            RtL_Grid(i, j) = Kt * (RtL_Grid(i, j).norm() - Cloth_Density) * RtL_Grid(i, j).normalized();
        }
    }

    // // LUtRD_Shear
    // LUtRD_Grid = Cloth_Grid;
    // LUtRD_Grid.block(1, 1, Grid_Height - 1, Grid_Width - 1) = Cloth_Grid.block(0, 0, Grid_Height - 1, Grid_Width - 1);
    // LUtRD_Grid = LUtRD_Grid - Cloth_Grid;
    // for(size_t i = 0; i < Grid_Height; i++){
    //     for(size_t j = 0; j < Grid_Width; j++){
    //         LUtRD_Grid(i, j) = Ks * (LUtRD_Grid(i, j).norm() - Cloth_Density) * LUtRD_Grid(i, j).normalized();
    //     }
    // }

    // // RUtLD_Shear
    // RUtLD_Grid = Cloth_Grid;
    // RUtLD_Grid.block(1, 0, Grid_Height - 1, Grid_Width - 1) = Cloth_Grid.block(0, 1, Grid_Height - 1, Grid_Width - 1);
    // RUtLD_Grid = RUtLD_Grid - Cloth_Grid;

    // for(size_t i = 0; i < Grid_Height; i++){
    //     for(size_t j = 0; j < Grid_Width; j++){
    //         RUtLD_Grid(i, j) = Ks * (RUtLD_Grid(i, j).norm() - Cloth_Density) * RUtLD_Grid(i, j).normalized();
    //     }
    // }

    // // RDtLU_Shear
    // RDtLU_Grid = Cloth_Grid;
    // RDtLU_Grid.block(0, 1, Grid_Height - 1, Grid_Width - 1) = Cloth_Grid.block(1, 0, Grid_Height - 1, Grid_Width - 1);
    // RDtLU_Grid = RDtLU_Grid - Cloth_Grid;
    // for(size_t i = 0; i < Grid_Height; i++){
    //     for(size_t j = 0; j < Grid_Width; j++){
    //         RDtLU_Grid(i, j) = Ks * (RDtLU_Grid(i, j).norm() - Cloth_Density) * RDtLU_Grid(i, j).normalized();
    //     }
    // }

    // // LDtRU_Shear
    // LDtRU_Grid = Cloth_Grid;
    // LDtRU_Grid.block(0, 0, Grid_Height - 1, Grid_Width - 1) = Cloth_Grid.block(1, 1, Grid_Height - 1, Grid_Width - 1);
    // LDtRU_Grid = LDtRU_Grid - Cloth_Grid;
    // for(size_t i = 0; i < Grid_Height; i++){
    //     for(size_t j = 0; j < Grid_Width; j++){
    //         LDtRU_Grid(i, j) = Ks * (LDtRU_Grid(i, j).norm() - Cloth_Density) * LDtRU_Grid(i, j).normalized();
    //     }
    // }

    // // U2_Flexion
    // UtD2_Grid = Cloth_Grid;
    // UtD2_Grid.block(0, 0, Grid_Height - 2, Grid_Width) = Cloth_Grid.block(2, 0, Grid_Height - 2, Grid_Width);
    // UtD2_Grid = UtD2_Grid - Cloth_Grid;
    // for(size_t i = 0; i < Grid_Height; i++){
    //     for(size_t j = 0; j < Grid_Width; j++){
    //         UtD2_Grid(i, j) = Kf * (UtD2_Grid(i, j).norm() - Cloth_Density) * UtD2_Grid(i, j).normalized();
    //     }
    // }

    // // D2_Flexion
    // DtU2_Grid = Cloth_Grid;
    // DtU2_Grid.block(2, 0, Grid_Height - 2, Grid_Width) = Cloth_Grid.block(0, 0, Grid_Height - 2, Grid_Width);
    // DtU2_Grid = DtU2_Grid - Cloth_Grid;
    // for(size_t i = 0; i < Grid_Height; i++){
    //     for(size_t j = 0; j < Grid_Width; j++){
    //         DtU2_Grid(i, j) = Kf * (DtU2_Grid(i, j).norm() - Cloth_Density) * DtU2_Grid(i, j).normalized();
    //     }
    // }

    // // L2_Flexion
    // LtR2_Grid = Cloth_Grid;
    // LtR2_Grid.block(0, 0, Grid_Height, Grid_Width - 2) = Cloth_Grid.block(0, 2, Grid_Height, Grid_Width - 2);
    // LtR2_Grid = LtR2_Grid - Cloth_Grid;
    // for(size_t i = 0; i < Grid_Height; i++){
    //     for(size_t j = 0; j < Grid_Width; j++){
    //         LtR2_Grid(i, j) = Kf * (LtR2_Grid(i, j).norm() - Cloth_Density) * LtR2_Grid(i, j).normalized();
    //     }
    // }

    // // R2_Flexion
    // RtL2_Grid = Cloth_Grid;
    // RtL2_Grid.block(0, 2, Grid_Height, Grid_Width - 2) = Cloth_Grid.block(0, 0, Grid_Height, Grid_Width - 2);
    // RtL2_Grid = RtL2_Grid - Cloth_Grid;
    // for(size_t i = 0; i < Grid_Height; i++){
    //     for(size_t j = 0; j < Grid_Width; j++){
    //         RtL2_Grid(i, j) = Kf * (RtL2_Grid(i, j).norm() - Cloth_Density) * RtL2_Grid(i, j).normalized();
    //     }
    // }

    //Force
    Internal_Force = UtD_Grid + DtU_Grid + LtR_Grid + RtL_Grid
                 + LUtRD_Grid + RUtLD_Grid + RDtLU_Grid + LDtRU_Grid
                 + UtD2_Grid + DtU2_Grid + LtR2_Grid + RtL2_Grid;

    Force_Matrix = Internal_Force + External_Force;
} 

void C_S_FILTER_LIDAR::Collision_Detection(){
    pcl::PointXYZI cloth_point; //布料中当前遍历到的点
    for (size_t i = 0; i < Grid_Height; i++){
        for (size_t j = 0; j < Grid_Width; j++){
            //std::cout << "i:" << i <<"j:" << j << std::endl;
            if(Coll_Flag(i, j) == 1.0){
                // 最近点及其距离
                Point3d nearest_point;
                double min_distance = std::numeric_limits<double>::max();
                // 遍历点云中的每个点，寻找最近的点
                for (size_t k = 0; k < Cloud->points.size(); ++k) {
                    pcl::PointXYZI cloud_point = Cloud->points[k]; 
                    Point3d p(cloud_point.x, cloud_point.y, cloud_point.z);
                    // 计算当前点与给定点之间的欧氏距离
                    double distance = (p - Cloth_Grid(i, j)).norm();
                    
                    // 更新最小距离及最近点
                    if (distance < min_distance) {
                        min_distance = distance;
                        nearest_point = p;
                    }
                }
                
                if (min_distance < Coll_thresh){  
                    Coll_Flag(i, j) = 0.0;
                    // Cloth_Grid(i, j) = nearest_point;
                }
            }
        }
    }
} 

Matrix3P C_S_FILTER_LIDAR::Cwiseproduct(Matrix3P matrix, Eigen::MatrixXd scale){
    Matrix3P m;
    m.resize(Grid_Height, Grid_Width);
    for(size_t i = 0; i < Grid_Height; i++){
        for(size_t j = 0; j < Grid_Width; j++){
            m(i, j) = scale(i, j) * matrix(i, j);
        }
    }
    return m;
}

void C_S_FILTER_LIDAR::Update_Cloth(){
    Vel_Grid = Vel_Grid + Cwiseproduct(Force_Matrix, F_a_scale);
    Vel_Grid = Cwiseproduct(Vel_Grid, Coll_Flag);
    Cloth_Grid = Cloth_Grid + Cwiseproduct(Vel_Grid, dt_scale);
}

void C_S_FILTER_LIDAR::Shot(){
    int iteration = 0;
    while (iteration < MaxIterations)
    {
        iteration ++;
        Collision_Detection();
        Cal_force();
        Update_Cloth();
        // std::cout << Vel_Cloth << std::endl;
        if (CalculateAverageNorm(Vel_Grid) < thresh){
            // std::cout << iteration << std::endl;
            break;
        }
    }
    // std::cout <<  iteration << std::endl;
} 

// Eigen::Vector3d C_S_FILTER_LIDAR::Get_3Dpos(cv::Point2f& pix_point, double& depth, const Eigen::Quaterniond& q, const Eigen::Vector3d& p){ 
//     // Eigen::Vector3d UV1(pix_point.x, pix_point.y, 1);
//     // return q * (q_CtoB * (depth * Intri_Matrix_Inverse * UV1)) + p;
// }

void C_S_FILTER_LIDAR::Get_Visualization_Cloth(){
    Vis_Cloud->clear();
    Barrier_Cloud->clear();
    Hole_Cloud->clear();
    int k = 0;
    for (int i = 0; i < Grid_Height; ++i) {
        for (int j = 0; j < Grid_Width; ++j) {
            pcl::PointXYZI point;
            point.x = Cloth_Grid(i, j).x();
            point.y = Cloth_Grid(i, j).y();
            point.z = Cloth_Grid(i, j).z();
            if(Coll_Flag(i, j) == 1.0){
                point.intensity = 255.0f;
                Hole_Cloud->points.push_back(point);
            }
            else{
                k++;
                point.intensity = 0.0f;
                Barrier_Cloud->points.push_back(point);
            }
            
            Vis_Cloud->points.push_back(point);
        }
    }
    // std::cout << k << std::endl;

    Vis_Cloud->width = Grid_Width;
    Vis_Cloud->height = Grid_Height;
    Vis_Cloud->is_dense = true;

    Hole_Cloud->width = Hole_Cloud->points.size();
    Hole_Cloud->height = 1 ;
    Hole_Cloud->is_dense = true;

    Barrier_Cloud->width = Barrier_Cloud->points.size();
    Barrier_Cloud->height = 1;
    Barrier_Cloud->is_dense = true;
} 


void C_S_FILTER_LIDAR::DownSampling(){
    // 下采样点云以加速聚类
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>);
    
    vg.setInputCloud(Cloud);
    
    vg.filter(*cloud_filtered);
    pcl::copyPointCloud(*cloud_filtered, *Cloud);
} 

void C_S_FILTER_LIDAR::Euclidean_Clustering(){
    // 创建 KdTree 对象以进行欧式聚类
    tree->setInputCloud(Cloud);

    ec.setSearchMethod(tree);
    ec.setInputCloud(Cloud);
    ec.extract(cluster_indices);

    // 遍历每个聚类，并将其添加到不同的点云对象中
    clusters_num = 0;
    for (const auto& indices : cluster_indices)
    {   ClusterInfo cluster_info;
        cluster_info.max_x.x() = -std::numeric_limits<double>::max();
        cluster_info.max_y.y() = -std::numeric_limits<double>::max();
        cluster_info.max_z.z() = -std::numeric_limits<double>::max();
        cluster_info.min_x.x() = std::numeric_limits<double>::max();
        cluster_info.min_y.y() = std::numeric_limits<double>::max();
        cluster_info.min_z.z() = std::numeric_limits<double>::max();

        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZI>);
        Point3d Center(0.0, 0.0, 0.0);
        for (const auto& idx : indices.indices){
            cloud_cluster->points.push_back(Cloud->points[idx]);
            Center.x() += Cloud->points[idx].x;
            Center.y() += Cloud->points[idx].y;
            Center.z() += Cloud->points[idx].z;
            if (Cloud->points[idx].x < cluster_info.min_x.x()){
                cluster_info.min_x.x() = Cloud->points[idx].x;
                cluster_info.min_x.y() = Cloud->points[idx].y;
                cluster_info.min_x.z() = Cloud->points[idx].z;
            }

            if (Cloud->points[idx].y < cluster_info.min_y.y()){
                cluster_info.min_y.x() = Cloud->points[idx].x;
                cluster_info.min_y.y() = Cloud->points[idx].y;
                cluster_info.min_y.z() = Cloud->points[idx].z;
            }

            if (Cloud->points[idx].z < cluster_info.min_z.z()){
                cluster_info.min_z.x() = Cloud->points[idx].x;
                cluster_info.min_z.y() = Cloud->points[idx].y;
                cluster_info.min_z.z() = Cloud->points[idx].z;
            }

            if (Cloud->points[idx].x > cluster_info.max_x.x()) {
                cluster_info.max_x.x() = Cloud->points[idx].x;
                cluster_info.max_x.y() = Cloud->points[idx].y;
                cluster_info.max_x.z() = Cloud->points[idx].z;
            }

            if (Cloud->points[idx].y > cluster_info.max_y.y()) {
                cluster_info.max_y.x() = Cloud->points[idx].x;
                cluster_info.max_y.y() = Cloud->points[idx].y;
                cluster_info.max_y.z() = Cloud->points[idx].z;
            }

            if (Cloud->points[idx].z > cluster_info.max_z.z()) {
                cluster_info.max_z.x() = Cloud->points[idx].x;
                cluster_info.max_z.y() = Cloud->points[idx].y;
                cluster_info.max_z.z() = Cloud->points[idx].z;
            }
        }

        cluster_info.span_x = cluster_info.max_x.x() - cluster_info.min_x.x();
        cluster_info.span_y = cluster_info.max_y.y() - cluster_info.min_y.y();
        cluster_info.span_z = cluster_info.max_z.z() - cluster_info.min_z.z();
        
        Center = Center / cloud_cluster->points.size();
        cloud_cluster->width = cloud_cluster->points.size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        cluster_info.center = Center;
        cluster_info.distance = Center.norm();
        pcl::copyPointCloud(*cloud_cluster, *cluster_info.cloud);

        Clusters_info.push_back(cluster_info);
        clusters_num++;
    }
    // 按照距离排序
    std::sort(Clusters_info.begin(), Clusters_info.end());

} 

        //        5———————————8
        //       /|          /|
        //      / |         / |
        //     /  |        /  |
        //    /1__|_______4   |
        //    |   6———————| ——7
        //    |  /        |  /
        //    | /         | /
        //    2/——————————3/
         
void C_S_FILTER_LIDAR::Find_suidao(ClusterInfo c_info){
    Point3d p1,p2,p3,p4,p5,p6,p7,p8;
    p1 << c_info.max_y.x(), c_info.max_y.y(), c_info.max_z.z();
    p2 << c_info.max_y.x(), c_info.max_y.y(), c_info.min_z.z();

    p3 << c_info.min_x.x(), c_info.min_x.y(), c_info.min_z.z();
    p4 << c_info.min_x.x(), c_info.min_x.y(), c_info.max_z.z();

    p7 << c_info.min_y.x(), c_info.min_y.y(), c_info.min_z.z();
    p8 << c_info.min_y.x(), c_info.min_y.y(), c_info.max_z.z();

    p5 = p8 + (p1 - p4);
    p6 = p7 + (p2 - p3);
    
    Suidao_center = (p1 + p2 + p3 + p4 + p5 + p6 + p7 + p8) / 8;
    Suidao_angle = asin((p7.x() - p3.x()) / Suidao_lenth); //返回弧度
}

// // 定义一个比较函数用于排序
// static bool C_S_FILTER_LIDAR::compare(const std::pair<int, std::vector<pcl::PointXYZI>>& a, const std::pair<int, std::vector<pcl::PointXYZI>>& b) {
//     return a.first > b.first;
// }

// void C_S_FILTER_LIDAR::Find_wall(){
//     Wall_Cloud->clear();
//     Wall_minY = std::numeric_limits<double>::max();
//     Wall_maxY = std::numeric_limits<double>::min();
    
//     float interval = X_max / x_seg_num;
//      // 统计每个区间中的点
//     for (auto& bin : bins_x) {   //先清空数值
//             bin.first = 0;
//             bin.second.clear();
//         }
//     for (const auto& point : Cloud->points) {
//         if (point.x >= 0 && point.x <= X_max) {
//             int bin_index = std::min(static_cast<int>(point.x / interval), x_seg_num - 1);
//             bins_x[bin_index].second.push_back(point);
//             bins_x[bin_index].first++;
//         }
//     }
//     std::sort(bins_x.begin(), bins_x.end(), [](const std::pair<int, std::vector<pcl::PointXYZI>>& a, const std::pair<int, std::vector<pcl::PointXYZI>>& b) {
//             return a.first > b.first;
//         });

//     const auto& max_bin = bins_x.front();

//     // 将max_bin.second中的点添加到新的点云对象中
//     for (const auto& point : max_bin.second) {
//         Wall_Cloud->points.push_back(point);
//         if(point.y < Wall_minY){
//             Wall_minY = point.y;
//         }
//         if(point.y > Wall_maxY){
//             Wall_maxY = point.y;
//         }
//     }
//     Wall_Cloud->width = Wall_Cloud->points.size();
//     Wall_Cloud->height = 1;  // 单行点云
//     Wall_Cloud->is_dense = true;
// }

void C_S_FILTER_LIDAR::Find_wall(){
    // 对点云进行分割以提取平面
    seg.setInputCloud(Cloud);
    seg.segment(*inliers, *coefficients);

    extract.setInputCloud(Cloud);
    extract.setIndices(inliers);
    extract.setNegative(false); // 设置为false提取平面内点，为true提取剩余点

    extract.filter(*Wall_Cloud);

}


void C_S_FILTER_LIDAR::Find_window_center(){
    float interval = (Wall_maxY - Wall_minY) / y_seg_num;
     // 统计每个区间中的点
    for (auto& bin : bins_y) {   //先清空数值
            bin.first = 0;
            bin.second.clear();
        }
    for (const auto& point : Wall_Cloud->points) {
        if (point.y >= Wall_minY && point.y <= Wall_maxY) {
            int bin_index = std::min(static_cast<int>((point.y - Wall_minY) / interval), y_seg_num - 1);
            bins_y[bin_index].second.push_back(point);
            bins_y[bin_index].first++;
        }
    }
    // 计算所有区间点数的总和和平均值
    int total_points = std::accumulate(bins_y.begin(), bins_y.end(), 0,
                                    [](int sum, const std::pair<int, std::vector<pcl::PointXYZI>>& bin) {
                                        return sum + bin.first;
                                    });
    float average_points = static_cast<float>(total_points) / y_seg_num;

    int n = 0;
    float sumy = 0.0;
    float sumx = 0.0;
    for (const auto& bin : bins_y) {
        if (bin.first < average_points) {
            for (const auto& point : bin.second){
                sumy += point.y;
                sumx += point.x;
                n ++;        
            }
        }
    }

    Window_center.y() = sumy / n;
    Window_center.x() = sumx / n;
}



bool C_S_FILTER_LIDAR::Get_Wallimg(pcl::PointCloud<pcl::PointXYZI>::Ptr& c){
    wall_image.setTo(cv::Scalar(0));
    // cv::rectangle(wall_image, cv::Point(0, 0), cv::Point(image_width, int(image_height / 4)), cv::Scalar(255), cv::FILLED);
    cv::rectangle(wall_image, cv::Point(0, int(image_height * 0.5)), cv::Point(image_width, image_height), cv::Scalar(255), cv::FILLED);
    for (const auto& point : c->points) {
        int c_x = int((Y_max - point.y) * projection_coeff) + (block_len - 1) / 2; //四周多围一圈，防止滑块溢出
        int c_y = int((Z_max - point.z) * projection_coeff) + (block_len - 1) / 2;
        // 确定矩形的左上角和右下角坐标
        int x_start = std::max(c_x - (block_len - 1) / 2, 0);
        int y_start = std::max(c_y - (block_len - 1) / 2, 0);
        int x_end = std::min(c_x + (block_len - 1) / 2, wall_image.cols - 1);
        int y_end = std::min(c_y + (block_len - 1) / 2, wall_image.rows - 1);

        // 将矩形范围内的所有像素值设置为 255
        for (int y = y_start; y <= y_end; ++y) {
            for (int x = x_start; x <= x_end; ++x) {
                wall_image.at<uchar>(y, x) = 255;
            }
        }
    }

    std::vector<cv::Mat> images;
    cv::Mat img_pro, img_clo, img_not,img_cen;
    cv::cvtColor(wall_image, img_pro, cv::COLOR_GRAY2BGR);
    cv::morphologyEx(wall_image, wall_image, cv::MORPH_CLOSE, element);
    cv::cvtColor(wall_image, img_clo, cv::COLOR_GRAY2BGR);
    cv::bitwise_not(wall_image, wall_image);
    cv::cvtColor(wall_image, img_not, cv::COLOR_GRAY2BGR);

    images.push_back(img_pro);
    images.push_back(img_clo);
    images.push_back(img_not);

    // 查找轮廓
    window_contours.clear();
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(wall_image, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

     // 排除与图像四边相邻的轮廓
    for (const auto& contour : contours) {
        bool isAdjacentToBorder = false;
        for (const auto& point : contour) {
            if (point.x == 0 || point.x == wall_image.cols - 1 || point.y == 0 || point.y == wall_image.rows - 1) {
                isAdjacentToBorder = true;
                break;
            }
        }
        if (!isAdjacentToBorder) {
            window_contours.push_back(contour);
        }
    }

    

    cv::cvtColor(wall_image, img_cen, cv::COLOR_GRAY2BGR);
    images.push_back(img_cen);
    cv::drawContours(img_cen, window_contours, -1, cv::Scalar(0, 255, 0), cv::FILLED);

    for (size_t i = 0; i < images.size(); ++i) {
        cv::Rect roi(cv::Point(i * image_width, 0), images[i].size());
        images[i].copyTo(result_img(roi));
    }


    
    if (window_contours.size() == 1){
        cv::Moments moments = cv::moments(window_contours[0]);

        // 计算重心坐标
        double cX = moments.m10 / moments.m00;
        double cY = moments.m01 / moments.m00;

        // 在图像上标记重心
        cv::circle(result_img, cv::Point(static_cast<int>(cX + 3 * image_width), static_cast<int>(cY)), 2, cv::Scalar(0, 0, 255), -1);
        Window_center.y() = Y_max - cX / projection_coeff;
        Window_center.z() = Z_max - cY / projection_coeff;
        Window_center.x() = std::numeric_limits<double>::min();
        for (const auto& point : c->points){
            if (point.x > Window_center.x())
            {
                
                Window_center.x() = point.x;
            }
        }
        return true;
    }
    else{
        return false;
    }
}

void C_S_FILTER_LIDAR::process_level1(){
    level1_waypoints.clear();
    std::vector<Eigen::Vector3d> points;
    for (size_t i = 0; i < Clusters_info.size(); ++i){
        points.push_back(Clusters_info[i].center);
    }
    // 计算x的最小值和最大值
    auto min_max_x = std::minmax_element(points.begin(), points.end(), [](const Eigen::Vector3d& a, const Eigen::Vector3d& b) {
        return a.x() < b.x();
    });
    
    double l1_min_x = min_max_x.first->x();
    double l1_max_x = min_max_x.second->x();
    double x_span = l1_max_x - l1_min_x;

    // 计算每个区间的大小
    double interval = x_span / interval_num;

    // 创建n个区间
    std::vector<std::vector<Eigen::Vector3d>> intervals(interval_num);

    // 将点分到各个区间
    for (const auto& point : points) {
        int index = std::min(static_cast<int>((point.x() - l1_min_x) / interval), interval_num - 1);
        intervals[index].push_back(point);
    }

    // 处理每个区间
    for (size_t i = 0; i < intervals.size(); ++i){
        Eigen::Vector3d p(l1_min_x + i * interval, level1_miny, 1.2);
        intervals[i].push_back(p);
        p << l1_min_x + i * interval, level1_maxy, 1.2;
        intervals[i].push_back(p);

        // 按y坐标升序排序
        std::vector<Eigen::Vector3d> sorted_points = intervals[i];
        std::sort(sorted_points.begin(), sorted_points.end(), [](const Eigen::Vector3d& a, const Eigen::Vector3d& b) {
            return a.y() < b.y();
        });
        

        // 找到y相差最多的相邻点
        double max_y_diff = 0;
        int max_y_diff_index = 0;
        for (size_t i = 1; i < sorted_points.size(); ++i) {
            double y_diff = sorted_points[i].y() - sorted_points[i - 1].y();
            if (y_diff > max_y_diff) {
                max_y_diff = y_diff;
                max_y_diff_index = i;
            }
        }

        // 计算中点
        Eigen::Vector3d mid = (sorted_points[max_y_diff_index - 1] + sorted_points[max_y_diff_index]) / 2;
        level1_waypoints.push_back(mid);
    }
}

