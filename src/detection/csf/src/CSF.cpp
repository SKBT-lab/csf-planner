#include "CSF.h"



void C_S_FILTER::Input_Depth(cv::Mat Depth){
    CV_Depth_ori = Depth;

    cv::Size imageSize = Depth.size();
    // std::cout << "Depth size (width x height): " << imageSize.width << " x " << imageSize.height << std::endl;  
    
    cv::resize(Depth, CV_Depth, New_Size, 0, 0, cv::INTER_NEAREST);
    
    int startRow = height - height * Kerase;

    
    // // 使用Eigen的Map将cv::Mat数据直接映射为Eigen::MatrixXf
    // Eigen::Map<Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> mat_map(
    //     CV_Depth.ptr<float>(), CV_Depth.rows, CV_Depth.cols);

    // // 将Eigen::MatrixXf转换为Eigen::MatrixXd
    // Depth_Image = mat_map.cast<double>();

    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            Depth_Image(y, x) = static_cast<double>(CV_Depth.at<float>(y, x));
        }
    }
    
    // Set the bottom fifth of the matrix to 100.0
    Depth_Image.block(startRow, 0, int(height * Kerase), width) = Eigen::MatrixXd::Constant(int(height * Kerase), width, 100.0);
}

void C_S_FILTER::Extract_Cloth_fromMsg(const csf::Cloth::ConstPtr& msg){
    int rows = msg->rows;
    int cols = msg->cols;

    Coll_Flag = Eigen::Map<const Eigen::MatrixXd>(msg->Coll_Flag.data(), rows, cols);
    Depth_Image = Eigen::Map<const Eigen::MatrixXd>(msg->Depth_Image.data(), rows, cols);
    Depth_Cloth = Eigen::Map<const Eigen::MatrixXd>(msg->Depth_Cloth.data(), rows, cols);

    // std::cout << Coll_Flag << std::endl;

}

MiniSnapTraj C_S_FILTER::Extract_Traj_fromMsg(const csf::Trajectory::ConstPtr& msg){
    MiniSnapTraj traj;
    traj.start_time = msg->start_time;

    // 赋值分段轨迹数量
    traj.traj_nums = msg->traj_nums;

    // 赋值轨迹总时间
    traj.time_duration = msg->time_duration;

    // 赋值每段轨迹的结束时间点
    traj.poly_time = msg->poly_time;

    // 赋值多项式轨迹系数矩阵
    traj.poly_coeff.clear();
    size_t coeff_size = msg->poly_coeff.size() / traj.traj_nums; // 假设每段轨迹的系数数量相同
    for (size_t i = 0; i < traj.traj_nums; ++i) {
        Eigen::VectorXd coeff(coeff_size);
        for (size_t j = 0; j < coeff_size; ++j) {
            coeff[j] = msg->poly_coeff[i * coeff_size + j];
        }
        traj.poly_coeff.push_back(coeff);
    }

    // 赋值路径点
    traj.waypoints.clear();
    for (const auto& point : msg->waypoints) {
        traj.waypoints.push_back(Eigen::Vector3d(point.x, point.y, point.z));
    }

    // 赋值硬路径点
    traj.hard_waypoints = Eigen::Vector3d(msg->hard_waypoints.x, msg->hard_waypoints.y, msg->hard_waypoints.z);

    // 赋值硬路径点ID和时间
    traj.hard_waypoints_ID = msg->hard_waypoints_ID;
    traj.hard_waypoints_time = msg->hard_waypoints_time;

    // 赋值软路径点列表和时间列表
    traj.soft_waypoints.points.clear();
    for (const auto& point : msg->soft_waypoints_plist) {
        traj.soft_waypoints.points.push_back(Eigen::Vector3d(point.x, point.y, point.z));
    }
    traj.soft_waypoints.times = msg->soft_waypoints_tlist;

    // 赋值重规划标志
    traj.replan_flag = msg->replan_flag;
    return traj;

}

Waypoints C_S_FILTER::Extract_Wps_fromMsg(const csf::Waypoints::ConstPtr& msg){
    Waypoints wps;
    wps.hard_waypoint << msg->hard_waypoint.x, msg->hard_waypoint.y, msg->hard_waypoint.z;
    for(const auto& point : msg->soft_waypoints){
        Eigen::Vector3d soft_wp(point.x, point.y, point.z);
        wps.soft_waypoints.push_back(soft_wp);
    }
    
    return wps;
}


void C_S_FILTER::Reset_Cloth(){
    Min_depth = Depth_Image.minCoeff();
    Depth_Cloth.fill(Min_depth);
    Vel_Cloth.fill(init_vel);
    Coll_Flag.fill(1.0);

    Coll_Flag.row(0).setZero();                // 第一行
    Coll_Flag.row(height - 1).setZero();            // 最后一行
    Coll_Flag.col(0).setZero();                // 第一列
    Coll_Flag.col(width - 1).setZero();            // 最后一列
    

    U_Traction.fill(0.0);
    D_Traction.fill(0.0);
    L_Traction.fill(0.0);
    R_Traction.fill(0.0);

    LU_Shear.fill(0.0);
    RU_Shear.fill(0.0);
    RD_Shear.fill(0.0);
    LD_Shear.fill(0.0);

    U2_Flexion.fill(0.0);
    D2_Flexion.fill(0.0);
    L2_Flexion.fill(0.0);
    R2_Flexion.fill(0.0);

    Spread_Flag.fill(false);

    CV_Cloth.setTo(cv::Scalar(0, 0, 0));

    Hole_Centers.clear();
    Barrier_Centers.clear();

    Hole_3Dpos.clear();
    Barrier_3Dpos.clear();

    for(int i = 0; i < Swps_list.size(); i++){
        Swps_list[i].clear();
        Swps_sum[i] << 0.0, 0.0, 0.0, 0.0;
    }
}


void C_S_FILTER::Cal_force(){

    // Internal_Force = Row_trans_matrix * Depth_Cloth + Depth_Cloth * Col_trans_matrix + Ks * ((U_matrix + D_matrix) * Depth_Cloth * (L_matrix + R_matrix) -  4 * Depth_Cloth);
    
    // array方法计算内力，提高实时性
    U_Traction.fill(0.0);
    D_Traction.fill(0.0);
    L_Traction.fill(0.0);
    R_Traction.fill(0.0);

    LU_Shear.fill(0.0);
    RU_Shear.fill(0.0);
    RD_Shear.fill(0.0);
    LD_Shear.fill(0.0);

    U2_Flexion.fill(0.0);
    D2_Flexion.fill(0.0);
    L2_Flexion.fill(0.0);
    R2_Flexion.fill(0.0);


    // Use array operations for efficiency
    Eigen::ArrayXXd Depth_Cloth_array = Depth_Cloth.array();

    // Up Traction
    U_Traction.block(1, 0, height - 1, width) = (Kt * (Depth_Cloth_array.block(0, 0, height - 1, width) - Depth_Cloth_array.block(1, 0, height - 1, width))).matrix();

    // Down Traction
    D_Traction.block(0, 0, height - 1, width) = (Kt * (Depth_Cloth_array.block(1, 0, height - 1, width) - Depth_Cloth_array.block(0, 0, height - 1, width))).matrix();

    // Left Traction
    L_Traction.block(0, 1, height, width - 1) = (Kt * (Depth_Cloth_array.block(0, 0, height, width - 1) - Depth_Cloth_array.block(0, 1, height, width - 1))).matrix();

    // Right Traction
    R_Traction.block(0, 0, height, width - 1) = (Kt * (Depth_Cloth_array.block(0, 1, height, width - 1) - Depth_Cloth_array.block(0, 0, height, width - 1))).matrix();
    
    // LUtRD_Shear
    LU_Shear.block(1, 1, height - 1, width - 1) = (Ks * (Depth_Cloth_array.block(0, 0, height - 1, width - 1) - Depth_Cloth_array.block(1, 1, height - 1, width - 1))).matrix();

    // RUtLD_Shear
    RU_Shear.block(1, 0, height - 1, width - 1) = (Ks * (Depth_Cloth_array.block(0, 1, height - 1, width - 1) - Depth_Cloth_array.block(1, 0, height - 1, width - 1))).matrix();

    // RDtLU_Shear
    RD_Shear.block(0, 0, height - 1, width - 1) = (Ks * (Depth_Cloth_array.block(1, 1, height - 1, width - 1) - Depth_Cloth_array.block(0, 0, height - 1, width - 1))).matrix();

    // LDtRU_Shear
    LD_Shear.block(0, 1, height - 1, width - 1) = (Ks * (Depth_Cloth_array.block(1, 0, height - 1, width - 1) - Depth_Cloth_array.block(0, 1, height - 1, width - 1))).matrix();

    // U2_Flexion
    U2_Flexion.block(2, 0, height - 2, width) = (Kf * (Depth_Cloth_array.block(0, 0, height - 2, width) - Depth_Cloth_array.block(2, 0, height - 2, width))).matrix();

    // D2_Flexion
    D2_Flexion.block(0, 0, height - 2, width) = (Kf * (Depth_Cloth_array.block(2, 0, height - 2, width) - Depth_Cloth_array.block(0, 0, height - 2, width))).matrix();

    // L2_Flexion
    L2_Flexion.block(0, 2, height, width - 2) = (Kf * (Depth_Cloth_array.block(0, 0, height, width - 2) - Depth_Cloth_array.block(0, 2, height, width - 2))).matrix();

    // R2_Flexion
    R2_Flexion.block(0, 0, height, width - 2) = (Kf * (Depth_Cloth_array.block(0, 2, height, width - 2) - Depth_Cloth_array.block(0, 0, height, width - 2))).matrix();

    // // Up_traction
    // U_Traction = Depth_Cloth;
    // U_Traction.block(0, 0, height - 1, width) = Depth_Cloth.block(1, 0, height - 1, width);
    // U_Traction = Kt * (U_Traction - Depth_Cloth);

    // // Down_traction
    // D_Traction = Depth_Cloth;
    // D_Traction.block(1, 0, height - 1, width) = Depth_Cloth.block(0, 0, height - 1, width);
    // D_Traction = Kt * (D_Traction - Depth_Cloth);

    // // Left_traction
    // L_Traction = Depth_Cloth;
    // L_Traction.block(0, 0, height, width - 1) = Depth_Cloth.block(0, 1, height, width - 1);
    // L_Traction = Kt * (L_Traction - Depth_Cloth);

    // // Right_traction
    // R_Traction = Depth_Cloth;
    // R_Traction.block(0, 1, height, width - 1) = Depth_Cloth.block(0, 0, height, width - 1);
    // R_Traction = Kt * (R_Traction - Depth_Cloth);

    // // LUtRD_Shear
    // LU_Shear = Depth_Cloth;
    // LU_Shear.block(1, 1, height - 1, width - 1) = Depth_Cloth.block(0, 0, height - 1, width - 1);
    // LU_Shear = Ks * (LU_Shear - Depth_Cloth);

    // // RUtLD_Shear
    // RU_Shear = Depth_Cloth;
    // RU_Shear.block(1, 0, height - 1, width - 1) = Depth_Cloth.block(0, 1, height - 1, width - 1);
    // RU_Shear = Ks * (RU_Shear - Depth_Cloth);

    // // RDtLU_Shear
    // RD_Shear = Depth_Cloth;
    // RD_Shear.block(0, 1, height - 1, width - 1) = Depth_Cloth.block(1, 0, height - 1, width - 1);
    // RD_Shear = Ks * (RD_Shear - Depth_Cloth);

    // // LDtRU_Shear
    // LD_Shear = Depth_Cloth;
    // LD_Shear.block(0, 0, height - 1, width - 1) = Depth_Cloth.block(1, 1, height - 1, width - 1);
    // LD_Shear = Ks * (LD_Shear - Depth_Cloth);

    // // U2_Flexion
    // U2_Flexion = Depth_Cloth;
    // U2_Flexion.block(0, 0, height - 2, width) = Depth_Cloth.block(2, 0, height - 2, width);
    // U2_Flexion = Kf * (U2_Flexion - Depth_Cloth);

    // // D2_Flexion
    // D2_Flexion = Depth_Cloth;
    // D2_Flexion.block(2, 0, height - 2, width) = Depth_Cloth.block(0, 0, height - 2, width);
    // D2_Flexion = Kf * (D2_Flexion - Depth_Cloth);

    // // L2_Flexion
    // L2_Flexion = Depth_Cloth;
    // L2_Flexion.block(0, 0, height, width - 2) = Depth_Cloth.block(0, 2, height, width - 2);
    // L2_Flexion = Kf * (L2_Flexion - Depth_Cloth);

    // // R2_Flexion
    // R2_Flexion = Depth_Cloth;
    // R2_Flexion.block(0, 2, height, width - 2) = Depth_Cloth.block(0, 0, height, width - 2);
    // R2_Flexion = Kf * (R2_Flexion - Depth_Cloth);

    // Force
    Internal_Force = U_Traction + D_Traction + L_Traction + R_Traction
                 + LU_Shear + RU_Shear + RD_Shear + LD_Shear
                 + U2_Flexion + D2_Flexion + L2_Flexion + R2_Flexion;

    Force_Matrix = Internal_Force + External_Force - K_drag * Vel_Cloth; 
} 

void C_S_FILTER::Cloth_Collision_Detection(){
   Coll_Distance = Depth_Image - Depth_Cloth;

//     std::cout << Coll_Distance << std::endl;
//    // 遍历矩阵，更新 Barrier_Mask
//     for (int i = 0; i < Coll_Flag.rows(); ++i) {
//         for (int j = 0; j < Coll_Flag.cols(); ++j) {
//             if (Coll_Distance(i, j) < 0.01) {
//                 Coll_Flag(i, j) = 0.0;
//             }
//             else{
//                 Coll_Flag(i, j) = 1.0;
//             }
//         }
//     }
    Coll_Flag = ((Coll_Distance.array() > 0.2) && Coll_Flag.cast<bool>().array()).cast<double>();
    // Coll_Flag = (Coll_Distance.array() > 0.2).cast<double>();
    Depth_Cloth = (Coll_Distance.array() < 0.2).select(Depth_Image, Depth_Cloth);
}

void C_S_FILTER::Update_Cloth(){
    Vel_Cloth = Vel_Cloth + Force_Matrix * dt / m;
    Vel_Cloth = Vel_Cloth.cwiseProduct(Coll_Flag);
    Depth_Cloth = Depth_Cloth + Vel_Cloth * dt;
} 

void C_S_FILTER::Shot(){
    int iteration = 0;
    while (iteration < MaxIterations)
    {
        iteration ++;
        Cloth_Collision_Detection();
        Cal_force();
        Update_Cloth();
        // std::cout << Vel_Cloth << std::endl;
        if (Vel_Cloth.sum() / (Vel_Cloth.size()) < thresh){
            // std::cout << iteration << std::endl;
            break;
        }
    }
    // std::cout <<  iteration << std::endl;
} 


void C_S_FILTER::Barrier_Check(){
    Eigen::MatrixXd Coll_Flag_cp = Coll_Flag;
    // Coll_Flag_cp.row(0) = Coll_Flag_cp.row(1);
    // Coll_Flag_cp.row(height - 1) = Coll_Flag_cp.row(height - 2);
    // Coll_Flag_cp.col(0) = Coll_Flag_cp.col(1);
    // Coll_Flag_cp.col(width - 1) = Coll_Flag_cp.col(width - 2);

    // 将 Eigen::MatrixXd 转换为 Eigen::ArrayXXd，以便使用数组操作
    Eigen::ArrayXXd Coll_Flag_Array = Coll_Flag_cp.array();

    // 创建一个 Eigen::ArrayXXd 掩码矩阵，0.0 的位置为 255，1.0 的位置为 0
    Eigen::ArrayXXd Barrier_Mask_Array = (Coll_Flag_Array == 0.0).cast<double>() * 255.0;

    // 使用 Eigen::Map 将 Eigen::ArrayXXd 数据映射到 cv::Mat
    Eigen::Map<Eigen::Matrix<uchar, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> map(Barrier_Mask.data, Barrier_Mask.rows, Barrier_Mask.cols);

    // 将 Barrier_Mask_Array 转换为 uchar 类型并赋值给 map
    map = Barrier_Mask_Array.cast<uchar>();

    cv::resize(Barrier_Mask, Barrier_Mask_OriSize, Ori_Size, 0, 0, cv::INTER_NEAREST);
    cv::bitwise_not(Barrier_Mask_OriSize, Access_Mask);
    cv::bitwise_and(Access_Mask, Safety_Filter, Access_Mask);
    Hole_Mask = Access_Mask.clone();
}

void C_S_FILTER::Clear_BG(){
    for (int y = 10; y < height_ori; ++y) {
        if (Access_Mask.at<uchar>(y, 10) == 255) {
            cv::floodFill(Hole_Mask, cv::Point(10, y), cv::Scalar(0), 0, cv::Scalar(0), cv::Scalar(0), cv::FLOODFILL_FIXED_RANGE);
            break;
        }
    }
}

void C_S_FILTER::Find_Hole(const Eigen::Quaterniond& q, const Eigen::Vector3d& p){
    Clear_BG();
    // 查找轮廓
    cv::findContours(Hole_Mask, Hole_Cnts, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    // 使用 std::remove_if 和 erase 删除面积小于指定阈值的轮廓
    Hole_Cnts.erase(
        std::remove_if(Hole_Cnts.begin(), Hole_Cnts.end(),
            [this](const std::vector<cv::Point>& contour) {
                return cv::contourArea(contour) < this->Cnt_Area_Threshold;
            }),
        Hole_Cnts.end()
    );

    // 遍历每个轮廓
    for (size_t i = 0; i < Hole_Cnts.size(); i++) {
        // 计算轮廓的矩
        cv::Moments mu = cv::moments(Hole_Cnts[i]);

        // 计算质心
        if (mu.m00 != 0) { // 防止除以零
            cv::Point2f center(mu.m10 / mu.m00, mu.m01 / mu.m00);
            Hole_Centers.push_back(center);
            Hole_3Dpos.push_back(Get_3Dpos(center, Depth_Cloth(int(center.y / Kspars), int(center.x / Kspars)), q, p));
        }
    }
}

void C_S_FILTER::Find_Barrier(const Eigen::Quaterniond& q, const Eigen::Vector3d& p){
    // 查找轮廓
    cv::findContours(Barrier_Mask_OriSize, Barrier_Cnts, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    // 使用 std::remove_if 和 erase 删除面积小于指定阈值的轮廓
    Barrier_Cnts.erase(
        std::remove_if(Barrier_Cnts.begin(), Barrier_Cnts.end(),
            [this](const std::vector<cv::Point>& contour) {
                return cv::contourArea(contour) < this->Cnt_Area_Threshold;
            }),
        Barrier_Cnts.end()
    );
    // 遍历每个轮廓
    for (size_t i = 0; i < Barrier_Cnts.size(); i++) {
        // 计算轮廓的矩
        cv::Moments mu = cv::moments(Barrier_Cnts[i]);

        // 计算质心
        if (mu.m00 != 0) { // 防止除以零
            cv::Point2f center(mu.m10 / mu.m00, mu.m01 / mu.m00);
            Barrier_Centers.push_back(center);
            Barrier_3Dpos.push_back(Get_3Dpos(center, Depth_Cloth(int(center.y / Kspars), int(center.x / Kspars)), q, p));
        }
    }
}


Eigen::Vector3d C_S_FILTER::Get_3Dpos(const cv::Point2f& pix_point, const double& depth, const Eigen::Quaterniond& q, const Eigen::Vector3d& p){ 
    Eigen::Vector3d UV1(pix_point.x, pix_point.y, 1);
    return q * (q_CtoB * (depth * Intri_Matrix_Inverse * UV1) + PC_bias) + p;
    // return q * (q_CtoB * (depth * Intri_Matrix_Inverse * UV1)) + p;
}

void C_S_FILTER::Show_Result(char X){
    // 查找Depth_Cloth中的最小值和最大值
    double min_depth = Depth_Cloth.minCoeff();
    double max_depth = Depth_Cloth.maxCoeff();

    // 遍历Coll_Flag并根据条件进行涂色
    for (int i = 0; i < Coll_Flag.rows(); ++i) {
        for (int j = 0; j < Coll_Flag.cols(); ++j) {
            if (Coll_Flag(i, j) == 0.0) {
                // 涂成红色
                CV_Cloth.at<cv::Vec3b>(i * Kspars, j * Kspars) = cv::Vec3b(0, 0, 255);
            } else {
                // 根据Depth_Cloth的值涂成蓝色
                double depth_value = Depth_Cloth(i, j);
                double normalized_value = (depth_value - min_depth) / (max_depth - min_depth);
                int blue_intensity = static_cast<int>(normalized_value * 255);
                CV_Cloth.at<cv::Vec3b>(i* Kspars, j * Kspars) = cv::Vec3b(blue_intensity, 0, 0);
            }
        }
    }

    // 将32FC1图像归一化并转换为8位无符号整型
    cv::Mat depth32FC1_normalized, depth_8U;
    cv::normalize(CV_Depth_ori, depth32FC1_normalized, 0, 255, cv::NORM_MINMAX);
    depth32FC1_normalized.convertTo(depth_8U, CV_8UC1);

    // 将单通道的8位图像转换为三通道
    cv::Mat depth_8U_color, Mask_color;
    cv::cvtColor(depth_8U, depth_8U_color, cv::COLOR_GRAY2BGR);
    cv::cvtColor(Access_Mask, Mask_color, cv::COLOR_GRAY2BGR);

    std::string Text;
    cv::Point textOrg;
    if(X == 'H'){
        cv::drawContours(Mask_color, Hole_Cnts, -1, cv::Scalar(0, 255, 0), 2); // 在副本图像上绘制轮廓
        for (size_t i = 0; i < Hole_Centers.size(); i++) {
            cv::circle(Mask_color, Hole_Centers[i], 5, cv::Scalar(0, 0, 255), -1);
            Text = "(" + std::to_string(Hole_3Dpos[i][0]).substr(0, 5)  + ", " + std::to_string(Hole_3Dpos[i][1]).substr(0, 5) + ", " + std::to_string(Hole_3Dpos[i][2]).substr(0, 5) + ")";
            textOrg.x = Hole_Centers[i].x - 100;
            textOrg.y = Hole_Centers[i].y + 40;
            cv::putText(Mask_color, Text, textOrg, cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(0, 0, 255), 1);
        }
    }

    else if(X == 'B'){
        cv::drawContours(Mask_color, Barrier_Cnts, -1, cv::Scalar(0, 255, 0), 2); // 在副本图像上绘制轮廓
        for (size_t i = 0; i < Barrier_Centers.size(); i++) {
            cv::circle(Mask_color, Barrier_Centers[i], 5, cv::Scalar(0, 0, 255), -1);
            Text = "(" + std::to_string(Barrier_3Dpos[i][0]).substr(0, 5)  + ", " + std::to_string(Barrier_3Dpos[i][1]).substr(0, 5) + ", " + std::to_string(Barrier_3Dpos[i][2]).substr(0, 5) + ")";
            textOrg.x = Barrier_Centers[i].x - 100;
            textOrg.y = Barrier_Centers[i].y + 40;
            cv::putText(Mask_color, Text, textOrg, cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(0, 0, 255), 1);
        }
    }

    // 并列合并两个图像
    cv::Mat combined1;
    cv::Mat combined2;
    cv::hconcat(depth_8U_color, CV_Cloth, combined1);
    cv::hconcat(combined1, Mask_color, combined2);

    // 显示结果
    cv::imshow("Combined Image", combined2);
    // cv::imshow("Barrier", Barrier_Mask);
    cv::waitKey(5);
}

void C_S_FILTER::Cloth_Visualization(){
    // 遍历Coll_Flag并根据条件进行涂色
    for (int i = 0; i < Coll_Flag.rows(); ++i) {
        for (int j = 0; j < Coll_Flag.cols(); ++j) {
            if (Coll_Flag(i, j) == 1.0 || i == 0 || i == (Coll_Flag.rows() - 1) || j == 0 || j == (Coll_Flag.cols() - 1)) {
                CV_Cloth_Mask.at<uchar>(i, j) = 0;
            } else {
                CV_Cloth_Mask.at<uchar>(i, j) = 255;
            }
        }
    }

    // 执行闭操作（膨胀 -> 腐蚀）
    cv::Mat process_image;
    // cv::morphologyEx(CV_Cloth_Mask, process_image, cv::MORPH_CLOSE, element);
    cv::dilate(CV_Cloth_Mask, process_image, element);

    CV_Cloth_Mask = process_image;

    // // 显示结果
    // cv::imshow("CV_Cloth", CV_Cloth_Mask);
    // // cv::imshow("Barrier", Barrier_Mask);
    // cv::waitKey(1);
}

Eigen::MatrixXd C_S_FILTER::Process_Coll_Flag(){
    Cloth_Visualization(); //利用该函数将Coll_Flag转为图像CV_Cloth_Mask以进行形态学操作
    //再转换回Matrix
    Eigen::MatrixXd result = Coll_Flag;
    for (int i = 0; i < Coll_Flag.rows(); ++i) {
        for (int j = 0; j < Coll_Flag.cols(); ++j) {
            if (CV_Cloth_Mask.at<uchar>(i, j) == 255 || i == 0 || i == (Coll_Flag.rows() - 1) || j == 0 || j == (Coll_Flag.cols() - 1)) {
                result(i, j) == 0.0;
            } else {
                result(i, j) == 1.0;
            }
        }
    }
    return result;
}

void C_S_FILTER::Get_SoftPoints_spread(const cv::Point& p, const cv::Point& init_p, double radius, double d_min, Sliding_mode mode){
    cv::Point dp = init_p - p;
    double dis = cv::norm(dp);

    if(dis > radius){
        return;
    }

    if(!(p.x > 0 && p.x < width && p.y > 0 && p.y < height)){
        return;
    }


    if(Coll_Flag(p.y, p.x) == 0.0){
        return;
    }

    // if((mode == FIND_POINT) && (abs(Depth_Cloth(p.y, p.x) - Min_depth) < 0.2)){
    //     return;
    // }

    if(Spread_Flag(p.y, p.x)){
        return;
    }

    if(mode == FIND_POINT){
        int ID = int((Depth_Cloth(p.y, p.x) - d_min) / Soft_seg_len);
        if(ID < Soft_points_num){
            // Eigen::Vector3d p_d(p.x, p.y, Depth_Cloth(p.y, p.x));
            // Swps_list[ID].push_back(p_d);
            Eigen::Vector4d p_d(p.x, p.y, Depth_Cloth(p.y, p.x), 1);
            Swps_sum[ID] = Swps_sum[ID] + p_d;
        }

        Spread_Flag(p.y, p.x) = true;
        for (const auto& direction : directions) {
            cv::Point p_new(p.x + direction.first, p.y + direction.second);
            //if(Depth_Cloth(p.y, p.x) - Depth_Cloth(p_new.y, p_new.x) >= -0.05){
                Get_SoftPoints_spread(p_new, init_p, radius, d_min, mode);
            //}
            // else{
            //     Spread_Flag(p_new.y, p_new.x) = true;
            // }
        }
    }

    else if(mode == FILL_HOLE){
        Coll_Flag(p.y, p.x) = 0.0;
        Spread_Flag(p.y, p.x) = true;
        for (const auto& direction : directions) {
            cv::Point p_new(p.x + direction.first, p.y + direction.second);
            //if(Depth_Cloth(p.y, p.x) - Depth_Cloth(p_new.y, p_new.x) >= -0.05){
                Get_SoftPoints_spread(p_new, init_p, radius, d_min, mode);
            //}
            // else{
            //     Spread_Flag(p_new.y, p_new.x) = true;
            // }
        }
    }

    
}

bool C_S_FILTER::Point_Check(const cv::Point& point){
    if(point.x >= width || point.y >= height || point.x < 0 || point.y < 0){
        return false;
    }

    else if(Coll_Flag(point.y, point.x) == 0.0){
        return false;
    }
    return true;
}

void C_S_FILTER::Point_Protect(cv::Point& point){
    if(point.x < 0){
        point.x = 0;
    }
    if(point.x > width - 1){
        point.x = width - 1;
    }
    if(point.y < 0){
        point.y = 0;
    }
    if(point.y > height - 1){
        point.y = height - 1;
    }
}

Waypoints C_S_FILTER::Sliding_Point(cv::Point init_point, const Eigen::Quaterniond &q, const Eigen::Vector3d &p){
    std::cout << "sliding start!" << std::endl;
    Waypoints Wps;
    bool Sliding_reset = false;
    // sld_p_img = cv::Mat::zeros(height, width, CV_8UC3);
    Coll_Flag_exp = Process_Coll_Flag();

    // cv::Mat cloth_image(Depth_Cloth.rows(), Depth_Cloth.cols(), CV_64FC1, Depth_Cloth.data());

    // cv::Mat filtered_image;
    // cv::filter2D(cloth_image, filtered_image, -1, Filter_kernel, cv::Point(-1, -1), 0, cv::BORDER_DEFAULT);

    // Eigen::Map<MatrixXd> filtered_eigen(filtered_image.ptr<double>(), filtered_image.rows, filtered_image.cols);
    // Depth_Cloth = filtered_eigen;
 
    cv::cvtColor(CV_Cloth_Mask, sld_p_img, cv::COLOR_GRAY2BGR);
    
    Point_Protect(init_point);

    cv::Point point = init_point;

    Eigen::Vector2d point_vec(double(point.x), double(point.y));
    // std::cout << "!!!!!!" << point.x << " " << point.y << std::endl;
    // std::cout << "!!!" << Coll_Flag.cols() << " " << Coll_Flag.rows() << std::endl;
    if(!Point_Check(point)){
        point.x += 1;                      //给定初始扰动
        point_vec[0] += 1;
    }
    int slid_num = 0;
    while (slid_num < 9999)
    {
        slid_num ++;
        if (Point_Check(point))
        {   
            double Dx = 0.0;
            double Dy = 0.0;
            double Max_d_depth = 0.0;
            for(int dx = -1; dx < 2; dx++){
                for(int dy = -1; dy < 2; dy++){
                    if(dx != 0 && dy != 0 && Coll_Flag_exp(point.y + dy, point.x + dx) == 1.0){
                        double d_depth = Depth_Cloth(point.y + dy, point.x + dx) - Depth_Cloth(point.y, point.x);
                        if(d_depth > Max_d_depth){
                            Max_d_depth = d_depth;
                            Dx = dx;
                            Dy = dy;
                        }
                    }
                }
            }
            if(Max_d_depth == 0.0){
                break;
            }
            else{
                point.x = point.x + Dx;
                point.y = point.y + Dy;
            }
        }

        else{
            cv::Point dp = point - init_point;
            Eigen::Vector2d dp_vec(double(dp.x), double(dp.y));
            Eigen::Vector2d unit_dp_vec = dp_vec.normalized();
            Eigen::Vector2d rotated_dp_vec(unit_dp_vec[1], -unit_dp_vec[0]);
            Eigen::Vector2d d_point = va * unit_dp_vec + vt * rotated_dp_vec;
            point_vec = point_vec + d_point;
            point.x = int(point_vec[0]);
            point.y = int(point_vec[1]);
        }

        if(cv::norm(point - init_point) > std::max(width, height) && !Sliding_reset){  // 若螺旋梯度场迭代发散，则将点重置到画面中央
            std::cout << "Sliding point reset!" << std::endl;
            init_point.x = int(0.5 * width);
            init_point.y = int(0.5 * height);

            if(Coll_Flag_exp(init_point.y, init_point.x) == 0.0){
                point.x = init_point.x + 1;
                point.y = init_point.y + 1;
            }
            else{
                point = init_point;
            }
            Sliding_reset = true;
        }

        if(cv::norm(point - init_point) > std::max(width, height) && Sliding_reset){
            Waypoints wps;
            wps.hard_waypoint << 0, 0, 0;
            std::cout << "Sliding num:" << slid_num << std::endl;
            return wps;
        }
        // sld_p_img.at<cv::Vec3b>(point.y, point.x) = cv::Vec3b(0, 0, 255);
        // std::cout << point << std::endl;
    }

    cv::Point spoint(point.x + 1, point.y);   //用于搜索距离point最近的障碍点
    Eigen::Vector2d spoint_vec(spoint.x, spoint.y);
    // std::cout << "spoint: " << spoint.x << ", " << spoint.y << std::endl;
    while(Coll_Flag(spoint.y, spoint.x) == 1.0){  //螺旋搜索最近的障碍点
        cv::Point dp = spoint - point;
        Eigen::Vector2d dp_vec(double(dp.x), double(dp.y));
        Eigen::Vector2d unit_dp_vec = dp_vec.normalized();
        Eigen::Vector2d rotated_dp_vec(unit_dp_vec[1], -unit_dp_vec[0]);
        Eigen::Vector2d d_point = va * unit_dp_vec + vt * rotated_dp_vec;
        spoint_vec = spoint_vec + d_point;
        spoint.x = int(spoint_vec[0]);
        spoint.y = int(spoint_vec[1]);
        Point_Protect(spoint);
        // std::cout << "spoint: " << spoint.x << ", " << spoint.y << std::endl;
    }

    Soft_seg_len = (Depth_Cloth(point.y, point.x) - Depth_Cloth(spoint.y, spoint.x)) / Soft_points_num;
    
    cv::Point dp_s_p = spoint - point;
    double r = cv::norm(dp_s_p);   //距最近障碍点的像素距离，作为下方递归函数的最远传播距离

    if(r < std::min({safetybox_pixelw, safetybox_pixelh, 0.25 * width, 0.25 * height})){ //当检测到当前通行区域过小以至于无法通行，将该区域填充为障碍区域并返回-1
        std::cout << "Area size is too small: " << r << std::endl;
        Get_SoftPoints_spread(point, point, r, Depth_Cloth(spoint.y, spoint.x), FILL_HOLE);
        // Wps = Sliding_Point(init_point, q, p);
        Eigen::Vector3d h(-1, -1, -1);
        Wps.hard_waypoint = h;
    }

    else{
        Get_SoftPoints_spread(point, point, r, Depth_Cloth(spoint.y, spoint.x), FIND_POINT);

        cv::Point2f h_uv(point.x * Kspars, point.y * Kspars);
        Eigen::Vector3d hwp = Get_3Dpos(h_uv, Depth_Cloth(point.y, point.x), q, p);
        std::vector<Eigen::Vector3d> swps;
        for(int i = 1; i < Soft_points_num; i++){
            if(Swps_sum[i][3] != 0){
                cv::Point2f s_uv(Kspars * Swps_sum[i][0]/Swps_sum[i][3], Kspars * Swps_sum[i][1]/Swps_sum[i][3]);
                double depth = Swps_sum[i][2]/Swps_sum[i][3];
                Eigen::Vector3d Pw = Get_3Dpos(s_uv, depth, q, p);
                swps.push_back(Pw);
            }
            
        }
        Wps.hard_waypoint = hwp;
        Wps.soft_waypoints = swps;
        std::cout << "Sliding point finished" << std::endl;
        
    }

    // cv::Mat mask(height, width, CV_8UC3);
    // for (int i = 0; i < height; ++i) {
    //     for (int j = 0; j < width; ++j) {
    //         uchar value = Spread_Flag(i, j) ? 255 : 0;
    //         mask.at<cv::Vec3b>(i, j) = cv::Vec3b(value, value, value);
    //     }
    // }
    // cv::circle(mask, point, 2, cv::Scalar(0, 255, 0), -1);
    
    // cv::imshow("Mask", mask);
    // cv::imshow("sliding Point", sld_p_img);
    // cv::waitKey(1);
    

    return Wps;
}

Eigen::Vector3d C_S_FILTER::fitLine(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2) {
    double m = (p2.y() - p1.y()) / (p2.x() - p1.x()); // 计算斜率
    double b = p1.y() - m * p1.x(); // 计算截距
    // y = mx + b -> mx - y + b = 0 -> [m, -1, b] * [x, y, 1]^T = 0

    Eigen::Vector3d coeff(m, -1, b);
    return coeff;
}

Eigen::Vector4d C_S_FILTER::fitPlane(const Eigen::Vector3d& p, const Eigen::Vector3d& a) {
    double d = - p.transpose() * a;
    Eigen::Vector4d coeff_plane(a.x(), a.y(), a.z(), d);
    return coeff_plane;
}

double C_S_FILTER::Cal_node_traj_plane(const Eigen::Vector4d& coeff, MiniSnapTraj traj, double t_start){
    double t_low = t_start;
    double t_up = traj.time_duration;

    double dis = std::numeric_limits<double>::max();
    double t = 0;

    int n = 0;
    while(abs(dis) > coll_thresh){
        n++;
        t = 0.5 * (t_low + t_up);
        auto info = tg.getTrajInfo(traj, t);
        Eigen::Vector3d pos = info.position;
        dis = (coeff[0] * pos[0] + coeff[1] * pos[1] + coeff[2] * pos[2] + coeff[3]) / std::sqrt(coeff[0] * coeff[0] + coeff[1] * coeff[1] + coeff[2] * coeff[2]);
        if(dis > 0){
            t_up = t;
        }
        else{
            t_low = t;
        }
        if((abs(t_up - t_low)) < 0.001 && abs(dis) > coll_thresh){  //轨迹全都位于平面的一侧，无交点
            t = -1;
            // std::cout << "n: " << n << " t_diff: " << t_up - t_low << " dis: " << abs(dis) << std::endl;
            break;
        }
    }
    return t;
}

Collision_info C_S_FILTER::Barrier_Collision_Detection(MiniSnapTraj traj, const Eigen::Quaterniond& q, const Eigen::Vector3d& p, double t_start){
    cv::Point2f p1(0, 0);
    Eigen::Vector3d P1 = Get_3Dpos(p1, Depth_Cloth(0, 0), q, p);
    cv::Point2f p2(width_ori, 0);
    Eigen::Vector3d P2 = Get_3Dpos(p2, Depth_Cloth(0, width - 1), q, p);
    cv::Point2f p3(0, height_ori);
    Eigen::Vector3d P3 = Get_3Dpos(p3, Depth_Cloth(height - 1, 0), q, p);

    // 拟合平面方程
    // 计算向量
    Eigen::Vector3d v1 = P2 - P1;
    Eigen::Vector3d v2 = P3 - P1;

    // 计算法向量:朝向轨迹前进方向
    Eigen::Vector3d n = v1.cross(v2);

    // 提取法向量分量
    double A = n.x();
    double B = n.y();
    double C = n.z();

    // 计算 D
    double D = - (A * P1.x() + B * P1.y() + C * P1.z());

    Eigen::Vector4d coeff(A, B, C, D);

    double t = Cal_node_traj_plane(coeff, traj, t_start);
    // std::cout << "t: " << t << std::endl;
    if(t != -1){
        auto info = tg.getTrajInfo(traj, t);
        Eigen::Vector3d obj_pos = info.position;
        std::pair<double, double> y_dy= tg.calculate_yaw(info);
        double obj_yaw = y_dy.first;

        Eigen::AngleAxisd rotation(obj_yaw, Eigen::Vector3d(0, 0, 1));
        Eigen::Matrix3d obj_rotation = rotation.toRotationMatrix();
        
        Eigen::Vector3d Pw_LU = obj_rotation * safetybox_LU + obj_pos;
        Eigen::Vector3d Pw_LD = obj_rotation * safetybox_LD + obj_pos;
        Eigen::Vector3d Pw_RU = obj_rotation * safetybox_RU + obj_pos;
        Eigen::Vector3d Pw_RD = obj_rotation * safetybox_RD + obj_pos;
        
        Eigen::Quaterniond q_inverse = q.inverse();

        Eigen::Vector3d Pc_LU = q_BtoC * q_inverse * (Pw_LU - p - PC_bias);
        Eigen::Vector3d Pc_LD = q_BtoC * q_inverse * (Pw_LD - p - PC_bias);
        Eigen::Vector3d Pc_RU = q_BtoC * q_inverse * (Pw_RU - p - PC_bias);
        Eigen::Vector3d Pc_RD = q_BtoC * q_inverse * (Pw_RD - p - PC_bias);
        
        Eigen::Vector3d UV1_LU = Intri_Matrix * (Pc_LU / Pc_LU[2]) / Kspars;
        Eigen::Vector3d UV1_LD = Intri_Matrix * (Pc_LD / Pc_LD[2]) / Kspars;
        Eigen::Vector3d UV1_RU = Intri_Matrix * (Pc_RU / Pc_RU[2]) / Kspars;
        Eigen::Vector3d UV1_RD = Intri_Matrix * (Pc_RD / Pc_RD[2]) / Kspars;
        UV1_LU.z() = UV1_LD.z() = UV1_RU.z() = UV1_RD.z() = 1.0;

        safetybox_pixelw = (UV1_RU - UV1_LU).norm();
        safetybox_pixelh = (UV1_RU - UV1_RD).norm();

        std::vector<Eigen::Vector3d> box_list;
        box_list.push_back(UV1_LU);
        box_list.push_back(UV1_LD);
        box_list.push_back(UV1_RU);
        box_list.push_back(UV1_RD);

        for(int i = 0; i < box_list.size(); i++){
            // std::cout << "box" << i << ": " << box_list[i][0] << ", " << box_list[i][1] << std::endl;
            if(box_list[i][0] > 0 && box_list[i][0] < width_ori && box_list[i][1] > 0 && box_list[i][1] < height_ori){
                cv::Point center(box_list[i][0] * Kspars, box_list[i][1] * Kspars);
                cv::circle(CV_Cloth, center, 1, cv::Scalar(0, 0, 255), -1);
            }
        }

        Eigen::Vector3d L_coeff = fitLine(UV1_LU, UV1_LD);
        Eigen::Vector3d R_coeff = fitLine(UV1_RU, UV1_RD);
        Eigen::Vector3d U_coeff = fitLine(UV1_LU, UV1_RU);
        Eigen::Vector3d D_coeff = fitLine(UV1_LD, UV1_RD);

        // std::cout << "LU: " << UV1_LU << std::endl;
        // std::cout << "RU: " << UV1_RU << std::endl;
        // std::cout << "LD: " << UV1_LD << std::endl;
        // std::cout << "RD: " << UV1_RD << std::endl;

        // std::cout << "L_coeff: " << L_coeff.transpose() << std::endl;
        // std::cout << "R_coeff: " << R_coeff.transpose() << std::endl;
        // std::cout << "U_coeff: " << U_coeff.transpose() << std::endl;
        // std::cout << "D_coeff: " << D_coeff.transpose() << std::endl;

        int min_x = int(std::min(UV1_LU.x(), UV1_LD.x()));
        int max_x = int(std::max(UV1_RU.x(), UV1_RD.x()));
        int min_y = int(std::min(UV1_LU.y(), UV1_RU.y()));
        int max_y = int(std::max(UV1_RD.y(), UV1_LD.y()));

        int total_num = 0;
        int colli_num = 0; 
        // std::cout << "x: " << min_x << " " << max_x << std::endl;
        // std::cout << "y: " << min_y << " " << max_y << std::endl;
        for(int x = min_x; x <= max_x; x++){
            for(int y = min_y; y <= max_y; y++){
                if(x > 1 && x < width - 2 && y > 1 && y < height - 2){ //排除掉边框，因为边框被常置为碰撞
                    Eigen::Vector3d XY1(x, y, 1);
                    double Lk = x - (y - L_coeff.z()) / L_coeff.x();
                    double Rk = x - (y - R_coeff.z()) / R_coeff.x();

                    double Uk = U_coeff.transpose() * XY1;
                    double Dk = D_coeff.transpose() * XY1;
                    // std::cout << XY1.transpose() << std::endl;
                    // std::cout << Lk << " " << Rk << " " << Uk << " " << Dk << std::endl;
                    if(Lk * Rk <= 0 && Uk * Dk <= 0){   //在四边形四条边之间
                        total_num ++;
                        CV_Cloth.at<cv::Vec3b>(y * Kspars, x * Kspars) = cv::Vec3b(255, 255, 0);
                        if(Coll_Flag(y, x) == 0.0 && Depth_Image(y, x) - Min_depth < coll_dis){
                        //if(Coll_Flag(y, x) == 0.0){
                            colli_num ++;
                        }
                    }
                }
            }
        }

        // Eigen::Vector3d UV1_LU_beforeSpars = Kspars * UV1_LU;
        // Eigen::Vector3d UV1_LD_beforeSpars = Kspars * UV1_LD;
        // Eigen::Vector3d UV1_RU_beforeSpars = Kspars * UV1_RU;
        // Eigen::Vector3d UV1_RD_beforeSpars = Kspars * UV1_RD;
        // Eigen::Vector3d L_coeff = fitLine(UV1_LU_beforeSpars, UV1_LD_beforeSpars);
        // Eigen::Vector3d R_coeff = fitLine(UV1_RU_beforeSpars, UV1_RD_beforeSpars);
        // Eigen::Vector3d U_coeff = fitLine(UV1_LU_beforeSpars, UV1_RU_beforeSpars);
        // Eigen::Vector3d D_coeff = fitLine(UV1_LD_beforeSpars, UV1_RD_beforeSpars);

        // int min_x = int(std::min(UV1_LU.x(), UV1_LD.x()));
        // int max_x = int(std::max(UV1_RU.x(), UV1_RD.x()));
        // int min_y = int(std::min(UV1_LU.y(), UV1_RU.y()));
        // int max_y = int(std::max(UV1_RD.y(), UV1_LD.y()));

        // int total_num = 0;
        // int colli_num = 0; 
        // std::cout << "x: " << min_x << " " << max_x << std::endl;
        // std::cout << "y: " << min_y << " " << max_y << std::endl;
        // for(int x = min_x; x <= max_x; x++){
        //     for(int y = min_y; y <= max_y; y++){
        //         if(x > 0 && x < width - 1 && y > 0 && y < height - 1){
        //             Eigen::Vector3d XY1(Kspars * x, Kspars * y, 1);
        //             double Lk = L_coeff.transpose() * XY1;
        //             double Rk = R_coeff.transpose() * XY1;
        //             double Uk = U_coeff.transpose() * XY1;
        //             double Dk = D_coeff.transpose() * XY1;
        //             if(Lk * Rk <= 0 && Uk * Dk <= 0){   //在四边形四条边之间
        //                 total_num ++;
        //                 CV_Cloth.at<cv::Vec3b>(y * Kspars, x * Kspars) = cv::Vec3b(255, 255, 0);
        //                 if(Coll_Flag(y, x) == 0.0 && Depth_Image(y, x) - Min_depth < coll_dis){
        //                 //if(Coll_Flag(y, x) == 0.0){
        //                     colli_num ++;
        //                 }
        //             }
        //         }
        //     }
        // }

        Eigen::Vector3d obj_UV1(-1, -1, -1);
        // std::cout << "total: " << total_num << std::endl;
        // std::cout << "colli: " << colli_num << std::endl;
        if(colli_num > K_Coll_tolerance * total_num){
            // Eigen::Vector3d obj_pc = q_BtoC * q_inverse * (obj_pos - p);
            // obj_UV1 = Intri_Matrix * (obj_pc / obj_pc[2]) / Kspars;
            obj_UV1 = (UV1_LD + UV1_LU + UV1_RD + UV1_RU) / 4;
            std::cout << "--- Collision detected! ---" << std::endl;
        }
        Collision_info colli_info(obj_UV1, n, t);
        return colli_info;
    }
    else{
        Eigen::Vector3d obj_UV1(-1, -1, -1);
        Collision_info colli_info(obj_UV1, n, t);
        return colli_info;
    }

}

Soft_waypoints C_S_FILTER::Get_Softwps_t(const Eigen::Vector3d& n, MiniSnapTraj traj, const std::vector<Eigen::Vector3d>& soft_waypoints, double t_start){
    Soft_waypoints Swpst;
    if(soft_waypoints.empty()){
        Swpst.enable = false;
    }
    else{
        Swpst.enable = true;
    }
    for(int i = 0; i < soft_waypoints.size(); i++){
        Eigen::Vector4d coeff = fitPlane(soft_waypoints[i], n);
        double t = Cal_node_traj_plane(coeff, traj, t_start);
        Swpst.points.push_back(soft_waypoints[i]);
        Swpst.times.push_back(t);
    }
    return Swpst;
}

double C_S_FILTER::projectionLength(const Eigen::Vector3d& A, const Eigen::Vector3d& B, const Eigen::Vector3d& C) {  
    Eigen::Vector3d AB = B - A;
    Eigen::Vector3d AC = C - A;
    
    double dotProduct = AB.dot(AC);
    double acNorm = AC.norm();
    
    if (acNorm == 0) {
        // 如果AC是零向量，投影长度无意义，可以返回0或抛出异常
        return 0;
    }
    
    return dotProduct / acNorm;
}


std::pair<MiniSnapTraj, bool> C_S_FILTER::Update_Trajectory(MiniSnapTraj traj, const Eigen::Quaterniond& q, const Eigen::Vector3d& p, int hwps_num){
    Collision_info colli_info = Barrier_Collision_Detection(traj, q, p, 0);
    Eigen::Vector4d coeff = fitPlane(p, colli_info.n);
    double t_now = Cal_node_traj_plane(coeff, traj, 0);
    // std::cout << "t_now: " << t_now << std::endl;
    // Collision_info colli_info = Barrier_Collision_Detection(traj, q, p, 0);
    std::pair<MiniSnapTraj, bool> traj_info;
    traj_info.first = traj;
    traj_info.second = false;
    if (colli_info.UV1[0] != -1)
    {   
        
        cv::Point uv(colli_info.UV1.x(), colli_info.UV1.y());
        Waypoints wps;
        wps.hard_waypoint << -1, -1, -1;
        int iter_num = 0;
        while(wps.hard_waypoint.x() == -1 && iter_num < update_num_thresh){
            wps = Sliding_Point(uv, q, p);
            iter_num ++;
        }
         
        if(wps.hard_waypoint.x() == 0 || wps.hard_waypoint.x() == -1){
            std::cout << "trajectory no change" << std::endl;;
            return traj_info;
        }
        // wps.soft_waypoints.push_back((wps.hard_waypoint + traj.waypoints.back()) / 2);
        
        Soft_waypoints swps_t;
        traj_info = tg.waypointInsert(traj, wps.hard_waypoint, colli_info.t, t_now + time_forward, hwps_num);
        MiniSnapTraj traj_no_Swps = traj_info.first;
        swps_t = Get_Softwps_t(colli_info.n, traj_no_Swps, wps.soft_waypoints, t_now);

        if(!swps_t.points.empty()){
            if(projectionLength(wps.hard_waypoint, swps_t.points[0], p) < Soft_enable_thresh){
                swps_t.points.clear();
                swps_t.times.clear();
            }
        }
        
        if((wps.hard_waypoint - traj_no_Swps.waypoints[traj_no_Swps.waypoints.size() - 1]).norm() > guidance_enable_thresh){
            double t = 0;
            swps_t.points.push_back(wps.hard_waypoint + (traj_no_Swps.waypoints[traj_no_Swps.waypoints.size() - 1] - wps.hard_waypoint) * 0.25);
            t = traj_no_Swps.poly_time[traj_no_Swps.traj_nums - 2] + (traj_no_Swps.poly_time[traj_no_Swps.traj_nums - 1] - traj_no_Swps.poly_time[traj_no_Swps.traj_nums - 2]) * 0.25;
            // std::cout << "t1: " << t << std::endl;
            swps_t.times.push_back(t);

            swps_t.points.push_back(wps.hard_waypoint + (traj_no_Swps.waypoints[traj_no_Swps.waypoints.size() - 1] - wps.hard_waypoint) * 0.5);
            t = traj_no_Swps.poly_time[traj_no_Swps.traj_nums - 2] + (traj_no_Swps.poly_time[traj_no_Swps.traj_nums - 1] - traj_no_Swps.poly_time[traj_no_Swps.traj_nums - 2]) * 0.5;
            // std::cout << "t2: " << t << std::endl;
            swps_t.times.push_back(t);
            
            swps_t.points.push_back(wps.hard_waypoint + (traj_no_Swps.waypoints[traj_no_Swps.waypoints.size() - 1] - wps.hard_waypoint) * 0.8);
            t = traj_no_Swps.poly_time[traj_no_Swps.traj_nums - 2] + (traj_no_Swps.poly_time[traj_no_Swps.traj_nums - 1] - traj_no_Swps.poly_time[traj_no_Swps.traj_nums - 2]) * 0.8;
            // std::cout << "t3: " << t << std::endl;
            swps_t.times.push_back(t);
        }
        
        if(!swps_t.points.empty()){
            swps_t.enable = true;
        }
        else{
            swps_t.enable = false;
        }
        
        traj.soft_waypoints = swps_t;
        traj_info = tg.waypointInsert(traj, wps.hard_waypoint, colli_info.t, t_now + time_forward, hwps_num, swps_t);
        
        // for(double t = 0.0; t < traj.time_duration; t += 0.3)   // go through each segment
        // {   
        //     auto info = tg.getTrajInfo(traj, t);
        //     Eigen::Vector3d pos = info.position;
        //     std::cout << t << ": " << pos.x() << " " << pos.y() << " " << pos.z() << std::endl;
        // } 
    }
    return traj_info;

}

MiniSnapTraj C_S_FILTER::Replan_Trajectory(MiniSnapTraj traj, const Eigen::Quaterniond& q, const Eigen::Vector3d& p, const Eigen::Vector3d& v){
    Collision_info colli_info = Barrier_Collision_Detection(traj, q, p, 0);
    // Eigen::Vector4d coeff = fitPlane(p, colli_info.n);
    // double t_now = Cal_node_traj_plane(coeff, traj, 0);

    // Traj_info traj_info_now = tg.getTrajInfo(traj, t_now);
    // Eigen::Vector3d vel_now = traj_info_now.velocity;

    // std::pair<MiniSnapTraj, bool> traj_info;
    // traj_info.first = traj;
    // traj_info.second = false;
    traj.replan_flag = false;
    if (colli_info.UV1[0] != -1)
    {   
        cv::Point uv(colli_info.UV1.x(), colli_info.UV1.y());
        Waypoints wps;
        wps.hard_waypoint << -1, -1, -1;
        int iter_num = 0;
        while(wps.hard_waypoint.x() == -1 && iter_num < update_num_thresh){
            wps = Sliding_Point(uv, q, p);
            iter_num ++;
        }
         
        if(wps.hard_waypoint.x() == 0 || wps.hard_waypoint.x() == -1){
            std::cout << "trajectory no change" << std::endl;
            return traj;
        }
        
        Soft_waypoints swps_t;
        std::vector<Eigen::Vector3d> hwps_list;
        hwps_list.push_back(p);
        hwps_list.push_back(wps.hard_waypoint);
        hwps_list.push_back(traj.waypoints.back());

        MiniSnapTraj traj_no_Swps = tg.trajGeneration(hwps_list, ros::Time::now(), -1, v, Eigen::Vector3d(0.0, 0.0, 0.0), Eigen::Vector3d(0.0, 0.0, 0.0), Eigen::Vector3d(0.0, 0.0, 0.0));
        swps_t = Get_Softwps_t(colli_info.n, traj_no_Swps, wps.soft_waypoints, 0);

        if(!swps_t.points.empty()){
            if(projectionLength(wps.hard_waypoint, swps_t.points[0], p) < Soft_enable_thresh){
                swps_t.points.clear();
                swps_t.times.clear();
            }
        }
        
        if((wps.hard_waypoint - traj_no_Swps.waypoints[traj_no_Swps.waypoints.size() - 1]).norm() > guidance_enable_thresh){
            double t = 0;
            swps_t.points.push_back(wps.hard_waypoint + (traj_no_Swps.waypoints[traj_no_Swps.waypoints.size() - 1] - wps.hard_waypoint) * 0.25);
            t = traj_no_Swps.poly_time[traj_no_Swps.traj_nums - 2] + (traj_no_Swps.poly_time[traj_no_Swps.traj_nums - 1] - traj_no_Swps.poly_time[traj_no_Swps.traj_nums - 2]) * 0.25;
            // std::cout << "t1: " << t << std::endl;
            swps_t.times.push_back(t);

            swps_t.points.push_back(wps.hard_waypoint + (traj_no_Swps.waypoints[traj_no_Swps.waypoints.size() - 1] - wps.hard_waypoint) * 0.5);
            t = traj_no_Swps.poly_time[traj_no_Swps.traj_nums - 2] + (traj_no_Swps.poly_time[traj_no_Swps.traj_nums - 1] - traj_no_Swps.poly_time[traj_no_Swps.traj_nums - 2]) * 0.5;
            // std::cout << "t2: " << t << std::endl;
            swps_t.times.push_back(t);
            
            swps_t.points.push_back(wps.hard_waypoint + (traj_no_Swps.waypoints[traj_no_Swps.waypoints.size() - 1] - wps.hard_waypoint) * 0.8);
            t = traj_no_Swps.poly_time[traj_no_Swps.traj_nums - 2] + (traj_no_Swps.poly_time[traj_no_Swps.traj_nums - 1] - traj_no_Swps.poly_time[traj_no_Swps.traj_nums - 2]) * 0.8;
            // std::cout << "t3: " << t << std::endl;
            swps_t.times.push_back(t);
        }
        
        if(!swps_t.points.empty()){
            swps_t.enable = true;
        }
        else{
            swps_t.enable = false;
        }
        
        traj = tg.trajGeneration(hwps_list, ros::Time::now(), -1, v, Eigen::Vector3d(0.0, 0.0, 0.0), Eigen::Vector3d(0.0, 0.0, 0.0), Eigen::Vector3d(0.0, 0.0, 0.0), swps_t);
        traj.soft_waypoints = swps_t;
        traj.replan_flag = true;
        
        // for(double t = 0.0; t < traj.time_duration; t += 0.3)   // go through each segment
        // {   
        //     auto info = tg.getTrajInfo(traj, t);
        //     Eigen::Vector3d pos = info.position;
        //     std::cout << t << ": " << pos.x() << " " << pos.y() << " " << pos.z() << std::endl;
        // } 
    }
    return traj;
}

Waypoints C_S_FILTER::Update_Waypoints(MiniSnapTraj traj, const Eigen::Quaterniond& q, const Eigen::Vector3d& p){
    Waypoints wps;
    wps.hard_waypoint << -1, -1, -1;
    Collision_info colli_info = Barrier_Collision_Detection(traj, q, p, 0);
    if (colli_info.UV1[0] != -1)
    {   
        cv::Point uv(colli_info.UV1.x(), colli_info.UV1.y());
        int iter_num = 0;
        while(wps.hard_waypoint.x() == -1 && iter_num < update_num_thresh){
            wps = Sliding_Point(uv, q, p);
            iter_num ++;
        }
    }
    return wps;
}