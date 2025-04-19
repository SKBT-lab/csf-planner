#include "console/console_CSF.hpp"

int main(int argc, char** argv){
    ros::init(argc, argv, "console_node");
    ros::NodeHandle nh("~");
    CONSOLE console(320, 240);
    console.init(nh);
    ros::spin();

    //ros::Rate loop_rate(10); // 10 Hz
    return 0;
}

void CONSOLE::init(ros::NodeHandle& nh){
    nh.param("fly_altitude", fly_altitude, 0.5);


    nh.param("landing_height", landing_height, 0.2);

    nh.param("rviz_delay", rviz_delay, false);

    nh.param("stagenum1", stagenum1, 1);
    nh.param("stagenum2", stagenum2, 1);
    nh.param("stagenum3", stagenum3, 5);
    nh.param("stagenum4", stagenum4, 2);
    nh.param("stagenum5", stagenum5, 1);

    stagenum_list.push_back(stagenum1);
    stagenum_list.push_back(stagenum2);
    stagenum_list.push_back(stagenum3);
    stagenum_list.push_back(stagenum4);
    stagenum_list.push_back(stagenum5);

    std::vector<bool> finishflag;

    finishflag.resize(stagenum1, false);
    planfinishflag_list.push_back(finishflag);
    flyfinishflag_list.push_back(finishflag);

    finishflag.resize(stagenum2, false);
    planfinishflag_list.push_back(finishflag);
    flyfinishflag_list.push_back(finishflag);

    finishflag.resize(stagenum3, false);
    planfinishflag_list.push_back(finishflag);
    flyfinishflag_list.push_back(finishflag);

    finishflag.resize(stagenum4, false);
    planfinishflag_list.push_back(finishflag);
    flyfinishflag_list.push_back(finishflag);
    
    finishflag.resize(stagenum5, false);
    planfinishflag_list.push_back(finishflag);
    flyfinishflag_list.push_back(finishflag);

    // finishflag.resize(1, false);
    
    // flyfinishflag_list.push_back(finishflag);
    // flyfinishflag_list.push_back(finishflag);
    // flyfinishflag_list.push_back(finishflag);

    // finishflag.resize(1, true);

    // planfinishflag_list.push_back(finishflag);
    // planfinishflag_list.push_back(finishflag);
    // planfinishflag_list.push_back(finishflag);



    //每一关起点坐标
    nh.param("start1_x", level1_start(0), 0.0);
    nh.param("start1_y", level1_start(1), 0.0);
    nh.param("start1_z", level1_start(2), 1.2);

    nh.param("start2_x", level2_start(0), 0.0);
    nh.param("start2_y", level2_start(1), 0.0);
    nh.param("start2_z", level2_start(2), 1.2);

    nh.param("start3_x", level3_start(0), 0.0);
    nh.param("start3_y", level3_start(1), 0.0);
    nh.param("start3_z", level3_start(2), 1.2);

    nh.param("start4_x", level4_start(0), 0.0);
    nh.param("start4_y", level4_start(1), 0.0);
    nh.param("start4_z", level4_start(2), 1.2);

    nh.param("start5_x", level5_start(0), 0.0);
    nh.param("start5_y", level5_start(1), 0.0);
    nh.param("start5_z", level5_start(2), 1.2);

    start_list.push_back(level1_start);
    start_list.push_back(level2_start);
    start_list.push_back(level3_start);
    start_list.push_back(level4_start);
    start_list.push_back(level5_start);

    nh.param("yaw1", yaw1, 0.0);
    nh.param("yaw2", yaw2, 0.0);
    nh.param("yaw3", yaw3, 0.0);
    nh.param("yaw4", yaw4, 0.0);
    nh.param("yaw5", yaw5, 0.0);

    yaw_list.push_back(yaw1 * M_PI);
    yaw_list.push_back(yaw2 * M_PI);
    yaw_list.push_back(yaw3 * M_PI);
    yaw_list.push_back(yaw4 * M_PI);
    yaw_list.push_back(yaw5 * M_PI);

    bool levelenable;
    nh.param("level1_enable", levelenable, true);
    levelenable_list.push_back(levelenable);
    nh.param("level2_enable", levelenable, false);
    levelenable_list.push_back(levelenable);
    nh.param("level3_enable", levelenable, false);
    levelenable_list.push_back(levelenable);
    nh.param("level4_enable", levelenable, false);
    levelenable_list.push_back(levelenable);
    nh.param("level5_enable", levelenable, false);
    levelenable_list.push_back(levelenable);

    levelenable_list.push_back(true); //对应降落，一定启用

    Eigen::Vector3d tag_to_center;

    nh.param("TagBias0_x", tag_to_center(0), 0.0);
    nh.param("TagBias0_y", tag_to_center(1), 0.0);
    nh.param("TagBias0_z", tag_to_center(2), 0.0);
    tag_to_center_list.push_back(tag_to_center);

    nh.param("TagBias1_x", tag_to_center(0), 0.0);
    nh.param("TagBias1_y", tag_to_center(1), 0.0);
    nh.param("TagBias1_z", tag_to_center(2), 0.0);
    tag_to_center_list.push_back(tag_to_center);

    nh.param("TagBias2_x", tag_to_center(0), 0.0);
    nh.param("TagBias2_y", tag_to_center(1), 0.0);
    nh.param("TagBias2_z", tag_to_center(2), 0.0);
    tag_to_center_list.push_back(tag_to_center);

    nh.param("TagBias3_x", tag_to_center(0), 0.0);
    nh.param("TagBias3_y", tag_to_center(1), 0.0);
    nh.param("TagBias3_z", tag_to_center(2), 0.0);
    tag_to_center_list.push_back(tag_to_center);

    nh.param("TagBias4_x", tag_to_center(0), 0.0);
    nh.param("TagBias4_y", tag_to_center(1), 0.0);
    nh.param("TagBias4_z", tag_to_center(2), 0.0);
    tag_to_center_list.push_back(tag_to_center);

    nh.param("time_forward", time_forward, 0.0);
    nh.param("v_thresh", v_thresh, 0.0);
    nh.param("v_desire", v_desire, 1.0);
    
    //调用PlanningVisualization构造函数进行初始化
    trajVisual_.reset(new PlanningVisualization(nh));

    odom_sub            = nh.subscribe("odom", 10, &CONSOLE::OdomCallback, this, ros::TransportHints().tcpNoDelay());
    depth_sub           = nh.subscribe("depth", 10, &CONSOLE::DepthCallback, this, ros::TransportHints().tcpNoDelay());
    apriltag_sub        = nh.subscribe("/tag_detections", 10, &CONSOLE::ApriltagCallback, this, ros::TransportHints().tcpNoDelay());
    if(cs_f.commu_mode == 1){
        wps_sub             = nh.subscribe("/csf_waypoints", 1, &CONSOLE::wpsCallback, this, ros::TransportHints().tcpNoDelay());
    }
    if(cs_f.commu_mode == 0){
        csf_sub             = nh.subscribe("csf_topic", 1, &CONSOLE::CSFCallback, this, ros::TransportHints().tcpNoDelay());
    }

    trajPath_pub        = nh.advertise<nav_msgs::Path>("trajectory", 1);
    pose_pub            = nh.advertise<geometry_msgs::PoseStamped>("virtual_pose", 10);
    controlCmd_pub      = nh.advertise<quadrotor_msgs::PositionCommand>("/console_position_cmd", 50);
    land_pub            = nh.advertise<quadrotor_msgs::TakeoffLand>("/px4ctrl/takeoff_land", 1);
    Bpc_pub             = nh.advertise<sensor_msgs::PointCloud2>("/barrier_pointcloud", 1);
    Hpc_pub             = nh.advertise<sensor_msgs::PointCloud2>("/hole_pointcloud", 1);
    obj_pos_pub         = nh.advertise<nav_msgs::Odometry>("/obj_pos", 10);
    traj_pub            = nh.advertise<csf::Trajectory>("/console_trajectory", 1);

    ControlCmdCallback_Timer = nh.createTimer(ros::Duration(0.01), &CONSOLE::ControlCmdCallback, this);
    Transformer_Timer        = nh.createTimer(ros::Duration(0.01), &CONSOLE::TransformerCallback, this);
    FSM_Timer                = nh.createTimer(ros::Duration(0.01), &CONSOLE::ControlFSM, this);
    CSF_Timer                = nh.createTimer(ros::Duration(0.1), &CONSOLE::CSF_Update, this);

    Cur_Traj.time_duration = 0;

    // std::thread csf_thread(&CONSOLE::CSF_Update, this);
    // csf_thread.detach();  // 分离线程
}


void CONSOLE::ApriltagCallback(const apriltag_ros::AprilTagDetectionArrayPtr& msg) {
    if (msg->detections.size() != 0) {
        apriltag_detected = true;
        CenterCalbyTag_list.clear();
        
        for (const auto& detection : msg->detections)
        {
            int ID = detection.id[0];
            Eigen::Vector3d tag_pos_C, tag_pos_B, tag_pos_W;  
            tag_pos_C.x() =  detection.pose.pose.pose.position.x;
            tag_pos_C.y() =  detection.pose.pose.pose.position.y;
            tag_pos_C.z() =  detection.pose.pose.pose.position.z;

            tag_pos_B = CtB_R * tag_pos_C + CtB_t;
            tag_pos_W = odom_q * tag_pos_B + odom_pos;

            CenterCalbyTag_list.push_back(tag_pos_W + tag_to_center_list[ID]);
        }

        Eigen::Vector3d sum = Eigen::Vector3d::Zero();
        for (const auto& vec : CenterCalbyTag_list) {
            sum += vec;
        }
        landing_center = sum / CenterCalbyTag_list.size();
        //std::cout << landing_center.x() << "," << landing_center.y() << "," << landing_center.z() << "," << std::endl;
    }
}

void CONSOLE::CSFCallback(const csf::Cloth::ConstPtr& csf_msg) {
   cs_f.Reset_Cloth();
   cs_f.Extract_Cloth_fromMsg(csf_msg);

   has_csf = true;
//    std::cout << "csf received" << std::endl; 
}

void CONSOLE::DepthCallback(const sensor_msgs::Image::ConstPtr& depth_msg) {
   try
    {
        cv_ptr = cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_32FC1);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
   
//    // 处理深度图像中的无穷大值
//     cv::MatIterator_<float> it, end;
//     for (it = cv_ptr->image.begin<float>(), end = cv_ptr->image.end<float>(); it != end; ++it)
//     {
//         if (std::isinf(*it))
//         {
//             *it = 100.0f;
//         }
//     }

    Depth_image = cv_ptr->image.clone(); 
    // std::cout << Depth_image.at<float>(143, 128) << std::endl;
    // std::cout << Depth_image.rows << ", " << Depth_image.cols << std::endl;
    has_depth = true;
}

void CONSOLE::TransformerCallback(const ros::TimerEvent &e){
    geometry_msgs::TransformStamped transform;
    transform.header.stamp = ros::Time::now();
    transform.header.frame_id = "world";
    transform.child_frame_id = "body";
    transform.transform.translation.x = odom_pos.x();  // 单位平移x坐标
    transform.transform.translation.y = odom_pos.y();  // 单位平移y坐标
    transform.transform.translation.z = odom_pos.z();  // 单位平移z坐标
    transform.transform.rotation.x = odom_q.x();     // 单位四元数x分量
    transform.transform.rotation.y = odom_q.y();     // 单位四元数y分量
    transform.transform.rotation.z = odom_q.z();     // 单位四元数z分量
    transform.transform.rotation.w = odom_q.w();     // 单位四元数w分量
    broadcaster.sendTransform(transform);     
}

void CONSOLE::OdomCallback(const odom_fusion::OdomStamp::ConstPtr& msg){
    odom_pos(0) = msg->pose.pose.pose.position.x;
    odom_pos(1) = msg->pose.pose.pose.position.y;
    odom_pos(2) = msg->pose.pose.pose.position.z;

    odom_vel(0) = msg->pose.twist.twist.linear.x;
    odom_vel(1) = msg->pose.twist.twist.linear.y;
    odom_vel(2) = msg->pose.twist.twist.linear.z;

    odom_q.x() = msg->pose.pose.pose.orientation.x;
    odom_q.y() = msg->pose.pose.pose.orientation.y;
    odom_q.z() = msg->pose.pose.pose.orientation.z;
    odom_q.w() = msg->pose.pose.pose.orientation.w;

    TF_height = msg->tf_height;
    // double yaw = atan2(2 * (odom_q.x()*odom_q.y() + odom_q.w()*odom_q.z()), odom_q.w()*odom_q.w() + odom_q.x()*odom_q.x() - odom_q.y()*odom_q.y() - odom_q.z()*odom_q.z());
    // cout << "yaw:" << yaw << endl;
    has_odom = true;

    // transform.setOrigin(tf::Vector3(msg->pose.pose.pose.position.x, msg->pose.pose.pose.position.y, msg->pose.pose.pose.position.z));
    // tf::quaternionMsgToTF(msg->pose.pose.pose.orientation, tf_q);
    // transform.setRotation(tf_q);

    // // 发布坐标变换
    // br.sendTransform(tf::StampedTransform(transform, msg->pose.header.stamp, "world", "body"));
}

void CONSOLE::wpsCallback(const csf::Waypoints::ConstPtr& wps_msg){
    Cur_Traj.replan_flag = false;
    Waypoints wps = cs_f.Extract_Wps_fromMsg(wps_msg);
    if(wps.hard_waypoint.x() == 0 || wps.hard_waypoint.x() == -1){
        std::cout << "trajectory no change" << std::endl;
        return;
    }
    else{
        std::vector<Eigen::Vector3d> hwps_list;
        hwps_list.push_back(odom_pos);
        hwps_list.push_back(wps.hard_waypoint);
        hwps_list.push_back(Cur_Traj.waypoints.back());

        Soft_waypoints swps_t;
        
        MiniSnapTraj traj_no_Swps = cs_f.tg.trajGeneration(hwps_list, ros::Time::now(), -1, odom_vel, Eigen::Vector3d(0.0, 0.0, 0.0), Eigen::Vector3d(0.0, 0.0, 0.0), Eigen::Vector3d(0.0, 0.0, 0.0));
        // swps_t = Get_Softwps_t(colli_info.n, traj_no_Swps, wps.soft_waypoints, 0);

        // if(!swps_t.points.empty()){
        //     if(projectionLength(wps.hard_waypoint, swps_t.points[0], p) < Soft_enable_thresh){
        //         swps_t.points.clear();
        //         swps_t.times.clear();
        //     }
        // }

        if((wps.hard_waypoint - traj_no_Swps.waypoints[traj_no_Swps.waypoints.size() - 1]).norm() > cs_f.guidance_enable_thresh){
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

        Cur_Traj = cs_f.tg.trajGeneration(hwps_list, ros::Time::now(), -1, odom_vel, Eigen::Vector3d(0.0, 0.0, 0.0), Eigen::Vector3d(0.0, 0.0, 0.0), Eigen::Vector3d(0.0, 0.0, 0.0), swps_t);
        Cur_Traj.soft_waypoints = swps_t;
        Cur_Traj.replan_flag = true;
        getVisual(Cur_Traj);
    }
}




