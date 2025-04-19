#include "console/console.hpp"

void CONSOLE::ControlFSM(const ros::TimerEvent &e){
  int current_mainstate = main_state;
  int current_substate = sub_state;

  if (!has_odom && TF_height < fly_altitude) {
    main_state = WAIT_TAKE_OFF_FINISHED;
  }
  else if (has_odom && odom_pos.z() >= fly_altitude && !task_begin) {
    main_state = LEVEL1;
    while (!levelenable_list[main_state]) //跳过禁用的关卡
    {
      nextLevel(main_state);
    }

    sub_state = Stage_1;
    task_begin = true;
  }

  else if (main_state == LEVEL5 && TF_height < (landing_height + 0.01)) {  //高度小于0.2时直接切降落
    main_state = LANDING;
  }

  else if(main_state != WAIT_TAKE_OFF_FINISHED && main_state != INIT && main_state != LANDING){
    if(flyfinishflag_list[main_state][stagenum_list[main_state] - 1] && rotate_finished){  //切换下一关卡
      nextLevel(main_state);
      while (!levelenable_list[main_state]) 
      {
        nextLevel(main_state);
      }
      sub_state = Stage_1;
    }
    
    else if(flyfinishflag_list[main_state][sub_state] && rotate_finished){         //切换下一阶段
      nextStage(sub_state);
    }
    else if(!planfinishflag_list[main_state][sub_state]){                //当前阶段还未规划,另：每次规划只执行一次
      //-----LEVEL1-----//
      if(main_state == LEVEL1 && sub_state == Stage_1){               //遍历每个关卡的每个阶段
        YAML::Node config;
        config = YAML::LoadFile("/home/skbt/SKBT_Drone/src/planner/console/config/8.yaml");
        double dis_1 = config["dis_1"].as<double>();
        double dis_1to2 = config["dis_1to2"].as<double>();
        double h = config["h"].as<double>();
        double r = config["r"].as<double>();
        int dir = config["dir"].as<int>();
        max_num = config["max_num"].as<int>();
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


        Cur_Traj = TG.trajGeneration(waypoints, ros::Time::now(), -1, Eigen::Vector3d(0.0, dir * 0.5, 0), Eigen::Vector3d(0.0, dir * 0.5, 0));
        traj_rotate = false;
        getVisual(Cur_Traj, Waypoints_matrix);
      }
     
      Traj_time_init = true;       //时间戳初始化标志位优先于规划结束标志位置成true
      planfinishflag_list[main_state][sub_state] = true;              
    }
  }
    
    
  
  const char* logMessage;
  // 检查状态转换，并使用 `ROS_WARN` 只在状态改变时打印。
  if (current_mainstate != main_state) {
    switch (main_state) {
      case INIT:
        logMessage = "------INIT------";
        printf("\033[1;32m%s\033[0m\n", logMessage);
        break;
      case LEVEL1:
        logMessage = "------LEVEL1------";
        printf("\033[1;32m%s\033[0m\n", logMessage);
        break;
      case LEVEL2:
        logMessage = "------LEVEL2------";
        printf("\033[1;32m%s\033[0m\n", logMessage);
        break;
      case LEVEL3:
        logMessage = "------LEVEL3------";
        printf("\033[1;32m%s\033[0m\n", logMessage);
        break;
      case LEVEL4:
        logMessage = "------LEVEL4------";
        printf("\033[1;32m%s\033[0m\n", logMessage);
        break;
      case LEVEL5:
        logMessage = "------LEVEL5------";
        printf("\033[1;32m%s\033[0m\n", logMessage);
        break;
      case LANDING:
        logMessage = "------LANDING------";
        printf("\033[1;32m%s\033[0m\n", logMessage);
        break;
      case WAIT_TAKE_OFF_FINISHED:
        logMessage = "------WAIT_TAKE_OFF_FINISHED------";
        printf("\033[1;32m%s\033[0m\n", logMessage);
        break;
      
    }
  }

  // 检查状态转换，并使用 `ROS_WARN` 只在状态改变时打印。
  if (current_substate != sub_state) {
    switch (sub_state) {
      case Stage_1:
        logMessage = "Stage_1";
        printf("\033[1;32m%s\033[0m\n", logMessage);
        break;
      case Stage_2:
        logMessage = "Stage_2";
        printf("\033[1;32m%s\033[0m\n", logMessage);
        break;
      case Stage_3:
        logMessage = "Stage_3";
        printf("\033[1;32m%s\033[0m\n", logMessage);
        break;
      case Stage_4:
        logMessage = "Stage_4";
        printf("\033[1;32m%s\033[0m\n", logMessage);
        break;
      case Stage_5:
        logMessage = "Stage_5";
        printf("\033[1;32m%s\033[0m\n", logMessage);
        break;
      case Stage_6:
        logMessage = "Stage_6";
        printf("\033[1;32m%s\033[0m\n", logMessage);
        break;
    }
  }
  
}


void CONSOLE::ControlCmdCallback(const ros::TimerEvent &e)
{
  if (main_state == WAIT_TAKE_OFF_FINISHED || main_state == INIT) {
    return;
  }
  else if (main_state == LANDING) {  //降落优先级最高
    if (!is_land){
      ros::Rate wait_cmd(100);
      for (int i = 0; i < 50; i++) wait_cmd.sleep();
      quadrotor_msgs::TakeoffLand cmd;
      cmd.takeoff_land_cmd = 2;
      land_pub.publish(cmd);
      is_land = true;
    }
  }
  else if(!(flyfinishflag_list[main_state][sub_state] && rotate_finished) && planfinishflag_list[main_state][sub_state]){ //规划完毕但是还没飞完
    if(!flyfinishflag_list[main_state][sub_state]){
      if(Traj_time_init){
        tra_start_time = ros::Time::now();
        Traj_time_init = false;
      }
      // flyfinishflag_list[main_state][sub_state] = ControlParse(Cur_Traj, tra_start_time, true, traj_rotate);
      int num = ControlParse_for_8(Cur_Traj, tra_start_time, true, traj_rotate);
      if(cur_num != num){
        cur_num = num;
        std::cout << cur_num << std::endl;
      }
      if(cur_num >= max_num){
        main_state = LANDING;
        const char* logMessage;
        logMessage = "Landing";
        printf("\033[1;32m%s\033[0m\n", logMessage);
        delay(1000);
      }
    }
    else if(!rotate_finished){   //用else if，优先跟轨迹，而后调整yaw角
      std::cout << "Rotating to " << desired_yaw * (180.0 / M_PI) << " ......" << std::endl;
      ros::Rate loop(100);

      double diff_yaw = (desired_yaw - last_yaw_);
      if(abs(diff_yaw) > M_PI){
        if(diff_yaw > 0){
          diff_yaw = - (2 * M_PI - abs(diff_yaw));
        }

        else if(diff_yaw < 0){
          diff_yaw = 2 * M_PI - abs(diff_yaw);
        }
      }
      diff_yaw = diff_yaw / 100;

      for (int i = 1; i <= 100; i++) {
        posiCmd.yaw = last_yaw_ + diff_yaw;
        posiCmd.yaw_dot = 0;
        controlCmd_pub.publish(posiCmd);
        last_yaw_ = posiCmd.yaw;
        loop.sleep();
      }
      delay(1000);
      std::cout << "Rotation finished" << std::endl;
      rotate_finished = true;
    }
  }  
}


  // if (state == STATE::FLY_TO_STRAT) {

  //   //使用minimunmsnap
  //   static ros::Time fisrt_tra_start_time = ros::Time::now();
  //   static bool FLY_TO_STRAT_INIT = true;
  //   //飞向起点的过程不进行转向
  //   ControlParse(Traj_start, fisrt_tra_start_time, FLY_TO_STRAT_INIT, start_trajectory_finished, true);
  //   FLY_TO_STRAT_INIT = false;
  // }

  
  // if (state == STATE::LANDING) {
  //   if (!is_land){
  //     ros::Rate wait_cmd(100);
  //     for (int i = 0; i < 50; i++) wait_cmd.sleep();
  //     quadrotor_msgs::TakeoffLand cmd;
  //     cmd.takeoff_land_cmd = 2;
  //     land_pub.publish(cmd);
  //     is_land = true;
  //   }
  // }



int CONSOLE::ControlParse_for_8(MiniSnapTraj& trajectory, ros::Time start_time, bool init, bool isAdjustYaw) {
    ros::Time time_now = ros::Time::now();
    double t_cur = (time_now - start_time).toSec();
    bool finished = false;
    Eigen::Vector3d pos(Eigen::Vector3d::Zero()), vel(Eigen::Vector3d::Zero());
    std::pair<double, double> yaw_yawdot(0, 0); 
    static ros::Time time_last = ros::Time::now();

    if (init) {
      time_last = ros::Time::now();
    } 

    if (t_cur >= 0.0){
      Traj_info info = TG.getTrajInfo(trajectory, t_cur - int(t_cur / trajectory.time_duration) * trajectory.time_duration);
      pos = info.position;
      vel = info.velocity;
      yaw_yawdot = calculate_yaw(info);
    }else{
      cout << "[trajectory server]: invalid time." << endl;
    }

    time_last = time_now;

    posiCmd.header.stamp = time_now;
    posiCmd.header.frame_id = "world";
    posiCmd.trajectory_flag = quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_READY;
    posiCmd.trajectory_id = 1;

    posiCmd.position.x = pos(0);
    posiCmd.position.y = pos(1);
    posiCmd.position.z = pos(2);

    posiCmd.velocity.x = vel(0);
    posiCmd.velocity.y = vel(1);
    posiCmd.velocity.z = vel(2);

    if (isAdjustYaw) {
      posiCmd.yaw = yaw_yawdot.first;
    }else {
      posiCmd.yaw = last_yaw_;
    }
    
    // posiCmd.yaw_dot = yaw_yawdot.second;
    posiCmd.yaw_dot = 0;
    last_yaw_ = posiCmd.yaw;
    controlCmd_pub.publish(posiCmd);
    return int(t_cur / trajectory.time_duration);
}

double CONSOLE::clamp(double value, double min, double max) {
    return std::max(min, std::min(value, max));
}

std::pair<double, double> CONSOLE::calculate_yaw(Traj_info& t_info){
  Vector3d pos = t_info.position;
  Vector3d vel = t_info.velocity;
  Vector3d acc = t_info.acceleration;

  double dx = vel.x();
  double dy = vel.y();

  double ddx = acc.x();
  double ddy = acc.y();

  double yaw = atan(dy / dx);
  double dyaw = 1 / (1 + (dy / dx) * (dy / dx)) * (ddy / dx - dy * ddx * dx / (dx * dx));

  dyaw = clamp(dyaw, -1.5, 1.5);

  if(dx < 0){               //由于反正切的值域是-PI/2到PI/2,而dx<0的部分在该范围之外，故额外叠加个PI
        yaw = yaw - M_PI;   //根据任务流程，机体优先顺时针旋转，故减PI而不是加PI
    }

  std::pair<double, double> yaw_dyaw;
  yaw_dyaw.first = yaw;
  yaw_dyaw.second = dyaw;
  return yaw_dyaw;  
}


void CONSOLE::getVisual(MiniSnapTraj& trajectory, MatrixXd wps){
    std::vector<Eigen::Vector3d> waypoint_list;

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

    ros::Rate loop_rate(100);
    for(double t = 0.0; t < trajectory.time_duration - 0.01; t += 0.01, count++)   // go through each segment
    {   
        Traj_info info = TG.getTrajInfo(trajectory, t);
        cur = info.position;
        vel = info.velocity;
        auto yaw_dyaw = calculate_yaw(info);
        yaw = yaw_dyaw.first;
        poses.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
        poses.pose.position.x = cur[0];
        poses.pose.position.y = cur[1];
        poses.pose.position.z = cur[2];
        trajPath.poses.push_back(poses);
        // std::cout << cur << std::endl;
        // std::cout << '###############' << std::endl;
        trajPath_pub.publish(trajPath);
        pose_pub.publish(poses);
        if (count % 1000 == 0) traj_len += (pre - cur).norm();
        pre = cur;
        if(rviz_delay){
          loop_rate.sleep();
        }
        
    }
}

bool CONSOLE::Find_circle(){
  csf.Input_Cloud(Cloud, 0.01, 8, -2, 2, -0.5, 2.5);
  csf.Reset_Cloth();
  csf.DownSampling();
  csf.Euclidean_Clustering();
  if(csf.Clusters_info.size() != 0){
    for (size_t i = 0; i < csf.Clusters_info.size(); ++i)
    {   
      if(csf.Clusters_info[i].span_y > 1.0 && csf.Clusters_info[i].span_y < 2.0 && csf.Clusters_info[i].span_z > 0.5 && csf.Clusters_info[i].span_z < 2.0){
        circle1_center << csf.Clusters_info[0].center.x(), csf.Clusters_info[0].center.y(), csf.Clusters_info[0].center.z();
        circle1_center = odom_q * circle1_center + odom_pos;
        return true;
      }
    }
  }

  else{
    return false;
  }
}
double CONSOLE::getYawfromQuaternion(Eigen::Quaterniond& q){
  tf::Quaternion Q(q.x(), q.y(), q.z(), q.w());
  tf::Matrix3x3 m(Q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  return yaw;
}

bool CONSOLE::Find_suidao(){
  csf.Input_Cloud(Cloud);
  csf.Reset_Cloth();
  csf.DownSampling();
  csf.Euclidean_Clustering();
  if(csf.Clusters_info.size() != 0){
    for (size_t i = 0; i < csf.Clusters_info.size(); ++i)
    {   
      if(csf.Clusters_info[i].span_y > 1.0 && csf.Clusters_info[i].span_y < 2.0 && csf.Clusters_info[i].span_z > 0.5 && csf.Clusters_info[i].span_z < 2.0){
        csf.Find_suidao(csf.Clusters_info[i]);
        suidao_center << csf.Suidao_center.x(), csf.Suidao_center.y(), csf.Suidao_center.z();
        suidao_center = odom_q * suidao_center + odom_pos;
        double yaw = getYawfromQuaternion(odom_q); 
        suidao_angle = yaw - (M_PI / 2 - csf.Suidao_angle);
        return true;
      }
    }
  }

  else{
    return false;
  }
}

bool CONSOLE::Find_window(int k, double dis){ //k=1检外窗，k=2检测内窗
  csf.Input_Cloud(Cloud, 0.01, dis, -3, 3, -3, 3);
  csf.Reset_Cloth();
  if(k == 1){
    csf.Find_wall();
    return csf.Get_Wallimg(csf.Wall_Cloud);
  }
  else{
    return csf.Get_Wallimg(csf.Cloud);
  }
  
  
}

void CONSOLE::delay(int milliseconds) {
    std::this_thread::sleep_for(std::chrono::milliseconds(milliseconds));
}

void CONSOLE::nextLevel(Main_STATE& m) {
  int temp = static_cast<int>(m);
  temp = temp + 1;
  m = static_cast<Main_STATE>(temp);
}

void CONSOLE::nextStage(Sub_STATE& m) {
  int temp = static_cast<int>(m);
  temp = temp + 1;
  m = static_cast<Sub_STATE>(temp);
}

Eigen::MatrixXd CONSOLE::vectorToMatrix(const std::vector<Eigen::Vector3d>& vec) {
    int n = vec.size();
    Eigen::MatrixXd mat(n, 3);

    for (int i = 0; i < n; ++i) {
        mat.row(i) = vec[i];
    }

    return mat;
}


Eigen::Vector3d CONSOLE::start_forward(double vel){   //用于产生一个朝向正前方的大小为vel的速度向量
  Eigen:Vector3d vel_vec(vel, 0, 0);
  return odom_q * vel_vec;
}

double CONSOLE::Get_right_distance(){
  pcl::PointCloud<pcl::PointXYZI>::Ptr r_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  csf.applyPassThroughFilter(Cloud, r_cloud, -0.5, 0.5, -10.0, 0.0, -0.5, 0.5);
  double r_dis = std::numeric_limits<double>::max();

  for (const auto& point : r_cloud->points)
  {
    if (point.y < r_dis){
          r_dis = point.y;
    }
  }
  return r_dis;
}

double CONSOLE::Get_left_distance(){
  pcl::PointCloud<pcl::PointXYZI>::Ptr l_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  csf.applyPassThroughFilter(Cloud, l_cloud, -0.5, 0.5, 0.0, 10.0, -0.5, 0.5);
  double l_dis = std::numeric_limits<double>::min();

  for (const auto& point : l_cloud->points)
  {
    if (point.y > l_dis){
          l_dis = point.y;
    }
  }
  return l_dis;
}

double CONSOLE::Correction_z(double z){
  return z;
}

int CONSOLE::Find_trees(){
  csf.Input_Cloud(Cloud, 0.01, 7, -3, 3, 0, 2);
  csf.Reset_Cloth();

  csf.DownSampling();
  csf.Euclidean_Clustering();
  if(csf.Clusters_info.size() > 1){
    csf.process_level1();
  }
  return csf.Clusters_info.size();
}

Eigen::Vector3d CONSOLE::body_to_world(Eigen::Vector3d& p_b){
  return odom_q * p_b + odom_pos;
}