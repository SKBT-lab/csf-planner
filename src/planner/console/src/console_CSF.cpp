#include "console/console_CSF.hpp"

void CONSOLE::ControlFSM(const ros::TimerEvent &e){
  int current_mainstate = main_state;
  int current_substate = sub_state;

  // std::lock_guard<std::mutex> lock(csf_mutex);

  if (!has_odom && abs(odom_pos.z()) < fly_altitude) {
    main_state = WAIT_TAKE_OFF_FINISHED;
    if(!has_odom){
      std::cout << "No Odom!" << std::endl;
    }
  }
  else if (has_odom && abs(odom_pos.z()) >= fly_altitude && !task_begin) {
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
        std::vector<Eigen::Vector3d> p_list;
        Eigen::Vector3d p;
        p << odom_pos.x(), odom_pos.y(), odom_pos.z();
        p_list.push_back(p);
        p = start_list[main_state]; 
        p_list.push_back(p);

        Cur_Traj = cs_f.tg.trajGeneration(p_list, ros::Time::now());    
        getVisual(Cur_Traj);    
        traj_ready = true;

        desired_yaw = yaw_list[main_state];    //在某一阶段如需调整yaw角，只需将期望yaw给到desired_yaw，并将rotate_finished置成false即可
        rotate_finished = true;
        traj_rotate = true;
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
        logMessage = "------WAIT_TAKE_OFF_FINISHED!------";
        printf("\033[1;32m%s\033[0m\n", logMessage);
        break;
      
    }
  }

  const char* logMessage2;
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
  // std::lock_guard<std::mutex> lock(csf_mutex);
  if (main_state == WAIT_TAKE_OFF_FINISHED || main_state == INIT) {
    return;
  }
  else if (main_state == LANDING) {
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
        Cur_Traj.start_time = ros::Time::now();
        traj_ready = true;
        Traj_time_init = false;
      }
      flyfinishflag_list[main_state][sub_state] = ControlParse(Cur_Traj, tra_start_time, true, traj_rotate);
      traj_ready = !flyfinishflag_list[main_state][sub_state]; //同步traj_ready，若当前轨迹跟踪完毕，则traj_ready反转，停止planner的更新，等待下一个初始轨迹规划完毕。
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
        // loop.sleep();
      }
      // delay(1000);
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

// void CONSOLE::CSF_Update(const ros::TimerEvent &e){   //单线程原始版
//     // std::lock_guard<std::mutex> lock(csf_mutex);

//   if(traj_ready && tra_start_time.toSec() != 0 && has_depth && has_odom){
//         csf.Input_Depth(Depth_image);
//         csf.Reset_Cloth();
//         csf.Shot();

//         pub_cloud();

//         // double t_now = (ros::Time::now() - tra_start_time).toSec();
//         // if(t_now < Cur_Traj.time_duration){
//         //     std::pair<MiniSnapTraj, bool> traj_info;        
//         //     traj_info = csf.Update_Trajectory(Cur_Traj, odom_q, odom_pos, Hardwaypoints_num);
//         //     Cur_Traj = traj_info.first;
//         //     getVisual(Cur_Traj);
            
//         //     if(traj_info.second){
//         //       Hardwaypoints_num ++;
//         //     }
//         //      csf.Show_Result('H');
//         //      cv::waitKey(1);
//         // }

//         double t_now = (ros::Time::now() - Cur_Traj.start_time).toSec();
//         if(t_now < Cur_Traj.time_duration){  
//             Cur_Traj = csf.Replan_Trajectory(Cur_Traj, odom_q, odom_pos, odom_vel);
//             if(Cur_Traj.replan_flag){
//               getVisual(Cur_Traj);
//             }
            
//             // csf.Show_Result('H');
//             // cv::waitKey(1);
//         }
//     }
// }


// void CONSOLE::CSF_Update(){  //std::thread多线程版
//     // std::lock_guard<std::mutex> lock(csf_mutex);
//   while(true){
//     if(traj_ready && tra_start_time.toSec() != 0 && has_depth && has_odom){
//       csf_mutex.lock();
//       Depth_image_cp = Depth_image;
//       csf_mutex.unlock();
//       csf.Input_Depth(Depth_image_cp);
//       csf.Reset_Cloth();
//       csf.Shot();

//       pub_cloud();

//       // double t_now = (ros::Time::now() - tra_start_time).toSec();
//       // if(t_now < Cur_Traj.time_duration){
//       //     std::pair<MiniSnapTraj, bool> traj_info;        
//       //     traj_info = csf.Update_Trajectory(Cur_Traj, odom_q, odom_pos, Hardwaypoints_num);
//       //     Cur_Traj = traj_info.first;
//       //     getVisual(Cur_Traj);
          
//       //     if(traj_info.second){
//       //       Hardwaypoints_num ++;
//       //     }
//       //      csf.Show_Result('H');
//       //      cv::waitKey(1);
//       // }

//       csf_mutex.lock();
//       Cur_Traj_cp = Cur_Traj;
//       odom_q_cp = odom_q;
//       odom_pos_cp = odom_pos;
//       odom_vel_cp = odom_vel;
//       csf_mutex.unlock();

//       double t_now = (ros::Time::now() - Cur_Traj_cp.start_time).toSec();
//       if(t_now < Cur_Traj_cp.time_duration){  
//           Cur_Traj_cp = csf.Replan_Trajectory(Cur_Traj_cp, odom_q_cp, odom_pos_cp, odom_vel_cp);
//           if(Cur_Traj_cp.replan_flag){
//             getVisual(Cur_Traj_cp);
//           }
//           csf_mutex.lock();
//           Cur_Traj = Cur_Traj_cp;
//           csf_mutex.unlock();
//           // csf.Show_Result('H');
//           // cv::waitKey(1);
//       }
//     }
//   }
//   delay(100);
// }

// void CONSOLE::CSF_Update(const ros::TimerEvent &e){   //双节点版，布料话题独立发布
//     // std::lock_guard<std::mutex> lock(csf_mutex);

//   if(traj_ready && tra_start_time.toSec() != 0 && has_csf && has_odom){

//         double t_now = (ros::Time::now() - Cur_Traj.start_time).toSec();
//         if(t_now < Cur_Traj.time_duration){  
//             Cur_Traj = cs_f.Replan_Trajectory(Cur_Traj, odom_q, odom_pos, odom_vel);
//             if(Cur_Traj.replan_flag){
//               getVisual(Cur_Traj);
//             }
            
//             // csf.Show_Result('H');
//             // cv::waitKey(1);
//         }
//     }
// }

// void CONSOLE::CSF_Update(const ros::TimerEvent &e){   //多节点版，航点发布
//     // std::lock_guard<std::mutex> lock(csf_mutex);
//   if(traj_ready && tra_start_time.toSec() != 0 && has_odom){
//     traj_msg.start_time = Cur_Traj.start_time;
//     traj_msg.traj_nums = Cur_Traj.traj_nums;
//     traj_msg.time_duration = Cur_Traj.time_duration;
//     traj_msg.poly_time = Cur_Traj.poly_time;

//     // 将多项式轨迹系数矩阵展平为一维数组
//     for (const auto& coeff : Cur_Traj.poly_coeff) {
//         for (int i = 0; i < coeff.size(); ++i) {
//             traj_msg.poly_coeff.push_back(coeff[i]);
//         }
//     }

//     // 将路径点转换为 geometry_msgs/Point
//     for (const auto& point : Cur_Traj.waypoints) {
//         geometry_msgs::Point p;
//         p.x = point.x();
//         p.y = point.y();
//         p.z = point.z();
//         traj_msg.waypoints.push_back(p);
//     }

//     // 转换硬路径点
//     traj_msg.hard_waypoints.x = Cur_Traj.hard_waypoints.x();
//     traj_msg.hard_waypoints.y = Cur_Traj.hard_waypoints.y();
//     traj_msg.hard_waypoints.z = Cur_Traj.hard_waypoints.z();

//     // 赋值硬路径点ID和时间
//     traj_msg.hard_waypoints_ID = Cur_Traj.hard_waypoints_ID;
//     traj_msg.hard_waypoints_time = Cur_Traj.hard_waypoints_time;

//     // 转换软路径点列表
//     for (const auto& point : Cur_Traj.soft_waypoints.points) {
//         geometry_msgs::Point p;
//         p.x = point.x();
//         p.y = point.y();
//         p.z = point.z();
//         traj_msg.soft_waypoints_plist.push_back(p);
//     }

//     // 赋值软路径点时间列表
//     traj_msg.soft_waypoints_tlist = Cur_Traj.soft_waypoints.times;

//     // 赋值重规划标志
//     traj_msg.replan_flag = Cur_Traj.replan_flag;
//     traj_pub.publish(traj_msg);
//   }
// }

void CONSOLE::zoom_time(double t0){   
  tv1 = tv2 = t0;
  Traj_info info_v1;
  while(tv1 > 0){
    tv1 -= 0.5;
    info_v1 = cs_f.tg.getTrajInfo(Cur_Traj, tv1);
    if(info_v1.velocity.norm() >= v_thresh){
      break;
    }
  }
  Traj_info info_v2;
  while(tv2 < Cur_Traj.time_duration){
    tv2 += 0.5;
    info_v2 = cs_f.tg.getTrajInfo(Cur_Traj, tv2);
    if(info_v2.velocity.norm() >= v_thresh){
      break;
    }
  }
  double v0 = (info_v2.position - info_v1.position).norm() / (tv2 - tv1);
  zoom_factor = v_desire / v0; //大于1
  dtv = (tv2 - tv1) - (tv2 - tv1) / zoom_factor;
  // std::cout << "tv1: " << tv1 << " " << "tv2: " << tv2 << std::endl;
  // std::cout << "zoom_factor: " << zoom_factor << std::endl;
}

void CONSOLE::CSF_Update(const ros::TimerEvent &e){   //双节点版，布料话题独立发布
    // std::lock_guard<std::mutex> lock(csf_mutex);
  if(cs_f.commu_mode == 0){
    if(traj_ready && tra_start_time.toSec() != 0 && has_csf && has_odom){

        double t_now = (ros::Time::now() - Cur_Traj.start_time).toSec();
        if(t_now < Cur_Traj.time_duration){  
            Cur_Traj = cs_f.Replan_Trajectory(Cur_Traj, odom_q, odom_pos, odom_vel);
            if(Cur_Traj.replan_flag){
              getVisual(Cur_Traj);
              zoom_enable = true;
            }
            if(zoom_enable){
              tv = Cur_Traj.soft_waypoints.times[0];
              zoom_time(tv);
            }
            // csf.Show_Result('H');
            // cv::waitKey(1);
        }
    }
  }

  if(cs_f.commu_mode == 1){
    if(traj_ready && tra_start_time.toSec() != 0 && has_odom){
      traj_msg.start_time = Cur_Traj.start_time;
      traj_msg.traj_nums = Cur_Traj.traj_nums;
      traj_msg.time_duration = Cur_Traj.time_duration;
      traj_msg.poly_time = Cur_Traj.poly_time;

      // 将多项式轨迹系数矩阵展平为一维数组
      for (const auto& coeff : Cur_Traj.poly_coeff) {
          for (int i = 0; i < coeff.size(); ++i) {
              traj_msg.poly_coeff.push_back(coeff[i]);
          }
      }

      // 将路径点转换为 geometry_msgs/Point
      for (const auto& point : Cur_Traj.waypoints) {
          geometry_msgs::Point p;
          p.x = point.x();
          p.y = point.y();
          p.z = point.z();
          traj_msg.waypoints.push_back(p);
      }

      // 转换硬路径点
      traj_msg.hard_waypoints.x = Cur_Traj.hard_waypoints.x();
      traj_msg.hard_waypoints.y = Cur_Traj.hard_waypoints.y();
      traj_msg.hard_waypoints.z = Cur_Traj.hard_waypoints.z();

      // 赋值硬路径点ID和时间
      traj_msg.hard_waypoints_ID = Cur_Traj.hard_waypoints_ID;
      traj_msg.hard_waypoints_time = Cur_Traj.hard_waypoints_time;

      // 转换软路径点列表
      for (const auto& point : Cur_Traj.soft_waypoints.points) {
          geometry_msgs::Point p;
          p.x = point.x();
          p.y = point.y();
          p.z = point.z();
          traj_msg.soft_waypoints_plist.push_back(p);
      }

      // 赋值软路径点时间列表
      traj_msg.soft_waypoints_tlist = Cur_Traj.soft_waypoints.times;

      // 赋值重规划标志
      traj_msg.replan_flag = Cur_Traj.replan_flag;
      traj_pub.publish(traj_msg);
    }
  }
  if(cs_f.commu_mode == 2){
    if(traj_ready && tra_start_time.toSec() != 0 && has_depth && has_odom){
      cs_f.Input_Depth(Depth_image);
      cs_f.Reset_Cloth();
      cs_f.Shot();

      pub_cloud();

      // double t_now = (ros::Time::now() - tra_start_time).toSec();
      // if(t_now < Cur_Traj.time_duration){
      //     std::pair<MiniSnapTraj, bool> traj_info;        
      //     traj_info = csf.Update_Trajectory(Cur_Traj, odom_q, odom_pos, Hardwaypoints_num);
      //     Cur_Traj = traj_info.first;
      //     getVisual(Cur_Traj);
          
      //     if(traj_info.second){
      //       Hardwaypoints_num ++;
      //     }
      //      csf.Show_Result('H');
      //      cv::waitKey(1);
      // }

      double t_now = (ros::Time::now() - Cur_Traj.start_time).toSec();
      if(t_now < Cur_Traj.time_duration){  
          Cur_Traj = cs_f.Replan_Trajectory(Cur_Traj, odom_q, odom_pos, odom_vel);
          if(Cur_Traj.replan_flag){
            getVisual(Cur_Traj);
            zoom_enable = true;
          }
          if(zoom_enable){
            tv = Cur_Traj.soft_waypoints.times[0];
            zoom_time(tv);
          }
          
          // csf.Show_Result('H');
          // cv::waitKey(1);
      }
    }
  }
}


// bool CONSOLE::ControlParse(MiniSnapTraj& trajectory, ros::Time start_time, bool init, bool isAdjustYaw) {
//   std::lock_guard<std::mutex> lock(csf_mutex);
//   ros::Time time_now = ros::Time::now();
//   double t_cur = (time_now - start_time).toSec();
//   bool finished = false;
//   Eigen::Vector3d pos(Eigen::Vector3d::Zero()), vel(Eigen::Vector3d::Zero());
//   std::pair<double, double> yaw_yawdot(0, 0); 
//   static ros::Time time_last = ros::Time::now();

//   if (init) {
//     time_last = ros::Time::now();
//   } 

//   if (t_cur < trajectory.time_duration && t_cur >= 0.0){
//     Traj_info info = csf.tg.getTrajInfo(trajectory, t_cur);
//     pos = info.position;
//     vel = info.velocity;
//     yaw_yawdot = calculate_yaw(info);
//   }
//   else if (t_cur >= trajectory.time_duration){
//     Traj_info info = csf.tg.getTrajInfo(trajectory, trajectory.time_duration);
//     pos = info.position;
//     vel.setZero();
//     yaw_yawdot.first = last_yaw_;
//     yaw_yawdot.second = 0;
//     finished = true; 
//   }
//   else{
//     cout << "[trajectory server]: invalid time." << endl;
//   }

//   time_last = time_now;

//   posiCmd.header.stamp = time_now;
//   posiCmd.header.frame_id = "world";
//   posiCmd.trajectory_flag = quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_READY;
//   posiCmd.trajectory_id = 1;

//   posiCmd.position.x = pos(0);
//   posiCmd.position.y = pos(1);
//   posiCmd.position.z = pos(2);

//   posiCmd.velocity.x = vel(0);
//   posiCmd.velocity.y = vel(1);
//   posiCmd.velocity.z = vel(2);

//   if (isAdjustYaw) {
//     posiCmd.yaw = yaw_yawdot.first;
//   }
//   else {
//     posiCmd.yaw = last_yaw_;
//   }
  
//   // posiCmd.yaw_dot = yaw_yawdot.second;
//   posiCmd.yaw_dot = 0;
//   last_yaw_ = posiCmd.yaw;
//   controlCmd_pub.publish(posiCmd);

//   obj_odom.header.stamp = time_now;
//   obj_odom.header.frame_id = "world";

//   // 设置位置 (x, y, z)
//   obj_odom.pose.pose.position.x = pos(0);
//   obj_odom.pose.pose.position.y = pos(1);
//   obj_odom.pose.pose.position.z = pos(2);

//   Eigen::Matrix3d rotation_matrix = Eigen::AngleAxisd(last_yaw_, Eigen::Vector3d::UnitZ()).toRotationMatrix();
//   Eigen::Quaterniond quat(rotation_matrix);

//   obj_odom.pose.pose.orientation.x = quat.x();
//   obj_odom.pose.pose.orientation.y = quat.y();
//   obj_odom.pose.pose.orientation.z = quat.z();
//   obj_odom.pose.pose.orientation.w = quat.w();

//   obj_pos_pub.publish(obj_odom);

//   return finished;
// }

bool CONSOLE::ControlParse(MiniSnapTraj trajectory, ros::Time start_time, bool init, bool isAdjustYaw) {
  // std::lock_guard<std::mutex> lock(csf_mutex);
  ros::Time time_now = ros::Time::now();
  // double t_cur = (time_now - start_time).toSec();
  double t_cur = (time_now - trajectory.start_time).toSec();
  t_cur += time_forward;
  if(zoom_enable && t_cur >= tv1 && t_cur < tv2 - dtv){
    t_cur = tv1 + (t_cur - tv1) * zoom_factor;
    // std::cout << "zoom!" << std::endl;
  }
  else if(zoom_enable && t_cur >= tv2 - dtv){
    t_cur = t_cur + dtv;
    // std::cout << "stop zoom!" << std::endl;
  }


  bool finished = false;
  Eigen::Vector3d pos(Eigen::Vector3d::Zero()), vel(Eigen::Vector3d::Zero());
  std::pair<double, double> yaw_yawdot(0, 0); 
  static ros::Time time_last = ros::Time::now();

  if (init) {
    time_last = ros::Time::now();
  } 

  if (t_cur < trajectory.time_duration && t_cur >= 0.0){
    Traj_info info = cs_f.tg.getTrajInfo(trajectory, t_cur);
    pos = info.position;
    vel = info.velocity;
    yaw_yawdot = calculate_yaw(info);
  }
  else if (t_cur >= trajectory.time_duration){
    Traj_info info = cs_f.tg.getTrajInfo(trajectory, trajectory.time_duration);
    pos = info.position;
    vel.setZero();
    yaw_yawdot.first = last_yaw_;
    yaw_yawdot.second = 0;
    finished = true; 
  }
  else{
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
  }
  else {
    posiCmd.yaw = last_yaw_;
  }
  
  // posiCmd.yaw_dot = yaw_yawdot.second;
  posiCmd.yaw_dot = 0;
  last_yaw_ = posiCmd.yaw;
  controlCmd_pub.publish(posiCmd);

  obj_odom.header.stamp = time_now;
  obj_odom.header.frame_id = "world";

  // 设置位置 (x, y, z)
  obj_odom.pose.pose.position.x = pos(0);
  obj_odom.pose.pose.position.y = pos(1);
  obj_odom.pose.pose.position.z = pos(2);

  Eigen::Matrix3d rotation_matrix = Eigen::AngleAxisd(last_yaw_, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  Eigen::Quaterniond quat(rotation_matrix);

  obj_odom.pose.pose.orientation.x = quat.x();
  obj_odom.pose.pose.orientation.y = quat.y();
  obj_odom.pose.pose.orientation.z = quat.z();
  obj_odom.pose.pose.orientation.w = quat.w();

  obj_pos_pub.publish(obj_odom);

  return finished;
}

double CONSOLE::clamp(double value, double min, double max) {
    return std::max(min, std::min(value, max));
}

std::pair<double, double> CONSOLE::calculate_yaw(const Traj_info& t_info){
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

void CONSOLE::getVisual(MiniSnapTraj trajectory){
    // std::lock_guard<std::mutex> lock(csf_mutex);
    trajVisual_->displayWaypoints(trajectory.waypoints);
    // std::cout << "WPS_nums: " << trajectory.waypoints.size() << std::endl;
    
    if(trajectory.hard_waypoints_ID != -1){
        std::vector<Eigen::Vector3d> hwps;
        hwps.push_back(trajectory.hard_waypoints);
        trajVisual_->displayHardWaypoints(hwps);
    }
    
    std::vector<Eigen::Vector3d> swps = trajectory.soft_waypoints.points;

    // for(int i = 0; i <  trajectory.soft_waypoints.points.size(); i++){
    //     Traj_info tj_info  = csf.tg.getTrajInfo(trajectory, trajectory.soft_waypoints.times[i]);
    //     swps.push_back(tj_info.position);
    // }
  
    trajVisual_->displaySoftWaypoints(swps);

    // for(int i = 0; i < trajectory.waypoints.size(); i++){
    //   std::cout << i << ": " << trajectory.waypoints[i] << std::endl;
    // }

    // for (size_t i = 0; i < trajectory.soft_waypoints.times.size(); i++)
    // {
    //     std::cout << trajectory.soft_waypoints.times[i] << " ";
    // }
    // std::cout << std::endl;

    double traj_len = 0.0;
    int count = 1;
    Eigen::Vector3d cur, vel;
    cur.setZero();
    geometry_msgs::PoseStamped poses;
    nav_msgs::Path trajPath;
    trajPath.header.frame_id = poses.header.frame_id = "world";
    trajPath.header.stamp    = poses.header.stamp    = ros::Time::now();
    double yaw = 0.0;
    poses.pose.orientation   = tf::createQuaternionMsgFromYaw(yaw);
    // trajPath.poses.clear();
    ros::Rate loop_rate(1000);
    double dt = trajectory.time_duration / 500;
    for(double t = 0.0; t < trajectory.time_duration; t += dt, count++)   // go through each segment
    {   
        auto info = cs_f.tg.getTrajInfo(trajectory, t);
        cur = info.position;
        // vel = info.velocity;
        poses.pose.position.x = cur[0];
        poses.pose.position.y = cur[1];
        poses.pose.position.z = cur[2];
        trajPath.poses.push_back(poses);
        // std::cout << cur << std::endl;
        // std::cout << '###############' << std::endl;
        // loop_rate.sleep();
    } 
    trajPath_pub.publish(trajPath);
}

double CONSOLE::getYawfromQuaternion(const Eigen::Quaterniond& q){
  tf::Quaternion Q(q.x(), q.y(), q.z(), q.w());
  tf::Matrix3x3 m(Q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  return yaw;
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




Eigen::Vector3d CONSOLE::start_forward(double vel){   //用于产生一个朝向正前方的大小为vel的速度向量
  Eigen::Vector3d vel_vec(vel, 0, 0);
  return odom_q * vel_vec;
}




double CONSOLE::Correction_z(double z){
  return z;
}


Eigen::Vector3d CONSOLE::body_to_world(const Eigen::Vector3d& p_b){
  return odom_q * p_b + odom_pos;
}

void CONSOLE::pub_cloud(){
  pcl::PointXYZ point;
  barrier_cloud->clear();
  access_cloud->clear();
  // 布料转化为点云，用于可视化

  // cv::Point2f cen(0.5 * csf.width * csf.Kspars, 0.5 * csf.height * csf.Kspars);
  // Eigen::Vector3d cen_pos = csf.Get_3Dpos(cen, csf.Depth_Cloth(int(0.5 * csf.height), int(0.5 * csf.width)), odom_q, odom_pos);
  // std::cout << "center: " <<  cen_pos << std::endl;

  for(int i = 0; i < cs_f.height; i++){
      for(int j = 0; j < cs_f.width; j++){
          cv::Point2f center(j * cs_f.Kspars, i * cs_f.Kspars);
          Eigen::Vector3d point3d = cs_f.Get_3Dpos(center, cs_f.Depth_Cloth(i, j), odom_q, odom_pos);
          point.x = point3d.x();
          point.y = point3d.y();
          point.z = point3d.z();
          if(cs_f.Coll_Flag(i, j) == 0.0){
              barrier_cloud->points.push_back(point);
          }
          else{
              access_cloud->points.push_back(point);
          }
      }
  }
  barrier_cloud->width = barrier_cloud->points.size();
  barrier_cloud->height = 1; // 表示无序点云
  barrier_cloud->is_dense = true;
  access_cloud->width = access_cloud->points.size();
  access_cloud->height = 1; // 表示无序点云
  access_cloud->is_dense = true;

  sensor_msgs::PointCloud2 pc_ros;
  pcl::toROSMsg(*barrier_cloud, pc_ros);
  pc_ros.header.stamp = ros::Time::now();
  pc_ros.header.frame_id = "world";
  Bpc_pub.publish(pc_ros);
  pcl::toROSMsg(*access_cloud, pc_ros);
  pc_ros.header.stamp = ros::Time::now();
  pc_ros.header.frame_id = "world";
  Hpc_pub.publish(pc_ros);
}