#include <odom_fusion/odom_fusion.h>


void ODOM_FUSION::init(ros::NodeHandle& nh){
    odom_fusion_pub = nh.advertise<odom_fusion::OdomStamp>("/fusion/odom", 50);
    odom_sub        = nh.subscribe("odom", 100, &ODOM_FUSION::OdomCallback, this, ros::TransportHints().tcpNoDelay());
    //TF_timer = nh.createTimer(ros::Duration(0.1), std::bind(&ODOM_FUSION::readLaser, this, std::placeholders::_1));
    std::thread TF_thread(&ODOM_FUSION::readLaser, this);
    TF_thread.detach();  // 分离线程
}

void ODOM_FUSION::OdomCallback(const xv_sdk::PoseStampedConfidence msg){
    

    odom_withH.pose.header = msg.poseMsg.header;
    odom_withH.pose.pose.pose.position.x = msg.poseMsg.pose.position.z;
    odom_withH.pose.pose.pose.position.y = - msg.poseMsg.pose.position.x;
    odom_withH.pose.pose.pose.position.z = - msg.poseMsg.pose.position.y;
    
    Qc.coeffs()[0] = msg.poseMsg.pose.orientation.x;  // 设置虚部 x
    Qc.coeffs()[1] = msg.poseMsg.pose.orientation.y;  // 设置虚部 y
    Qc.coeffs()[2] = msg.poseMsg.pose.orientation.z;  // 设置虚部 z
    Qc.coeffs()[3] = msg.poseMsg.pose.orientation.w;  // 设置实部 w

    Qb = q_CtoB * Qc * q_CtoB.inverse();  //DS80返回的位姿是在相机系下的，需变换到body系
    
    odom_withH.pose.pose.pose.orientation.x = Qb.coeffs()[0];
    odom_withH.pose.pose.pose.orientation.y = Qb.coeffs()[1];
    odom_withH.pose.pose.pose.orientation.z = Qb.coeffs()[2];
    odom_withH.pose.pose.pose.orientation.w = Qb.coeffs()[3];

    odom_withH.tf_height = altitude;

    if(first_pose){
        odom_fusion_pub.publish(odom_withH);
        last_pose = odom_withH;
        first_pose = false;
        return;
    }

    // 计算时间间隔
    ros::Duration dt = msg.poseMsg.header.stamp - last_pose.pose.header.stamp;
    double dt_sec = dt.toSec();

    if (dt_sec > 0) {
        // 计算线速度
        double vx = (odom_withH.pose.pose.pose.position.x - last_pose.pose.pose.pose.position.x) / dt_sec;
        double vy = (odom_withH.pose.pose.pose.position.y - last_pose.pose.pose.pose.position.y) / dt_sec;
        double vz = (odom_withH.pose.pose.pose.position.z - last_pose.pose.pose.pose.position.z) / dt_sec;

        // 计算角速度
        tf::Quaternion q1, q2;
        tf::quaternionMsgToTF(last_pose.pose.pose.pose.orientation, q1);
        tf::quaternionMsgToTF(odom_withH.pose.pose.pose.orientation, q2);

        tf::Quaternion q_diff = q1.inverse() * q2;
        tf::Matrix3x3 m(q_diff);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        double wx = roll / dt_sec;
        double wy = pitch / dt_sec;
        double wz = yaw / dt_sec;
    
        odom_withH.pose.twist.twist.linear.x = vx;
        odom_withH.pose.twist.twist.linear.y = vy;
        odom_withH.pose.twist.twist.linear.z = vz;

        odom_withH.pose.twist.twist.angular.x = wx;
        odom_withH.pose.twist.twist.angular.y = wy;
        odom_withH.pose.twist.twist.angular.z = wz;

    }
    odom_fusion_pub.publish(odom_withH);
    last_pose = odom_withH;

}



void ODOM_FUSION::readLaser(){
    while (true)
    {
        if(serial_port.available()){
            uint8_t buffer[9];
            size_t bytes_read = serial_port.read(buffer, 9);
            // 检查是否成功读取到完整的数据帧
            if (bytes_read != 9){
                std::cout << "Failed to read complete data frame" <<  std::endl;
            }
            else{
                for (size_t i = 0; i < 9; ++i) {
                    if(buffer[i] == 0x59 && buffer[(i + 1) % 9] == 0x59){
                        // 解析距离值
                        uint16_t distance = (buffer[(i + 3) % 9] << 8) | buffer[(i + 2) % 9];
                        tmp_altitude = static_cast<float>(distance) / 100.0;  // 距离值以米为单位
                        if(tmp_altitude != 16.25){
                            altitude = tmp_altitude;
                            // std::cout << altitude <<  std::endl;
                            have_altitude = true;    
                        }
                    }
                }
            }
        }
    }
}    