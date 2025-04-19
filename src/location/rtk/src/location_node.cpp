#include <rtk/location.h>
using namespace std;

int main(int argc, char** argv){
    ros::init(argc, argv, "location_node");
    ros::NodeHandle nh("~");
    LOCATION loc;
    loc.init(nh);
    ros::spin();
    return 0;
}


/*
**初始化函数
**读RTK的串口数据
**读激光测距的数据
**读mavros传回的飞机姿态数据和气压计信息
*/
void LOCATION::init(ros::NodeHandle& nh){

    // WGS84_2_world();
    // // baro_alti_sub   = nh.subscribe("/mavros/altitude", 1, &LOCATION::processBaroCallback, this);
    // PX4_odom_data_sub = nh.subscribe("/mavros/local_position/odom", 1, &LOCATION::processPX4Odometry, this);
    // odom_pub          = nh.advertise<nav_msgs::Odometry>("fusion/odom", 10);

    // processData_timer_ = nh.createTimer(ros::Duration(0.1), &LOCATION::processDataCallback, this);

    /********这里使用线程来处理而不是ros中的定时器（timer），原因是这两个功能都是与硬件完成数据交互，需要长时间运行保证稳定性*******/
    std::thread rtk_thread(&LOCATION::read_rtk_via_uart, this);
    rtk_thread.detach();                                            //分离线程在后台运行
    // std::thread laser_thread(&LOCATION::read_laser_via_uart, this);
    // laser_thread.detach();

}

void LOCATION::processPX4Odometry(const nav_msgs::Odometry::ConstPtr &msg){
    odom_msg_.header = msg->header;
    odom_msg_.child_frame_id = msg->child_frame_id;
    odom_msg_.pose.pose.orientation = msg->pose.pose.orientation;
    odom_msg_.twist.twist.linear = msg->twist.twist.linear;
    odom_msg_.twist.twist.angular = msg->twist.twist.angular;
    odom_msg_.pose.covariance = msg->pose.covariance;
    odom_msg_.twist.covariance = msg->twist.covariance;
}


void LOCATION::processDataCallback(const ros::TimerEvent &e){
    odom_msg_.pose.pose.position.x = 0;
    odom_msg_.pose.pose.position.y = 0;
    odom_msg_.pose.pose.position.z = laser_altitude;
    odom_pub.publish(odom_msg_);  
}


/*
**用于配置TFmini Plus串口，并解析数据
*/
void LOCATION::read_laser_via_uart(){
    serial::Serial serial_port;
    try
    {
        // 打开串口设备
        serial_port.setPort("/dev/ttyUSB0");
        serial_port.setBaudrate(115200);
        serial_port.setBytesize(serial::eightbits);  // 设置数据位为8
        serial_port.setStopbits(serial::stopbits_one);  // 设置停止位为1
        serial_port.setParity(serial::parity_none);  // 设置奇偶校验位为None
        serial::Timeout timeout = serial::Timeout::simpleTimeout(100000);
        serial_port.setTimeout(timeout);
        serial_port.open();
    }
    catch (serial::IOException& e)
    {
        ROS_ERROR_STREAM("Failed to open the serial port: " << e.what());
        return ;
    }

    if(serial_port.isOpen())
    {
        ROS_INFO_STREAM("Serial Port opened");
    }

    //经过实测发现，前41个字节没有用，所以先读取进行过滤
    serial_port.read(41);       

    float distance_m;  // 距离值以米为单位

    // 解析强度值
    uint16_t strength;

    // 解析温度值
    float temperature_c;

    ros::Rate loop_time(100);
    while(ros::ok()){
        if(serial_port.available()){
            uint8_t buffer[9];
            size_t bytes_read = serial_port.read(buffer, 9);
            // 检查是否成功读取到完整的数据帧
            if (bytes_read != 9)
            {
                ROS_ERROR_STREAM("Failed to read complete data frame");
                continue;
            }
            //检查帧尾
            // uint8_t sum = 0;
            // for (int i = 0; i < 8; i++) {
            //     sum += buffer[i];
            // }
            // uint8_t checksum = sum % 256;  // 取累加和的低 8 位
            // if (checksum != buffer[8])
            // {
            //     ROS_ERROR_STREAM("Checksum verification failed");
            //     continue;
            // }
            // 检查帧头
            if (buffer[0] == 0x59 && buffer[1] == 0x59)
            {
                // 解析距离值
                uint16_t distance = (buffer[3] << 8) | buffer[2];
                distance_m = static_cast<float>(distance) / 100.0;  // 距离值以米为单位
                laser_altitude = distance_m;

                // 解析强度值
                strength = (buffer[5] << 8) | buffer[4];

                // 解析温度值
                int16_t temperature = (buffer[7] << 8) | buffer[6];
                temperature_c = static_cast<float>(temperature) / 8.0 - 256.0;  // 温度值以摄氏度为单位

                // 输出解析结果
                std::cout << "Distance: " << distance_m << " m" << std::endl;
                std::cout << "Strength: " << strength << std::endl;
                std::cout << "Temperature: " << temperature_c << " °C" << std::endl;           
            }
        }
    // odom_msg_.pose.pose.position.x = distance_m;
    // odom_pub.publish(odom_msg_);  
    }
}


/*
**气压计数据处理回调函数
*/
void LOCATION::processBaroCallback(const mavros_msgs::Altitude::ConstPtr &msg){

}


/*
**该函数用于读取rtk的串口数据，并将其解析为经纬高的形式
*/
void LOCATION::read_rtk_via_uart(){

    serial::Serial serial_port;
    try
    {
        // 打开串口设备
        serial_port.setPort("/dev/ttyUSB0");
        serial_port.setBaudrate(115200);
        serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);
        serial_port.setTimeout(timeout);
        serial_port.open();
    }
    catch (serial::IOException& e)
    {
        ROS_ERROR_STREAM("Failed to open the serial port: " << e.what());
        return ;
    }

    if(serial_port.isOpen())
    {
        ROS_INFO_STREAM("Serial Port opened");
    }

    // 发送指令
    std::string Mobile_Statioan = "cshg mode rover\r\n";
    std::string com1 = "log com1 gpgga ontime 1\r\n";
    std::string com3 = "log com3 gpgga ontime 1\r\n";

    serial_port.write(Mobile_Statioan);
    std::this_thread::sleep_for(std::chrono::seconds(2));

    serial_port.write(com1);
    std::this_thread::sleep_for(std::chrono::seconds(2));

    serial_port.write(com3);
    std::this_thread::sleep_for(std::chrono::seconds(2));

    ROS_INFO_STREAM("Serial send completed");    

    while (ros::ok())
    {
        // 从串口读取一行数据
        std::string line = serial_port.readline();
        cout << line << endl;
        parse_gpgga(line);

    }

}


/*
**RTK传回的gpgga字节流解析函数
*/
void LOCATION::parse_gpgga(const std::string& gpgga_data){
    /*将数据流字符串按照逗号进行分割*/
    std::vector<std::string> fields;
    size_t start = 0;
    size_t commaIndex = gpgga_data.find(',', start);
    while (commaIndex != std::string::npos)
    {
        fields.push_back(gpgga_data.substr(start, commaIndex - start));
        start = commaIndex + 1;
        commaIndex = gpgga_data.find(',', start);
    }
    fields.push_back(gpgga_data.substr(start));

    if (fields[0] != "$GPGGA" || fields[6] != "4")
    {
        cout << "rtk read error" << endl;
    }
    else
    {
        lat = std::stod(fields[2].substr(0, 2)) + std::stod(fields[2].substr(2)) / 100;
        if (fields[3] == "S")
        {
            lat = -lat;
        }
        lon = std::stod(fields[4].substr(0, 3)) + std::stod(fields[4].substr(3)) / 100;
        if (fields[5] == "W")
        {
            lon = -lon;
        }
        alt = std::stod(fields[9]);

        cout << "高度：" << alt << endl;
        WGS84_2_world();
    }
}




