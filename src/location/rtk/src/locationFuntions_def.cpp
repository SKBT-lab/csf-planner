/*
**单独写一个cpp是为了实现LOCATION类中一些功能函数，减少程序的冗余
*/

#include "rtk/location.h"
using namespace std;


void LOCATION::Quaternion2Euler(const geometry_msgs::Quaternion& q, Euler& euler){
    // roll (x-axis rotation)

    // 将四元数转换为Eigen的Quaternion类型
    Eigen::Quaterniond eigenQuaternion(q.w, q.x, q.y, q.z);
    // 将Eigen的Quaternion类型转换为旋转矩阵(Matrix3d)
    Eigen::Matrix3d rotationMatrix = eigenQuaternion.toRotationMatrix();

    euler.roll = atan2(rotationMatrix(2, 1), rotationMatrix(2, 2));
    euler.pitch = asin(-rotationMatrix(2, 0));
    euler.yaw = atan2(rotationMatrix(1, 0), rotationMatrix(0, 0));

    cout << "YAW   : " << euler.yaw << endl;
    cout << "PITCH : " << euler.pitch << endl;  
    cout << "ROLL  : " << euler.roll << endl; 

    cout << "---------------------------" << endl;         

}


void LOCATION::WGS84_2_world(){
    
    double _lat = lat * (M_PI/180);
    double _lon = lon * (M_PI/180);
    double Re = 6378137.0;
    double e2 = 0.00669437999013;
    double tmp = 1 - e2;
    double tmp2 = sqrt(1 - e2 * sin(_lat) * sin(_lat));
    double N = Re / tmp2;
    

    geometry_msgs::Point origin_point;

    while(flag == 0){
        origin_point.x = (N + alt) * cos(_lat) * cos(_lon);
        origin_point.y = (N + alt) * cos(_lat) * sin(_lon);
        origin_point.z = (Re * tmp * sin(_lat)) / tmp2 + alt * sin(_lat);
        flag = 9;
    }

    world_point.x = (N + alt) * cos(_lat) * cos(_lon) - origin_point.x;
    world_point.y = (N + alt) * cos(_lat) * sin(_lon) - origin_point.y;
    world_point.z = (Re * tmp * sin(_lat)) / tmp2 + alt * sin(_lat) - origin_point.z;

    cout << "world_point.x: " << world_point.x << endl;
    cout << "world_point.y: " << world_point.y << endl;
    cout << "world_point.z: " << world_point.z << endl;
}
