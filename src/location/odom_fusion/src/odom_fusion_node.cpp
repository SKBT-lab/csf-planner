#include <odom_fusion/odom_fusion.h>


int main(int argc, char** argv){
    ros::init(argc, argv, "odom_fusion_node");
    ros::NodeHandle nh("~");
    ODOM_FUSION odomfusion;
    odomfusion.init(nh);
    ros::spin();
    return 0;    
}




