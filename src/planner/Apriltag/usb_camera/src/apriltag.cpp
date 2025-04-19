#include <apriltag/apriltag.h>


int main(int argc, char** argv){
    ros::init(argc, argv, "apriltag_node");
    ros::NodeHandle nh("~");
    APRILTAG apriltag;
    apriltag.init(nh);
    ros::spin();
    return 0;    
}

void APRILTAG::init(ros::NodeHandle &nh){
  camInit();
  imageTimer = nh.createTimer(ros::Duration(0.1), &APRILTAG::imageCallback, this);
  image_transport::ImageTransport it(nh);
  cam_pub = it.advertise("/apriltag/camera", 10);
}


void APRILTAG::imageCallback(const ros::TimerEvent &e){
  
  bool isSuccess = cap.read(frame);
  if(!isSuccess)//if the video ends, then break
  {
      std::cout<<"video ends"<<std::endl;
  }
  image = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
  cam_pub.publish(image);
}



void APRILTAG::camInit(){
  cap.open(6);//open video from the path
  if(!cap.isOpened()){
    std::cout<<"open video failed!"<<std::endl;
    return;
  }
  else{
    std::cout<<"open camera success!"<<std::endl;
  }  
}