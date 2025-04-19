#include <circle_detection.h>

std::pair<bool, cv::Point3d> CIRCLE::DetectionCallback(const sensor_msgs::Image::ConstPtr& msg) {
    
    cv_bridge::CvImagePtr col;
    col = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::TYPE_8UC3);
    color_pic = col->image;
    cv::cvtColor(color_pic,color_pic,cv::COLOR_RGB2BGR);

    //***************************** EDCIRCLES Circle Segment Detection *****************************
    // Detection of circles directly from the input image
    cv::cvtColor(color_pic, img_gray, cv::COLOR_BGR2GRAY);//转变为灰度图
    EDCircles testEDCircles = EDCircles(img_gray);//
    Mat circleImg = testEDCircles.drawResult(true, ImageStyle::BOTH);//第一个参数true时显示背景图像，false时隐去背景图像  第二个参数CIRCLES显示检测到的圆, ELLIPSES显示检测到的椭圆, BOTH显示检测到的圆和椭圆

    //Get circle information as [cx, cy, r]
    vector<mCircle> circles = testEDCircles.getCircles();
    vector<mEllipse> Ellipse = testEDCircles.getEllipses();
    /*判断是否是同心圆*/
   
    if(circles.size() > 1){
            
        for(auto i = 0 ; i < static_cast<int>(circles.size()) ; ++i){
                        
            for(auto j = i + 1 ; j <static_cast<int>( circles.size()); ++j){
                
                double dis = CIRCLE::distance_cv_points(circles[i].center,circles[j].center);
                distance_i_j hh;
                hh.distance = dis;//a b之间的欧氏距离
                hh.i = int(i);
                hh.j = int(j);
                hh.radius = (circles[i].r + circles[j].r) / 2;
                ED_lib_dis.push_back(hh);//push_back() 是 C++ 中的一个向量成员函数，它用于将元素添加到向量的末尾。当调用 push_back() 时，它会将传入的元素添加到向量的末尾，并将向量的大小增加一个单位。也就是说，ED_lib_dis 中原本有 n 个元素，调用 push_back(hh) 后，ED_lib_dis 将会有 n+1 个元素，最后一个元素就是 hh。
            }
        }
        sort(ED_lib_dis.begin() , ED_lib_dis.end() ,  CIRCLE::cv_point_cmp);//这将使用 cv_point_cmp 函数作为排序的比较准则，按照 distance 的值从小到大对 ED_lib_dis 中的元素进行排序。
    
        // vector<distance_i_j> circle;
        
        // for (const auto& tmp : ED_lib_dis) {
        //     if (tmp.distance < 30) {
        //         circle.push_back(tmp);
        //     }
        // }

        // if (!circle.empty()) {
        //     sort(circle.begin(), circle.end(), radius_cmp);
        //     ring_width=abs(circles[circle[0].i].r- circles[circle[0].j].r);
        //     des_cen = circles[circle[0].i].center;//
        //     des_r = circles[circle[0].i].r;//                                 
        //     have_circle_ = true;
        // }

        if (ED_lib_dis[0].distance < 30) {
            ring_width=abs(circles[ED_lib_dis[0].i].r- circles[ED_lib_dis[0].j].r);
            des_cen = circles[ED_lib_dis[0].i].center;//
            des_r = circles[ED_lib_dis[0].i].r < circles[ED_lib_dis[0].j].r ? circles[ED_lib_dis[0].i].r : circles[ED_lib_dis[0].j].r;//                                 
            have_circle_ = true;        
        }

        // /*画出ED_lib_dis中的同心圆*/
        // if(ED_lib_dis[0].distance < 30){
            

        // }
    }else{
                
        have_circle_ = false;
    }

    /*判断是否是同心椭圆*/
  
     if(Ellipse.size() > 1){
            
        for(auto i = 0 ; i < static_cast<int>(Ellipse.size()) ; ++i){
                        
            for(auto j = i + 1 ; j <static_cast<int>( Ellipse.size()); ++j){
                
                double dis = CIRCLE::distance_cv_points(Ellipse[i].center,Ellipse[j].center);
                distance_i_j hh;
                hh.distance = dis;//a b之间的欧氏距离
                hh.i = int(i);
                hh.j = int(j);
                hh.radius = (Ellipse[i].axes.height + Ellipse[j].axes.height) / 2;
                ED_lib_disE.push_back(hh);//push_back() 是 C++ 中的一个向量成员函数，它用于将元素添加到向量的末尾。当调用 push_back() 时，它会将传入的元素添加到向量的末尾，并将向量的大小增加一个单位。也就是说，ED_lib_disE 中原本有 n 个元素，调用 push_back(hh) 后，ED_lib_disE 将会有 n+1 个元素，最后一个元素就是 hh。
            }
        }
        sort(ED_lib_disE.begin() , ED_lib_disE.end() ,  CIRCLE::cv_point_cmp);//这将使用 cv_point_cmp 函数作为排序的比较准则，按照 distance 的值从小到大对 ED_lib_disE 中的元素进行排序。
    
        // vector<distance_i_j> ellipse;

        // for (const auto& tmp : ED_lib_disE) {
        //     if (tmp.distance < 30) {
        //         ellipse.push_back(tmp);
        //     }
        // }

        // if (!ellipse.empty()) {
        //     sort(ellipse.begin(), ellipse.end(), radius_cmp);
        //     decE_cen=Ellipse[ellipse[0].i].center;//将其中一个椭圆的圆心赋给decE_cen
        //     decE_r = Ellipse[ellipse[0].i].axes.height;                                
        //     have_Ellipse = true;
        // }


        /*画出ED_lib_disE中的同心椭圆*/
        if(ED_lib_disE[0].distance < 30){
            
            if (Ellipse[ED_lib_disE[0].i].axes.height >= 0 && Ellipse[ED_lib_disE[0].i].axes.width >= 0 ) {
                decE_cen=Ellipse[ED_lib_disE[0].i].center;//将其中一个椭圆的圆心赋给decE_cen
                decE_r = Ellipse[ED_lib_disE[0].i].axes.height < Ellipse[ED_lib_disE[0].j].axes.height ? Ellipse[ED_lib_disE[0].i].axes.height : Ellipse[ED_lib_disE[0].j].axes.height;

            }              
            have_Ellipse = true;
        }
    }else{
                
        have_Ellipse = false;
    }
    std::pair<bool, cv::Point3d> res;

    if (have_circle_ && des_r > 50 && des_r < 250) {
    // if (have_circle_) {
        res.first = true;
        res.second = cv::Point3d(des_cen.x, des_cen.y, des_r);
    }else if (have_Ellipse && decE_r > 50 && decE_r < 250) {
    // }else if (have_Ellipse) {
        res.first = true;
        res.second = cv::Point3d(decE_cen.x, decE_cen.y, decE_r);
    }else {
        res.first = false;
    }
    if (res.first) {
        cv::circle(color_pic,cv::Point2d(res.second.x, res.second.y),res.second.z,cv::Scalar(0,255,0) ,ring_width);
        cv::circle(color_pic,cv::Point2d(res.second.x, res.second.y),2,cv::Scalar(0,255,0) ,2);
    }

    ED_lib_dis.clear();
    ED_lib_disE.clear();
    cv::imshow("ED_lib ",color_pic); 
    cv::waitKey(10); 

    return res;

}
  
// /*发布圆心坐标消息，同心圆的优先级高于同心椭圆的优先级，当有同心圆或者同心椭圆的时候point_msg.C=1，反之point_msg.C=0*/
// void CIRCLE::waypoint_callback(const ros::TimerEvent&){
    
//     if(  have_circle_ ){     
//        point_msg.C=1;
//        point_msg.center.x = des_cen.x;
//        point_msg.center.y = des_cen.y;     

//     }else if( have_Ellipse){
//         point_msg.C=1;
//         point_msg.center.x = decE_cen.x;
//         point_msg.center.y = decE_cen.y;
//     }
//     else{
//         point_msg.C=0;

//     }
//      // 发布消息
//     center_pub.publish(point_msg);
// }

inline double CIRCLE::distance_cv_points(const cv::Point &a , const cv::Point &b){

    return sqrt(pow(abs(a.x-b.x),2) + pow(abs(a.y-b.y),2));//a b两点之间的欧氏距离
}


inline bool CIRCLE::cv_point_cmp(const distance_i_j &a , const distance_i_j &b){
    
    return a.distance < b.distance;
}

inline bool CIRCLE::radius_cmp(const distance_i_j &a , const distance_i_j &b){
    
    return a.radius > b.radius;
}



