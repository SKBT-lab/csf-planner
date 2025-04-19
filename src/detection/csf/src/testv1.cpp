#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/highgui/highgui.hpp"

cv::Mat depth_image;
cv::Mat depth_image_noinf;
bool barrier_flag;

// 全局变量，用于保存突变阈值
int threshold_value = 3;

// 区域生长函数
void regionGrowing(const cv::Mat& src, cv::Mat& dst, int row, int col, int threshold) {
    if (dst.at<uchar>(row, col) == 0) {
        dst.at<uchar>(row, col) = 255;

        for (int i = -1; i <= 1; ++i) {
            for (int j = -1; j <= 1; ++j) {
                if (row + i >= 0 && row + i < src.rows && col + j >= 0 && col + j < src.cols &&
                    // std::cout << abs(src.at<cv::Vec3b>(row + i, col + j)[1] - src.at<cv::Vec3b>(row, col)[1]) << std::endl;
                    abs(src.at<cv::Vec3b>(row + i, col + j)[2] - src.at<cv::Vec3b>(row, col)[2]) < threshold) {
                    regionGrowing(src, dst, row + i, col + j, threshold);
                }
            }
        }
    }
}

void onTrackbarChange(int new_value, void* userdata)
{
    threshold_value = new_value;
}

bool areCentersEqual(const cv::Mat& centers, double threshold) {
    int k = centers.rows;
    for (int i = 0; i < k; ++i) {
        for (int j = i + 1; j < k; ++j) {
            double dist = cv::norm(centers.row(i) - centers.row(j));
            if (dist < threshold) {
                return false;
            }
        }
    }
    return true;
}

void depthImageCallback(const sensor_msgs::Image::ConstPtr& msg)
{
    // 将ROS图像消息转换为OpenCV格式
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    depth_image = cv_ptr->image.clone();
    // 处理深度图像中的无穷大值
    cv::MatIterator_<float> it, end;
    for (it = cv_ptr->image.begin<float>(), end = cv_ptr->image.end<float>(); it != end; ++it)
    {
        if (std::isinf(*it))
        {
            *it = 100.0f;
        }
    }
    depth_image_noinf = cv_ptr->image;

    // // 将深度放大 100 倍
    // cv::Mat depth_scaled = depth_image_noinf * 200;
    // // 转换深度图像为向量形式
    // cv::Mat depth_vector;
    // depth_scaled.reshape(1, 1).convertTo(depth_vector, CV_32FC1);

    // // 设置K均值聚类参数
    // int k = 4; // 聚类数目
    // cv::Mat labels;
    // cv::Mat centers;
    // cv::TermCriteria criteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.1);

    // // 运行K均值聚类
    // cv::kmeans(depth_vector, k, labels, criteria, 3, cv::KMEANS_RANDOM_CENTERS, centers);
    // // 输出各个聚类中心
    // if (areCentersEqual(centers, 0.5)){
    //     cv::Mat clustered_image_gray(cv_ptr->image.size(), CV_8UC1);
    //     for (int i = 0; i < cv_ptr->image.rows; ++i)
    //     {
    //         for (int j = 0; j < cv_ptr->image.cols; ++j)
    //         {
    //             int label = labels.at<int>(i * cv_ptr->image.cols + j);
    //             clustered_image_gray.at<uchar>(i, j) = static_cast<uchar>(centers.at<float>(label));
    //         }
    //     }

    //     // 根据聚类结果重新着色深度图像
    //     cv::Mat clustered_image_color(depth_image_noinf.size(), CV_8UC3);
    //     cv::applyColorMap(clustered_image_gray, clustered_image_color, cv::COLORMAP_JET);
    //     // 显示聚类后的深度图像
    //     cv::imshow("Clustered Depth Image", clustered_image_color);
    // }

    

//     cv::Mat blank(depth_image_noinf.rows, depth_image_noinf.cols, CV_8UC1, cv::Scalar(0));
//     float depth_threshold = 0.1f; // 深度变化阈值
//     for (int i = 0; i < depth_image_noinf.rows; ++i)
//     {
//     for (int j = 1; j < depth_image_noinf.cols; ++j)
//     {
//         // 计算相邻像素之间的深度差异
//         float depth_diff = depth_image_noinf.at<float>(i, j-1) - depth_image_noinf.at<float>(i, j);
//         if (depth_diff > depth_threshold)
//         {
//             // 标记突变点
//             blank.at<uchar>(i, j) = 255; // 将对应像素设为白色
//         }
//     }
// }

cv::Mat blank(depth_image_noinf.rows, depth_image_noinf.cols, CV_8UC3, cv::Scalar(0, 0, 0));
cv::Mat blank_gray(depth_image_noinf.rows, depth_image_noinf.cols, CV_8UC1, cv::Scalar(0));
float depth_threshold = 0.2f; // 深度变化阈值
for (int i = 0; int(i < depth_image_noinf.rows * 4 / 5); ++i){
    if (depth_image_noinf.at<float>(i, 0) == 100.0){
        barrier_flag = false;
    }
    else{
        barrier_flag = true;
    }

    for (int j = 1; j < depth_image_noinf.cols; ++j){
        float depth = depth_image_noinf.at<float>(i, j - 1);
        if(barrier_flag == false){
            blank.at<cv::Vec3b>(i, j - 1) = cv::Vec3b(0, 255, 0);
            blank_gray.at<uchar>(i, j - 1) = 0;
        }
        else{
            blank.at<cv::Vec3b>(i, j - 1) = cv::Vec3b(0, 0, int(depth / 5 * 255));
            blank_gray.at<uchar>(i, j - 1) = int(depth / 5 * 255);
        }
         
        float depth_diff = depth_image_noinf.at<float>(i, j-1) - depth_image_noinf.at<float>(i, j);
        if (depth_diff > depth_threshold && barrier_flag == false){
            barrier_flag = true;
        }
        else if (depth_diff < (-1 * depth_threshold) && barrier_flag == true){
            barrier_flag = false;
        }
        else if (depth_diff > depth_threshold && barrier_flag == true){
            for(int n = j - 1; n >0; n--){
                
                if(blank.at<cv::Vec3b>(i, n) != cv::Vec3b(0, 255, 0)){
                    blank.at<cv::Vec3b>(i, n) = cv::Vec3b(0, 255, 0);
                    blank_gray.at<uchar>(i, n) = 0;
                }
                else{
                    break;
                }
            }
        }
    
    
        // else if (depth_diff < (-1 * depth_threshold) && barrier_flag == false){
        //     for(int n = j; n >=0; j--){
        //         if(blank.at<cv::Vec3b>(i, n) == cv::Vec3b(0, 255, 0)){
        //             blank.at<cv::Vec3b>(i, n) == cv::Vec3b(0, 0, 255);
        //         }
        //         else{
        //             break;
        //         }
        //     } 
        // }

    }
}
    // 存储每个灰度值对应的点的坐标集合
    std::vector<std::vector<cv::Point>> hist_points(256);

    // 遍历图像，将每个像素的坐标加入相应灰度值的点集
    for (int row = 0; row < blank_gray.rows; ++row) {
        for (int col = 0; col < blank_gray.cols; ++col) {
            int intensity = blank_gray.at<uchar>(row, col); // 获取当前像素的灰度值
            hist_points[intensity].push_back(cv::Point(col, row)); // 将当前像素的坐标加入相应灰度值的点集
        }
    }


    //绘制直方图

    // 统计每个灰度值对应的像素点个数
    std::vector<int> hist_counts(256);
    for (int i = 0; i < 256; ++i) {
        hist_counts[i] = hist_points[i].size();
    }

    // 绘制灰度直方图
    int hist_width = 512;
    int hist_height = 400;
    cv::Mat hist_image(hist_height, hist_width, CV_8UC3, cv::Scalar(255, 255, 255));

    int bin_width = hist_width / 256;
    for (int i = 0; i < 256; ++i) {
        cv::rectangle(hist_image, cv::Point(i * bin_width, hist_height),
                      cv::Point((i + 1) * bin_width, hist_height - hist_counts[i]),
                      cv::Scalar(0, 0, 0), -1);
    }
    cv::imshow("Histogram", hist_image);



    //统计最多的点集
    int max_pix = 1;
    for (size_t i = 1; i < hist_points.size(); ++i) {
        if (hist_points[i].size() > hist_points[max_pix].size()) {
            max_pix = i;
        }
    }

    // 创建空白图像
    cv::Mat Hist_result(blank_gray.size(), CV_8U, cv::Scalar(0));

    // 绘制最多点的集合到空白图像上
    for (int i = max_pix - 3; i <= max_pix + 3; ++i){
        for (const auto& point : hist_points[i]) {
        Hist_result.at<uchar>(point.y, point.x) = 255;
    }
    }

    // 找到Mask中的所有轮廓
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(Hist_result, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    // 找到面积最大的轮廓
    double max_area = 0;
    int max_contour_index = -1;
    for (size_t i = 0; i < contours.size(); ++i) {
        double area = cv::contourArea(contours[i]);
        if (area > max_area) {
            max_area = area;
            max_contour_index = i;
        }
    }

    cv::Mat blank_bgr;
    cv::cvtColor(blank_gray, blank_bgr, cv::COLOR_GRAY2BGR);

    // 绘制面积最大的轮廓的边界框
    if (max_contour_index != -1) {
        cv::Rect bounding_rect = cv::boundingRect(contours[max_contour_index]);
        cv::rectangle(blank_bgr, bounding_rect, cv::Scalar(0, 255, 0), 2);
    }    
    



    
    //************ 区域生成
    cv::Mat blank2(depth_image_noinf.rows, depth_image_noinf.cols, CV_8UC1, cv::Scalar(0));

    regionGrowing(blank, blank2, 120, 160, 2);

    // 显示结果
    cv::imshow("Segmented Image", blank2);
    //********************


    
    cv::imshow("blank", blank);
    cv::imshow("blankGray", blank_gray);
    //cv::imshow("Hist", Hist_result);
    cv::imshow("result", blank_bgr);
    // 显示深度图像
    cv::imshow("Depth Image", depth_image);
    cv::waitKey(1);
}

void onMouse(int event, int x, int y, int flags, void* userdata)
{
    if (event == cv::EVENT_LBUTTONDOWN)
    {
        // 输出点击位置处的像素值
        if (!depth_image.empty())
        {
            float depth_value = depth_image_noinf.at<float>(y, x);
            ROS_INFO("Depth value at (%d, %d): %f", y, x, depth_value);
        }
    }
}

int main(int argc, char **argv)
{
    // 初始化ROS节点
    ros::init(argc, argv, "test");

    // 创建节点句柄
    ros::NodeHandle nh;

    // 创建一个Subscriber，订阅名为"/depth_image"的深度图话题
    ros::Subscriber sub = nh.subscribe("/xv_sdk/xv_dev/tof_camera/image", 1, depthImageCallback);

    // 创建一个空白窗口
    cv::namedWindow("Depth Image");

    // 设置鼠标事件回调函数
    cv::setMouseCallback("Depth Image", onMouse, NULL);

     // 创建一个空白窗口
    cv::namedWindow("Marked Columns");

    // 创建一个滑动条
    cv::createTrackbar("Threshold", "Marked Columns", &threshold_value, 100, onTrackbarChange);

    // 循环等待回调函数
    ros::spin();

    return 0;
}