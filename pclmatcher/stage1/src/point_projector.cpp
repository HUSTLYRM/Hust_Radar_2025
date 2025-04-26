#include "my_ros_project/point_projector.h"
#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

namespace point_projector {

// 构造函数
PointProjector::PointProjector(ros::NodeHandle& nh) : nh_(nh) {
    // 设置相机内参矩阵（假设为简化的内参）
    camera_matrix_ = (cv::Mat_<double>(3, 3) << 500, 0, 320, 0, 500, 240, 0, 0, 1);

    // 设置相机到赛场的外参矩阵
    cv::Mat camera_to_field_matrix = (cv::Mat_<double>(4, 4) << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1);

    // 订阅 3D 点坐标
    points_sub_ = nh_.subscribe("/centroids", 1, &PointProjector::pointsCallback, this);

    // 订阅图像数据
    image_sub_ = nh_.subscribe("/camera/image", 1, &PointProjector::imageCallback, this);

    // 发布处理后的图像
    image_pub_ = nh_.advertise<sensor_msgs::Image>("/processed_image", 1);
}

// 析构函数
PointProjector::~PointProjector() {}

// 回调函数：接收 3D 点数据
void PointProjector::pointsCallback(const std_msgs::Float32MultiArray::ConstPtr& msg) {
    points_.clear();
    for (size_t i = 0; i < msg->data.size(); i += 3) {
        geometry_msgs::Point point;
        point.x = msg->data[i];
        point.y = msg->data[i + 1];
        point.z = msg->data[i + 2];
        points_.push_back(point);
    }
}

// 回调函数：接收图像数据
void PointProjector::imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    // 使用 cv_bridge 将 ROS 图像消息转换为 OpenCV 图像
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    // 存储绘制的 ROI 的中心点
    std::vector<cv::Point> projected_points;

    // 投影 3D 点到图像平面
    for (const auto& point : points_) {
        cv::Point projected_point = projectToImagePlane(point);
        projected_points.push_back(projected_point);
    }

    // 处理并发布图像
    processAndPublishImage(cv_ptr->image, projected_points);
}

// 投影函数：将 3D 点投影到图像平面
cv::Point PointProjector::projectToImagePlane(const geometry_msgs::Point& point_3d) {
    cv::Mat X = (cv::Mat_<double>(4, 1) << point_3d.x, point_3d.y, point_3d.z, 1.0);// [x,y,z]
    //变换到相机坐标系
    cv::Mat X_2d = camera_to_field_matrix * X;
    //投影到图像平面
    cv::Mat x_2d = camera_matrix_ * X_2d;// [x,y,1]
    // cv::Mat x_2d = camera_matrix_ * X;  // 投影到图像平面
    int x_pixel = static_cast<int>(x_2d.at<double>(0) / x_2d.at<double>(2));
    int y_pixel = static_cast<int>(x_2d.at<double>(1) / x_2d.at<double>(2));
    return cv::Point(x_pixel, y_pixel);
}

// 处理图像并发布：在图像上画 ROI 区域并截取
void PointProjector::processAndPublishImage(const cv::Mat& image, const std::vector<cv::Point>& points) {
    cv::Mat image_copy = image.clone();

    for (const auto& point : points) {
        // 在每个点的位置画一个矩形 ROI（假设每个 ROI 的大小为 250x250 像素）
        cv::Rect roi(point.x - 25, point.y - 25, 250, 250);  // 左上角(点位置-半宽高)，大小(50x50)
        cv::rectangle(image_copy, roi, cv::Scalar(0, 255, 0), 2);  // 绿色矩形框
    }

    // 发布处理后的图像
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image_copy).toImageMsg();
    image_pub_.publish(msg);
}

}  // namespace my_ros_project

/

//测试函数
void test() {
    cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) << 500, 0, 320, 0, 500, 240, 0, 0, 1);
    cv::Mat image = cv::Mat::zeros(480, 640, CV_8UC3);
    // 创建一些测试的3D点
    std::vector<geometry_msgs::Point> points_3d = {
        {1.0, 2.0, 5.0},  // 点1
        {-1.0, -2.0, 5.0}, // 点2
        {0.5, 1.0, 3.0}    // 点3
    };
    for (const auto& point_3d : points_3d) {
        // 3D点的齐次坐标表示 (x, y, z, 1)
        cv::Mat X = (cv::Mat_<double>(4, 1) << point_3d.x, point_3d.y, point_3d.z, 1.0);
        
        // 通过相机内参将3D点投影到2D图像平面
        cv::Mat x_2d = camera_matrix * X;  // 得到投影坐标
        int x_pixel = static_cast<int>(x_2d.at<double>(0) / x_2d.at<double>(2));  // 归一化
        int y_pixel = static_cast<int>(x_2d.at<double>(1) / x_2d.at<double>(2));  // 归一化
        
        // 在图像上画ROI框，假设每个ROI为50x50的矩形
        cv::Rect roi(x_pixel - 25, y_pixel - 25, 50, 50);  // 计算ROI区域，中心为投影点
        cv::rectangle(image, roi, cv::Scalar(0, 255, 0), 2);  // 在图像上绘制绿色矩形框
    }
     // 显示处理后的图像
    cv::imshow("Processed Image", image);
    cv::waitKey(0);  // 按键退出
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "point_projector_node");
    ros::NodeHandle nh;

    my_ros_project::PointProjector point_projector(nh);

    ros::spin();
    return 0;
}
