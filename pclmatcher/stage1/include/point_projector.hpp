#ifndef MY_ROS_PROJECT_POINT_PROJECTOR_H
#define MY_ROS_PROJECT_POINT_PROJECTOR_H

#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Point.h>
#include <opencv2/opencv.hpp>
#include <vector>

namespace point_projector {

class PointProjector {
public:
    PointProjector(ros::NodeHandle& nh);
    ~PointProjector();

private:
    ros::NodeHandle nh_;
    ros::Subscriber points_sub_;
    ros::Subscriber image_sub_;
    ros::Publisher image_pub_;

    std::vector<geometry_msgs::Point> points_;
    cv::Mat camera_matrix_;  // 相机内参矩阵

    void pointsCallback(const std_msgs::Float32MultiArray::ConstPtr& msg);
    void imageCallback(const sensor_msgs::ImageConstPtr& msg);
    cv::Point projectToImagePlane(const geometry_msgs::Point& point_3d);
    void processAndPublishImage(const cv::Mat& image, const std::vector<cv::Point>& points);
};

}  // namespace my_ros_project

#endif  // MY_ROS_PROJECT_POINT_PROJECTOR_H
