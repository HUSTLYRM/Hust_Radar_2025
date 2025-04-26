#include "my_ros_project/point_projector.h"
#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

namespace point_projector {

// ���캯��
PointProjector::PointProjector(ros::NodeHandle& nh) : nh_(nh) {
    // ��������ڲξ��󣨼���Ϊ�򻯵��ڲΣ�
    camera_matrix_ = (cv::Mat_<double>(3, 3) << 500, 0, 320, 0, 500, 240, 0, 0, 1);

    // �����������������ξ���
    cv::Mat camera_to_field_matrix = (cv::Mat_<double>(4, 4) << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1);

    // ���� 3D ������
    points_sub_ = nh_.subscribe("/centroids", 1, &PointProjector::pointsCallback, this);

    // ����ͼ������
    image_sub_ = nh_.subscribe("/camera/image", 1, &PointProjector::imageCallback, this);

    // ����������ͼ��
    image_pub_ = nh_.advertise<sensor_msgs::Image>("/processed_image", 1);
}

// ��������
PointProjector::~PointProjector() {}

// �ص����������� 3D ������
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

// �ص�����������ͼ������
void PointProjector::imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    // ʹ�� cv_bridge �� ROS ͼ����Ϣת��Ϊ OpenCV ͼ��
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    // �洢���Ƶ� ROI �����ĵ�
    std::vector<cv::Point> projected_points;

    // ͶӰ 3D �㵽ͼ��ƽ��
    for (const auto& point : points_) {
        cv::Point projected_point = projectToImagePlane(point);
        projected_points.push_back(projected_point);
    }

    // ��������ͼ��
    processAndPublishImage(cv_ptr->image, projected_points);
}

// ͶӰ�������� 3D ��ͶӰ��ͼ��ƽ��
cv::Point PointProjector::projectToImagePlane(const geometry_msgs::Point& point_3d) {
    cv::Mat X = (cv::Mat_<double>(4, 1) << point_3d.x, point_3d.y, point_3d.z, 1.0);// [x,y,z]
    //�任���������ϵ
    cv::Mat X_2d = camera_to_field_matrix * X;
    //ͶӰ��ͼ��ƽ��
    cv::Mat x_2d = camera_matrix_ * X_2d;// [x,y,1]
    // cv::Mat x_2d = camera_matrix_ * X;  // ͶӰ��ͼ��ƽ��
    int x_pixel = static_cast<int>(x_2d.at<double>(0) / x_2d.at<double>(2));
    int y_pixel = static_cast<int>(x_2d.at<double>(1) / x_2d.at<double>(2));
    return cv::Point(x_pixel, y_pixel);
}

// ����ͼ�񲢷�������ͼ���ϻ� ROI ���򲢽�ȡ
void PointProjector::processAndPublishImage(const cv::Mat& image, const std::vector<cv::Point>& points) {
    cv::Mat image_copy = image.clone();

    for (const auto& point : points) {
        // ��ÿ�����λ�û�һ������ ROI������ÿ�� ROI �Ĵ�СΪ 250x250 ���أ�
        cv::Rect roi(point.x - 25, point.y - 25, 250, 250);  // ���Ͻ�(��λ��-����)����С(50x50)
        cv::rectangle(image_copy, roi, cv::Scalar(0, 255, 0), 2);  // ��ɫ���ο�
    }

    // ����������ͼ��
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image_copy).toImageMsg();
    image_pub_.publish(msg);
}

}  // namespace my_ros_project

/

//���Ժ���
void test() {
    cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) << 500, 0, 320, 0, 500, 240, 0, 0, 1);
    cv::Mat image = cv::Mat::zeros(480, 640, CV_8UC3);
    // ����һЩ���Ե�3D��
    std::vector<geometry_msgs::Point> points_3d = {
        {1.0, 2.0, 5.0},  // ��1
        {-1.0, -2.0, 5.0}, // ��2
        {0.5, 1.0, 3.0}    // ��3
    };
    for (const auto& point_3d : points_3d) {
        // 3D�����������ʾ (x, y, z, 1)
        cv::Mat X = (cv::Mat_<double>(4, 1) << point_3d.x, point_3d.y, point_3d.z, 1.0);
        
        // ͨ������ڲν�3D��ͶӰ��2Dͼ��ƽ��
        cv::Mat x_2d = camera_matrix * X;  // �õ�ͶӰ����
        int x_pixel = static_cast<int>(x_2d.at<double>(0) / x_2d.at<double>(2));  // ��һ��
        int y_pixel = static_cast<int>(x_2d.at<double>(1) / x_2d.at<double>(2));  // ��һ��
        
        // ��ͼ���ϻ�ROI�򣬼���ÿ��ROIΪ50x50�ľ���
        cv::Rect roi(x_pixel - 25, y_pixel - 25, 50, 50);  // ����ROI��������ΪͶӰ��
        cv::rectangle(image, roi, cv::Scalar(0, 255, 0), 2);  // ��ͼ���ϻ�����ɫ���ο�
    }
     // ��ʾ������ͼ��
    cv::imshow("Processed Image", image);
    cv::waitKey(0);  // �����˳�
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "point_projector_node");
    ros::NodeHandle nh;

    my_ros_project::PointProjector point_projector(nh);

    ros::spin();
    return 0;
}
