#ifndef PCLMATCHER_H
#define PCLMATCHER_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include "std_msgs/Float32MultiArray.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sstream>
#include <pcl/filters/passthrough.h>
#include <pcl/common/transforms.h>
#include <Eigen/Dense>
#include <thread>
#include <iostream>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <vector>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_hull.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/segmentation/segment_differences.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/surface/mls.h>
#include <pcl/console/time.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/radius_outlier_removal.h>//半径滤波器
#include <chrono>
#include <unordered_map>
#include <Eigen/Core>
#include <visualization_msgs/Marker.h>
#include "mycsf.h"
#include "fasteuclideancluster.h"
#include "euclideancluster.h"

class PCLMatcher {
public:
    PCLMatcher(ros::NodeHandle& nh);
    ~PCLMatcher();

    void loadPCD(const std::string& file_path);
    void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& input_cloud);
    void fieldCloudPublisher();
    void icp_run();
    void trans_publisher(Eigen::Matrix4f final_transform);
    
    void initialPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
    void computeInitialAlignment();
    void clickedPointCallback(const geometry_msgs::PointStamped::ConstPtr& msg);
    
    void removeOverlappingPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr& live_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr& field_cloud);
    void icpFunction(pcl::PointCloud<pcl::PointXYZ>::Ptr& sourcecloud, pcl::PointCloud<pcl::PointXYZ>::Ptr& targetcloud, float transformationEpsilon, float distance, float euclideanFitnessEpsilon, int maximumIterations, bool useReciprocalCorrespondences, pcl::PointCloud<pcl::PointXYZ>::Ptr& aligncloud, Eigen::Matrix4f& final_transform);
    void voxel_grid_upsample(pcl::PointCloud<pcl::PointXYZ>::Ptr& sourcecloud);
    // 定义高斯核函数
    inline float kernel(float x)
    {
        return 2 * sqrt(x) * exp(-0.5 * x);
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber cloud_sub;
    ros::Publisher field_pub;  // 发布加载的 PCD 点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr field_cloud;

    sensor_msgs::PointCloud2 ros_field_cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud;  // 过滤后的点云
    Eigen::Affine3f level_transform; // 用于将点云调整到水平位置的变换矩阵
    bool isLevel = false;
    ros::Publisher adjusted_pub;  // 发布调整后的点云
    ros::Publisher icpadjusted_pub;
    ros::Publisher icp_transformation_pub;
    std::thread field_pub_thread;
    pcl::PointCloud<pcl::PointXYZ>::Ptr readyICP_cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr icp_cloud;;
    // 提取地面点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr ground_readyICP;
    pcl::PointCloud<pcl::PointXYZ>::Ptr ground_field;

    
    Eigen::Matrix4f cumulative_transform; // 初始化累积变换矩阵
    std::thread icp_thread;
    bool isreadyICP = false;
    bool isICPFinish = false;
    bool if_auto_align = true;
    ros::Subscriber initial_pose_sub;
    Eigen::Affine3f initial_pose; // 存储初始位姿的变换

    const int max_icp_iterations = 10; // 设置最大的ICP迭代次数
    const double fitness_score_threshold = 0.01; // 设置适应度得分阈值

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloud_buffer; // 用于存储点云的缓存
    void mergePointClouds(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& plb, pcl::PointCloud<pcl::PointXYZ>::Ptr& pl); // 合并点云的函数

    std::vector<Eigen::Vector3f> map_feature_points; // 先验地图的特征点
    std::vector<Eigen::Vector3f> sensor_feature_points; // 传感器的特征点
    Eigen::Matrix4f initial_alignment_transform;
    Eigen::Matrix4f final_transform;
    
                                   
    ros::Subscriber clickpoint_sub;
    bool isINITFinish = false;

    ros::Publisher obstaclecloud_pub;
    ros::Publisher centroid_publisher;


    ClothSimulationFilter csf;
    std::vector<Eigen::Vector3f> m_centroids;
    ros::Publisher marker_pub;

    void upsampleVoxelGrid(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, float leaf_size, int points_per_voxel);
    void fastEuclideanCluster(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, double radius, int searchmaxsize, int minclustersize, int maxclustersize, std::vector<Eigen::Vector3f>& centroids);
    void statisticalOutlierRemoval(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, int num, float thresh);
    void meanshiftclustering(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, int bandWidth);
    void euclideancluster(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, double radius, int minclustersize, int maxclustersize);
    void radiusoutlierremoval(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, double near_radius, int near_neighbor, double far_radius, int far_neighbor);
    void printCentroids(const std::vector<Eigen::Vector3f>& centroids);
    void publishCentroidMarkers(const ros::Publisher& marker_pub, const std::vector<Eigen::Vector3f>& centroids);
    void auto_align();
};

#endif // PCLMATCHER_H
