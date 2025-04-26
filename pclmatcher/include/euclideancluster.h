#pragma once
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
class EuclideanCluster
{
public:
	void setInputCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud); // 输入点云
	void setClusterTolerance(double tolerance);                               // 搜索半径
	void setMinClusterSize(int minCluster);                                   // 聚类最小点数
	void setMaxClusterSize(int maxCluster);                                   // 聚类最大点数
	void extract(std::vector<std::vector<int>>& m_clusterIndices);    // 欧式聚类
	void clusterColor(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& ccloud); // 聚类结果分类渲染

	~EuclideanCluster() {}

private:
	double m_tolerance = 0.02; // 搜索半径默认参数
	int m_minCluster = 100;    // 聚类最小点数默认参数
	int m_maxCluster = 25000;  // 聚类最大点数默认参数
	pcl::PointCloud<pcl::PointXYZ>::Ptr m_cloud;
};

