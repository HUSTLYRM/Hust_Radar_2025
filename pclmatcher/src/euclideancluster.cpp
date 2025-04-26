#include "euclideancluster.h"
#include <pcl/kdtree/kdtree_flann.h>  // kdtree近邻搜索
// 设置输入点云
void EuclideanCluster::setInputCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
	m_cloud = cloud;
};
// 设置搜索半径
void EuclideanCluster::setClusterTolerance(double tolerance)
{
	m_tolerance = tolerance;
};
// 设置聚类最小点数
void EuclideanCluster::setMinClusterSize(int minCluster)
{
	m_minCluster = minCluster;
};
// 设置聚类最大点数
void EuclideanCluster::setMaxClusterSize(int maxCluster)
{
	m_maxCluster = maxCluster;
};
// 欧式聚类计算过程
void EuclideanCluster::extract(std::vector<std::vector<int>>& m_clusterIndices)
{
	std::vector<int> nn_indices;	// kdtree搜索到的点的索引
	std::vector<float> nn_dists;	// kdtree搜索到的点的距离
	pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kdtree(new pcl::KdTreeFLANN<pcl::PointXYZ>());
	kdtree->setInputCloud(m_cloud);                     // 设置需要建立kdtree的点云指针
	std::vector<bool> processed(m_cloud->points.size(), false);	//定义点云的点是否被处理过的标记
	//遍历点云中的每一个点
	for (int i = 0; i < m_cloud->points.size(); ++i)
	{
		// 如果该点已经处理则跳过
		if (processed[i])
		{
			continue;
		}

		std::vector<int> seed_queue;  //定义一个种子点队列
		int sq_idx = 0;				//记录已经处理种子点队列中的种子点的个数
		seed_queue.push_back(i);	//加入一个种子点
		processed[i] = true;		//标记这个种子点已经被搜索过了

		while (sq_idx < seed_queue.size())   //遍历每一个种子点
		{
			if (!kdtree->radiusSearch(m_cloud->points[seed_queue[sq_idx]], m_tolerance, nn_indices, nn_dists))
			{
				++sq_idx;
				continue;   //没找到近邻点就继续
			}

			for (size_t j = 0; j < nn_indices.size(); ++j)     //遍历搜索到的种子点的邻近点       
			{
				// 种子点的近邻点中如果已经处理则跳出循环
				if (processed[nn_indices[j]])
				{
					continue;
				}

				seed_queue.push_back(nn_indices[j]);	//将此种子点的临近点作为新的种子点，入队操作
				processed[nn_indices[j]] = true;		//该点已经处理，打标签
			}

			++sq_idx;
		}

		// 过滤满足最大点数和最小点数的类
		if (seed_queue.size() >= m_minCluster && seed_queue.size() <= m_maxCluster)
		{
			m_clusterIndices.push_back(seed_queue);
		}
	}
}
// 聚类结果分类渲染
void EuclideanCluster::clusterColor(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& ccloud)
{
	double R = rand() % (256) + 0;
	double G = rand() % (256) + 0;
	double B = rand() % (256) + 0;
	
	for_each(ccloud->begin(), ccloud->end(),
		[R,G,B](pcl::PointXYZRGB& point) 
		{ point.r = R, point.g = G, point.b = B; });

};

