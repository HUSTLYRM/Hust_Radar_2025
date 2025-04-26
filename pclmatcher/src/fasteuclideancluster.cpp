#include "fasteuclideancluster.h"
#include <pcl/kdtree/kdtree_flann.h>  // kdtree近邻搜索
#include <unordered_map>
// 设置输入点云
void FastEuclideanCluster::setInputCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
	m_cloud = cloud;
};
// 设置搜索半径
void FastEuclideanCluster::setClusterTolerance(double tolerance)
{
	m_tolerance = tolerance;
};
// 设置邻域搜索最大点数
void FastEuclideanCluster::setSearchMaxSize(int maxSize)
{
	m_maxSize = maxSize;
}

// 设置聚类最小点数
void FastEuclideanCluster::setMinClusterSize(int minCluster)
{
	m_minCluster = minCluster;
};
// 设置聚类最大点数
void FastEuclideanCluster::setMaxClusterSize(int maxCluster)
{
	m_maxCluster = maxCluster;
};
// 快速欧式聚类计算过程
void FastEuclideanCluster::extract(std::vector<std::vector<int>>& m_clusterIndices)
{
	std::vector<int> labels(m_cloud->points.size(), 0); // initalize all point label as 0
	int segLab = 1; // Segment label
	// 建立kd-tree索引
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	kdtree.setInputCloud(m_cloud);
	pcl::Indices indices;
	std::vector<float> dists;

	for (int i = 0; i < m_cloud->points.size(); ++i)
	{
		// if Pi.lable =0
		if (labels[i] == 0)
		{
			// Pnn = FindNeighbor(pi,dth)
			kdtree.radiusSearch(m_cloud->points[i], m_tolerance, indices, dists, m_maxSize);

			std::vector<int> nLabs;	// Pnn.lab

			for (int j = 0; j < indices.size(); ++j)
			{
				// if Nonzero(Pnn.lab)
				if (labels[indices[j]] != 0)
				{
					nLabs.push_back(labels[indices[j]]);
				}
			}
			// minSegLab = min(N0nzero(Pnn.lab),SegLab)
			int minSegLab = 0;
			if (!nLabs.empty())
			{
				minSegLab = *std::min_element(nLabs.begin(), nLabs.end());
			}
			else
			{
				minSegLab = segLab;
			}
			// Segment mrege loop:
			// foreach pj in Pnn do
			for (int j = 0; j < nLabs.size(); ++j)
			{
				// if pj.lab > minSeglab then 
				if (nLabs[j] > minSegLab)
				{
					// foreach pk.lab in P do 
					for (int k = 0; k < labels.size(); ++k)
					{
						// if pk.lab = Pj.lab then
						if (labels[k] == nLabs[j])
						{
							// pk.lab = minSegLab
							labels[k] = minSegLab;
						} // end
					} // end 
				} // end
			} // end

			// 将所有邻近点分类
			for (size_t nnIdx = 0; nnIdx < indices.size(); ++nnIdx)
			{
				labels[indices[nnIdx]] = minSegLab;
			}
			segLab++;
		}
	}

	// 根据分类标签对点云进行分类
	std::vector<std::vector<int>>clusterIndices;
	std::unordered_map<int, int> segID;
	int index = 1;
	for (int i = 0; i < m_cloud->points.size(); i++)
	{
		int label = labels[i];

		if (!segID[label])
		{
			segID[label] = index;
			clusterIndices.push_back(std::vector<int>());
			index++;
		}
		clusterIndices[segID[label] - 1].push_back(i);
	}

	// ------------------------筛选符合点数阈值的类别-------------------------------
	for (int i = 0; i < clusterIndices.size(); ++i)
	{
		if (clusterIndices[i].size() > m_minCluster && clusterIndices[i].size() < m_maxCluster)
		{
			m_clusterIndices.push_back(clusterIndices[i]);
		}
	}
	
}
// 聚类结果分类渲染
void FastEuclideanCluster::clusterColor(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& ccloud)
{
	double R = rand() % (256) + 0;
	double G = rand() % (256) + 0;
	double B = rand() % (256) + 0;
	
	for_each(ccloud->begin(), ccloud->end(),
		[R,G,B](pcl::PointXYZRGB& point) 
		{ point.r = R, point.g = G, point.b = B; });

};

