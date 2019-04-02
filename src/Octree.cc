#include "Octree.h"
#include <math.h>
#include <limits>
#include <inttypes.h>

#include <vector>
#include <cmath>

#include <stdio.h>
#include <sys/time.h>
#include <iostream>
#include <pcl/common/transforms.h>
#include <bitset>

namespace GlobalPlan
{

template <typename PointSourceType>
Octree<PointSourceType>::Octree()
{
	resolution_ = 0.4;
	max_level_ = 2;
	octree_.reset();
}
template <typename PointSourceType>
Eigen::Vector3i Octree<PointSourceType>::getNodeIdxFromPoint(PointSourceType point, int level)
{
	double resolution = resolution_ * std::pow(3, level);
	double x = std::round(point.x / resolution);
	double y = std::round(point.y / resolution);
	double z = std::round(point.z / resolution);
	return Eigen::Vector3i(x, y, z);
}

template <typename PointSourceType>
void Octree<PointSourceType>::setInput(typename pcl::PointCloud<PointSourceType>::Ptr point_cloud)
{
	cloud_ = point_cloud;
	octree_ = boost::make_shared<std::vector<Nodemap>>();

	//for level 0;
	(*octree_).resize(max_level_);
	int level = 0;
	std::cout << "create octree level.." << std::endl;
	Nodemap &nodemap = (*octree_)[level];
	for (int i = 0; i < (int)point_cloud->points.size(); i++)
	{
		auto &point = point_cloud->points[i];
		auto idx = getNodeIdxFromPoint(point, level);
		if (nodemap.find(idx) == nodemap.end())
		{
			auto node = new OctreeNode();
			node->level = level;
			//node->idx = idx;
			node->point_idx.push_back(i);
			node->centroid = Eigen::Vector3d(point.x, point.y, point.z);
			nodemap[idx] = node;
		}
		else
		{
			OctreeNode *node = nodemap[idx];
			node->point_idx.push_back(i);
			double s = (double)node->point_idx.size();
			node->centroid = node->centroid * (s - 1) / s + Eigen::Vector3d(point.x, point.y, point.z) / s;
		}
	}

	for (int level = 1; level < max_level_; level++)
	{
		//std::cout<<"level:"<<level<<std::endl;
		Nodemap &nodemap = (*octree_)[level];
		for (auto m : (*octree_)[level - 1])
		{
			Eigen::Vector3i cidx = m.first;
			OctreeNode *cnode = m.second;
			Eigen::Vector3i pidx = Eigen::Vector3i(std::round(cidx(0) / 2),
												   std::round(cidx(1) / 2),
												   std::round(cidx(2) / 2));


			if (nodemap.find(pidx) == nodemap.end())
			{
				auto node = new OctreeNode();
				node->level = level;
				node->point_idx.insert(node->point_idx.begin(), cnode->point_idx.begin(), cnode->point_idx.end());
				node->centroid = cnode->centroid;
				nodemap[pidx] = node;
				node->cnode.insert(cnode);
				cnode->pnode = node;
			}
			else
			{
				OctreeNode *node = nodemap[pidx];
				node->point_idx.insert(node->point_idx.begin(), cnode->point_idx.begin(), cnode->point_idx.end());
				double l = (double)cnode->point_idx.size();
				double s = (double)node->point_idx.size();
				node->centroid = node->centroid * (s - l) / s + cnode->centroid * l / s;
				node->cnode.insert(cnode);
				cnode->pnode = node;
			}
			
		}
	}
	std::cout << "compute covariance.." << std::endl;
	computeCovariance();
	std::cout << "OK!" << std::endl;

}

template <typename PointSourceType>
std::vector<OctreeNode *> Octree<PointSourceType>::getLevelNode(int level)
{
	std::vector<OctreeNode *> nodes;
	for (auto m : (*octree_)[level])
	{
		nodes.push_back(m.second);
	}
	return nodes;
}

template <typename PointSourceType>
void Octree<PointSourceType>::computeCovariance()
{
	for (int level = 0; level < max_level_; level++)
	{
		for (auto m : (*octree_)[level])
		{
			auto &node = m.second;
			Eigen::Vector4d centroid = Eigen::Vector4d(node->centroid(0), node->centroid(1), node->centroid(2), 1);
			Eigen::Matrix3d covariance;
			pcl::computeCovarianceMatrixNormalized(*cloud_, node->point_idx, centroid, covariance);
			node->covariance = covariance;
		}
	}
	
}

template class Octree<pcl::PointXYZ>;
template class Octree<pcl::PointXYZI>;

} // namespace GlobalPlan