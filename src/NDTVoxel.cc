#include "NDTVoxel.h"
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
NDTVoxel<PointSourceType>::NDTVoxel(const double resolution)
{
	step_ = 2;
	resolution_ = resolution;
	voxel_.reset();
}

template <typename PointSourceType>
Eigen::Vector3i NDTVoxel<PointSourceType>::getNodeIdxFromPoint(const PointSourceType point, const int mode) const
{
	double resolution = resolution_;
	double x, y, z;
	if (!mode)
	{
		x = std::floor(point.x / resolution);
		y = std::floor(point.y / resolution);
		z = std::floor(point.z / resolution);
	}
	else
	{
		x = std::round(point.x / resolution);
		y = std::round(point.y / resolution);
		z = std::round(point.z / resolution);
	}
	return Eigen::Vector3i(x, y, z);
}

template <typename PointSourceType>
void NDTVoxel<PointSourceType>::setInput(const typename pcl::PointCloud<PointSourceType>::Ptr point_cloud)
{
	cloud_ = point_cloud;
	voxel_ = boost::make_shared<std::vector<Nodemap>>();

	(*voxel_).resize(2);
	std::cout << "Create voxel.." << std::endl;

	for (int mode = 0; mode < 2; mode++)
	{
		Nodemap &nodemap = (*voxel_)[mode];
		for (int i = 0; i < (int)point_cloud->points.size(); i++)
		{
			auto &point = point_cloud->points[i];
			auto idx = getNodeIdxFromPoint(point, mode);
			if (nodemap.find(idx) == nodemap.end())
			{
				auto node = new NDTVoxelNode();
				node->mode = mode;
				node->idx = idx;
				node->point_idx.push_back(i);
				node->centroid = Eigen::Vector3d(point.x, point.y, point.z);
				nodemap[idx] = node;
			}
			else
			{
				NDTVoxelNode *node = nodemap[idx];
				node->point_idx.push_back(i);
				double s = (double)node->point_idx.size();
				node->centroid = node->centroid * (s - 1) / s + Eigen::Vector3d(point.x, point.y, point.z) / s;
			}
		}
	}
	computeCovariance();
}

template <typename PointSourceType>
std::vector<NDTVoxelNode *> NDTVoxel<PointSourceType>::getAllNode() const
{
	std::vector<NDTVoxelNode *> nodes;
	for (int mode = 0; mode < 2; mode++)
	{
		for (auto m : (*voxel_)[mode])
		{
			nodes.push_back(m.second);
		}
	}
	return nodes;
}

template <typename PointSourceType>
void NDTVoxel<PointSourceType>::computeCovariance()
{
	for (int mode = 0; mode < 2; mode++)
	{
		for (auto m : (*voxel_)[mode])
		{
			auto &node = m.second;
			Eigen::Vector4d centroid = Eigen::Vector4d(node->centroid(0), node->centroid(1), node->centroid(2), 1);
			Eigen::Matrix3d covariance;

			pcl::computeCovarianceMatrixNormalized(*cloud_, node->point_idx, centroid, covariance);
			node->covariance = covariance;
		}
	}
}

template <typename PointSourceType>
void NDTVoxel<PointSourceType>::getNodeBoundary(const NDTVoxelNode *node,
												double &min_x,
												double &min_y,
												double &min_z,
												double &max_x,
												double &max_y,
												double &max_z) const
{
	double x = node->idx(0);
	double y = node->idx(1);
	double z = node->idx(2);

	if (node->mode == 0)
	{
		min_x = x * resolution_;
		min_y = y * resolution_;
		min_z = z * resolution_;
		max_x = (x + 1) * resolution_;
		max_y = (y + 1) * resolution_;
		max_z = (z + 1) * resolution_;
	}
	else
	{
		min_x = (x - 0.5) * resolution_;
		min_y = (y - 0.5) * resolution_;
		min_z = (z - 0.5) * resolution_;
		max_x = (x + 0.5) * resolution_;
		max_y = (y + 0.5) * resolution_;
		max_z = (z + 0.5) * resolution_;
	}
}

template class NDTVoxel<pcl::PointXYZ>;
template class NDTVoxel<pcl::PointXYZI>;

} // namespace GlobalPlan
