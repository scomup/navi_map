#ifndef RANSAC_H_
#define RANSAC_H_

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <pcl/point_types.h>

#include <pcl/common/transforms.h>


#include <stdlib.h>

#include "NDTVoxelNode.h"
#include "common.h"

namespace GlobalPlan
{


static void getRondomNum(size_t l, size_t &a, size_t &b, size_t &c)
{
	while (true)
	{
		a = rand() % l;
		b = rand() % l;
		c = rand() % l;
		if (a != b && b != c && a != c)
			break;
	}
}
template <typename PointSourceType>
Eigen::Vector4d getRondomPlane(const typename pcl::PointCloud<PointSourceType>::Ptr point_cloud, const std::vector<int> &idx)
{
	size_t s = idx.size();
	size_t a;
	size_t b;
	size_t c;
	getRondomNum(s, a, b, c);
	PointSourceType pa = point_cloud->points[idx[a]];
	PointSourceType pb = point_cloud->points[idx[b]];
	PointSourceType pc = point_cloud->points[idx[c]];
	auto ea = Eigen::Vector3d(pa.x, pa.y, pa.z);
	auto eb = Eigen::Vector3d(pb.x, pb.y, pb.z);
	auto ec = Eigen::Vector3d(pc.x, pc.y, pc.z);
	auto o = (ea + eb + ec) / 3;
	auto v0 = eb - ea;
	auto v1 = ec - ea;
	auto normal = v0.cross(v1);
	normal /= normal.norm();
	return FindPlane<double>(o, normal);
}

template <typename PointSourceType>
Eigen::Vector4d RansacPlane(const typename pcl::PointCloud<PointSourceType>::Ptr point_cloud, const std::vector<int> &idx, std::vector<int> &inliers)
{
	double max_score = 0;
	Eigen::Vector4d best_plane;
	for (int l = 0; l < 100; l++)
	{
		double score = 0;
		auto plane = getRondomPlane<PointSourceType>(point_cloud, idx);
		for (auto i : idx)
		{
			auto &p = point_cloud->points[i];
			auto pp = Eigen::Vector3d(p.x, p.y, p.z);
			double dist = GetDistPlaneToPoint<double>(plane, pp);
			score += exp(-std::abs(dist) * 20);
		}
		if (score > max_score)
		{
			max_score = score;
			best_plane = plane;
		}
	}
	for (auto i : idx)
	{
		auto &p = point_cloud->points[i];
		auto pp = Eigen::Vector3d(p.x, p.y, p.z);
		double dist = GetDistPlaneToPoint<double>(best_plane, pp);
		if (dist < 0.05)
			inliers.push_back(i);
	}

	return best_plane;
}

template <typename PointSourceType>
NDTVoxelNode* RecomputeNode(const typename pcl::PointCloud<PointSourceType>::Ptr point_cloud, NDTVoxelNode *node)
{
	std::vector<int> inliers;
	GlobalPlan::RansacPlane<pcl::PointXYZ>(point_cloud, node->point_idx, inliers);
	int all_num = node->point_idx.size();
	//std::cout << "old number:" << node->point_idx.size() << std::endl;
	Eigen::Vector4d centroid;
	pcl::compute3DCentroid(*point_cloud, inliers, centroid);
	Eigen::Matrix3d covariance;
	Eigen::Vector3d centroid3d = centroid.head<3>();
	pcl::computeCovarianceMatrixNormalized(*point_cloud, inliers, centroid, covariance);
	int  inliers_num = inliers.size();
	double r = (double)(inliers_num)/(double)(all_num);
	if (r < 0.3)
		return nullptr;
	NDTVoxelNode *new_node = new NDTVoxelNode();

	new_node->mode = node->mode;
	new_node->idx = node->idx;
	new_node->centroid = centroid3d;
	new_node->covariance = covariance;
	new_node->point_idx = inliers;
	new_node->pnode = node->pnode;
	new_node->cnode = node->cnode;
	return new_node;
}

template <typename PointSourceType>
std::vector<int> computeInliers(const typename pcl::PointCloud<PointSourceType>::Ptr point_cloud, NDTVoxelNode *node)
{
	std::vector<int> inliers;

	auto &centroid = node->centroid;
	auto &covariance = node->covariance;

	Eigen::JacobiSVD<Eigen::Matrix3d> svd(covariance, Eigen::ComputeFullU | Eigen::ComputeFullV);

	if (std::isnan(svd.singularValues().x()) ||
		std::isnan(svd.singularValues().y()) ||
		std::isnan(svd.singularValues().z()))
	{
		return inliers;
	}

	Eigen::Matrix3d R(svd.matrixU());
	R.col(0).normalize();
	R.col(1).normalize();
	R.col(2) = R.col(0).cross(R.col(1));
	R.col(2).normalize();
	R.col(0) = R.col(1).cross(R.col(2));
	R.col(0).normalize();

	Eigen::Matrix3d S;
	S << sqrt(svd.singularValues().x()), 0, 0,
		0, sqrt(svd.singularValues().y()), 0,
		0, 0, sqrt(svd.singularValues().z());

	Eigen::Matrix3d T = Eigen::Matrix3d::Identity();
	T = R * S;
	Eigen::Matrix3d Tinv = T.inverse();

	for (size_t n : node->point_idx)
	{
		Eigen::Vector3d p = Eigen::Vector3d(point_cloud->points[n].x, point_cloud->points[n].y, point_cloud->points[n].z);
		p = p - centroid;
		Eigen::Vector3d v = Tinv * p;
		double dist = v.norm();
		if (dist < sqrt(6))
			inliers.push_back(n);
	}
	return inliers;
}

} // namespace GlobalPlan

#endif
