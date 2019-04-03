#ifndef RANSAC_H_
#define RANSAC_H_

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <stdlib.h>

#include "OctreeNode.h"

/* The octree is built on top of a voxel grid to fasten the nearest neighbor search */
namespace GlobalPlan
{

//template <PointSourceType>
//void Octree<PointSourceType>::setInput(typename pcl::PointCloud<PointSourceType>::Ptr point_cloud)

template <typename FloatType>
FloatType GetDistPlaneToPoint(const Eigen::Matrix<FloatType, 4, 1> &plane,
							  const Eigen::Matrix<FloatType, 3, 1> &point)
{
	FloatType x = point.x();
	FloatType y = point.y();
	FloatType z = point.z();
	FloatType a = plane[0];
	FloatType b = plane[1];
	FloatType c = plane[2];
	FloatType d = plane[3];
	return std::abs(a * x + b * y + c * z + d) / std::sqrt(a * a + b * b + c * c);
}

template <typename FloatType>
Eigen::Matrix<FloatType, 4, 1> FindPlane(const Eigen::Matrix<FloatType, 3, 1> point, const Eigen::Matrix<FloatType, 3, 1> normal_vector)
{
	FloatType x = point.x();
	FloatType y = point.y();
	FloatType z = point.z();
	FloatType a = normal_vector.x();
	FloatType b = normal_vector.y();
	FloatType c = normal_vector.z();

	return Eigen::Matrix<FloatType, 4, 1>(a, b, c, -(a * x + b * y + c * z));
}

void getRondomNum(size_t l, size_t &a, size_t &b, size_t &c)
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
Eigen::Vector4d RansacPlane(const typename pcl::PointCloud<PointSourceType>::Ptr point_cloud, const std::vector<int> &idx, std::vector<int> &inliner)
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
			score += exp(-std::abs(dist) * 5);
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
		if (dist < 0.1)
			inliner.push_back(i);
	}

	return best_plane;
}

template <typename PointSourceType>
Eigen::Vector4d RecomputeNode(const typename pcl::PointCloud<PointSourceType>::Ptr point_cloud, OctreeNode *node)
{
	std::vector<int> inliner;
	GlobalPlan::RansacPlane<pcl::PointXYZ>(point_cloud, node->point_idx, inliner);
	std::cout << "old number:" << node->point_idx.size() << std::endl;
	Eigen::Vector4d centroid;
	pcl::compute3DCentroid(*point_cloud, inliner, centroid);
	Eigen::Matrix3d covariance;
	Eigen::Vector3d centroid3d = centroid.head<3>();
	pcl::computeCovarianceMatrixNormalized(*point_cloud, inliner, centroid, covariance);
	node->point_idx = inliner;
	node->centroid = centroid3d;
	node->covariance = covariance;
	std::cout << "new number:" << inliner.size() << std::endl;
}

} // namespace GlobalPlan

#endif
