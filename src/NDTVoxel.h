#ifndef NDTVOXEL_H_
#define NDTVOXEL_H_

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <float.h>
#include <vector>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <unordered_map>
#include <boost/functional/hash.hpp>

#include "NDTVoxelNode.h"
#include "hash.h"
/* The voxel is built on top of a voxel grid to fasten the nearest neighbor search */
namespace GlobalPlan {


template <typename PointSourceType>
class NDTVoxel {
public:
	using Nodemap = std::unordered_map<Eigen::Vector3i, NDTVoxelNode*, hashkey3i>;

	NDTVoxel(const double resolution = 0.8);

	/* Input is a vector of boundaries and ptsum of the voxel grid
	 * Those boundaries is needed since the number of actually occupied voxels may be
	 * much smaller than reserved number of voxels */
	void setInput(const typename pcl::PointCloud<PointSourceType>::Ptr point_cloud);

	std::vector<NDTVoxelNode*> getAllNode() const;
	void getNodeBoundary(const NDTVoxelNode *node,
					   double &min_x,
					   double &min_y,
					   double &min_z,
					   double &max_x,
					   double &max_y,
					   double &max_z) const;

  private:
	Eigen::Vector3i getNodeIdxFromPoint(const PointSourceType point, const int mode)const;
	void computeCovariance();

	double resolution_;
	double step_;
	boost::shared_ptr<std::vector<Nodemap>> voxel_;
	Nodemap node_map_;
	typename pcl::PointCloud<PointSourceType>::Ptr cloud_;


};
}

#endif
