#ifndef OCTREE_H_
#define OCTREE_H_

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

/* The octree is built on top of a voxel grid to fasten the nearest neighbor search */
namespace GlobalPlan {

struct hashkey3i
{
	std::size_t operator()(const Eigen::Vector3i &state) const
	{
		size_t seed = 0;
		boost::hash_combine(seed, state.x());
		boost::hash_combine(seed, state.y());
		boost::hash_combine(seed, state.z());
		return seed;
	}
};

struct OctreeNode
{
	int level;
	//Eigen::Vector3i idx;
	Eigen::Vector3d centroid;
	Eigen::Matrix3d covariance;
	std::vector<int> point_idx;
	OctreeNode* pnode;
	std::set<OctreeNode*> cnode;
};

template <typename PointSourceType>
class Octree {
public:
	using Nodemap = std::unordered_map<Eigen::Vector3i, OctreeNode*, hashkey3i>;

	Octree();

	/* Input is a vector of boundaries and ptsum of the voxel grid
	 * Those boundaries is needed since the number of actually occupied voxels may be
	 * much smaller than reserved number of voxels */
	void setInput(typename pcl::PointCloud<PointSourceType>::Ptr point_cloud);

	std::vector<OctreeNode*> getLevelNode(int level);



private:
  Eigen::Vector3i getNodeIdxFromPoint(PointSourceType point, int level);
  void computeCovariance();

  double resolution_;
  double step_;
  int max_level_;
  boost::shared_ptr<std::vector<Nodemap>> octree_;
  Nodemap node_map_;
  typename pcl::PointCloud<PointSourceType>::Ptr cloud_;


};
}

#endif
