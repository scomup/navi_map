
#ifndef NDTVOXELNODE_H_
#define NDTVOXELNODE_H_

#include <vector>
#include <set>

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

namespace GlobalPlan
{

struct NDTVoxelNode
{
	int mode;
	Eigen::Vector3i idx;
	Eigen::Vector3d centroid;
	Eigen::Matrix3d covariance;
	std::vector<int> point_idx;
	NDTVoxelNode* pnode;
	std::set<NDTVoxelNode*> cnode;
};

}

#endif