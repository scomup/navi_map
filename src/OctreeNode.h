
#ifndef OCTREENODE_H_
#define OCTREENODE_H_

#include <vector>
#include <set>

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

namespace GlobalPlan
{

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

}

#endif