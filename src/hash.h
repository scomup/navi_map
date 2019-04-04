#ifndef HASH_H_
#define HASH_H_

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <boost/functional/hash.hpp>


/* The voxel is built on top of a voxel grid to fasten the nearest neighbor search */
namespace GlobalPlan
{


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

struct hashkey2i
{
	std::size_t operator()(const Eigen::Vector2i &state) const
	{
		size_t seed = 0;
		boost::hash_combine(seed, state.x());
		boost::hash_combine(seed, state.y());
		return seed;
	}
};

struct hashkey2d
{
	std::size_t operator()(const Eigen::Vector2d &state) const
	{
		size_t seed = 0;
		boost::hash_combine(seed, state.x());
		boost::hash_combine(seed, state.y());
		return seed;
	}
};
struct hashkey3d
{
	std::size_t operator()(const Eigen::Vector3d &state) const
	{
		size_t seed = 0;
		boost::hash_combine(seed, state.x());
		boost::hash_combine(seed, state.y());
		boost::hash_combine(seed, state.z());
		return seed;
	}
};


} // namespace GlobalPlan

#endif
