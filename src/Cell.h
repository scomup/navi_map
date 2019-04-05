
#ifndef CELL_H_
#define CELL_H_

#include <vector>
#include <set>

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

namespace GlobalPlan
{

struct Cell
{
	Cell(double x, double y, double z)
	{
		position = Eigen::Vector3d(x, y, z);
		neighbours[0] = nullptr;
		neighbours[1] = nullptr;
		neighbours[2] = nullptr;
		neighbours[3] = nullptr;
		neighbours[4] = nullptr;
		neighbours[5] = nullptr;
		neighbours[6] = nullptr;
		neighbours[7] = nullptr;
		untraversable = false;
		dist = 10;
		
	}
	Eigen::Vector3d position;
	bool untraversable;
	double dist;
	std::array<Cell *, 8> neighbours;
};


}

#endif