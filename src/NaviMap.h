#ifndef NAVIMAP_H_
#define NAVIMAP_H_

#include "MultiLevelGrid.h"

#include <flann/algorithms/dist.h>
#include <flann/algorithms/kdtree_single_index.h>
#include <flann/flann.hpp>

namespace GlobalPlan
{
constexpr double inf = std::numeric_limits<double>::infinity();

class NaviMap : public MultiLevelGrid
{
public:
  NaviMap(const double ndt_voxel_resolution, const double grid_resolution);
  void setInput(const pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud);
  Cell *FindNearestCell(const Eigen::Vector3d &position, double *distanceOut = nullptr);
  bool FindPath(const Cell *start, const Cell *goal, std::vector<Eigen::Vector3d> &path);

private:
  void initKDtree();
  void setPath(CostTable &cost_table,
               const Cell *start, 
               const Cell *goal, 
               std::vector<Eigen::Vector3d> &path);

private:
  flann::Index<flann::L2_Simple<double>> kdtree_;
};
} // namespace GlobalPlan

#endif
