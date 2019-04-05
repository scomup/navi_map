#ifndef MULTILEVELGRID_H_
#define MULTILEVELGRID_H_

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <flann/algorithms/dist.h>
#include <flann/algorithms/kdtree_single_index.h>
#include <flann/flann.hpp>

#include "NDTVoxel.h"
#include "Cell.h"

namespace GlobalPlan
{

class MultiLevelGrid
{
    enum NodeType {PLANE, LINEAR, SPHERICAL};

  public:
    using CellMap2i = std::unordered_map<Eigen::Vector2i, std::vector<Cell *>, hashkey2i>;
    using CellMap3d = std::unordered_map<Eigen::Vector3d, Cell *, hashkey3d>;
    using CostTable = std::unordered_map<Eigen::Vector3d, double, hashkey3d>;

    MultiLevelGrid(const double ndt_voxel_resolution, const double grid_resolution);
    void setInput(const pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud);
    const std::vector<NDTVoxelNode*>& traversable_node() const;
    const std::vector<NDTVoxelNode*>& obstacle_node() const;
    const CellMap3d& cellmap3d() const;
    
std::vector<int> obstacle_idx;
//std::vector<int> traversable_idx;

  private:
    void NodeClassification();
    void NodeAnalysis(NDTVoxelNode *node, NodeType &type, double &theta);
    void makeTraversableGrid();
    //void makeObstacleKDtree();
    void computeObstacle();
    //double findNearestObstDist(const Eigen::Vector3d &position);
    void computeConection(Cell *cell);
    void computeConectionForAllCells();
    void computeInflation();


  private:
    boost::shared_ptr<NDTVoxel<pcl::PointXYZ>> voxel_;
    double ndt_voxel_resolution_;
    double grid_resolution_;
    std::vector<NDTVoxelNode*> traversable_node_;
    std::vector<NDTVoxelNode*> obstacle_node_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_;
    CellMap2i cellmap2i_;
    CellMap3d cellmap3d_;
    //flann::Index<flann::L2_Simple<double>> obst_kdtree_;

};
} // namespace GlobalPlan

#endif
