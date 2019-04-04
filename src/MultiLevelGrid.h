#ifndef MULTILEVELGRID_H_
#define MULTILEVELGRID_H_

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include "NDTVoxel.h"
#include "Cell.h"
#include "ransac.h"

namespace GlobalPlan
{

class MultiLevelGrid
{
    enum NodeType {PLANE, LINEAR, SPHERICAL};

  public:
    using CellMap2d = std::unordered_map<Eigen::Vector2d, std::vector<Cell *>, hashkey2d>;
    using CellMap3d = std::unordered_map<Eigen::Vector3d, Cell *, hashkey3d>;
    using CostTable = std::unordered_map<Eigen::Vector3d, double, hashkey3d>;

    MultiLevelGrid(const double ndt_voxel_resolution, const double grid_resolution);
    void setInput(const pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud);
    const std::vector<NDTVoxelNode*>& traversable_node() const;
    const std::vector<NDTVoxelNode*>& obstacle_node() const;
    const CellMap3d& cellmap3d() const;
    

  private:
    void NodeClassification();
    void NodeAnalysis(NDTVoxelNode *node, NodeType &type, double &theta);
    void makeTraversableGrid(NDTVoxelNode *node, NodeType &type, double &theta);
    void makeTraversableGrid();


  private:
    boost::shared_ptr<NDTVoxel<pcl::PointXYZ>> voxel_;
    double ndt_voxel_resolution_;
    double grid_resolution_;
    std::vector<NDTVoxelNode*> traversable_node_;
    std::vector<NDTVoxelNode*> obstacle_node_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_;
    CellMap2d cellmap2d_;
    CellMap3d cellmap3d_;
};
} // namespace GlobalPlan

#endif
