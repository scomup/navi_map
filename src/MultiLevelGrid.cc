
#include "MultiLevelGrid.h"

namespace GlobalPlan
{



MultiLevelGrid::MultiLevelGrid(const double ndt_voxel_resolution, const double grid_resolution)
    : ndt_voxel_resolution_(ndt_voxel_resolution),
      grid_resolution_(grid_resolution)
{
  voxel_ = boost::make_shared<NDTVoxel<pcl::PointXYZ>>(ndt_voxel_resolution);
}

void MultiLevelGrid::setInput(const pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud)
{
  cloud_ = point_cloud;
  voxel_->setInput(point_cloud);
  NodeClassification();
  makeTraversableGrid();
}

void MultiLevelGrid::NodeAnalysis(NDTVoxelNode *node, NodeType &type, double &theta)
{
  auto covariance = node->covariance;

  Eigen::JacobiSVD<Eigen::Matrix3d> svd(covariance, Eigen::ComputeFullU | Eigen::ComputeFullV);

  bool is_plane = svd.singularValues().z() / svd.singularValues().y() < 0.08;
  bool is_linear = svd.singularValues().y() / svd.singularValues().x() < 0.08;

  if (is_plane)
    type = NodeType::PLANE;
  else if (is_linear)
    type = NodeType::LINEAR;
  else
    type = NodeType::SPHERICAL;

  Eigen::Vector3d plane_normal = svd.matrixU().rightCols<1>();
  Eigen::Vector3d z_axis = Eigen::Vector3d(0, 0, 1);
  plane_normal = plane_normal / plane_normal.norm();

  theta = acos(plane_normal.dot(z_axis));
}

void MultiLevelGrid::NodeClassification()
{
  auto nodes = voxel_->getAllNode();

  for (auto node : nodes)
  {

    if (node->point_idx.size() < 100)
      continue;

    NodeType type;
    double theta;
    NodeAnalysis(node, type, theta);

    if (type != NodeType::SPHERICAL && theta < M_PI / 6)
    {
      traversable_node_.push_back(node);
    }
    else if (type == NodeType::SPHERICAL && theta < M_PI / 6)
    {
      auto new_node = RecomputeNode<pcl::PointXYZ>(cloud_, node);
      NodeAnalysis(new_node, type, theta);
      if (theta < M_PI / 6)
        traversable_node_.push_back(new_node);

      std::vector<int> outliner;
      std::set_difference(node->point_idx.begin(),
                          node->point_idx.end(),
                          new_node->point_idx.begin(),
                          new_node->point_idx.end(),
                          std::inserter(outliner, outliner.begin()));
      //std::cout<<"in: "<<new_node->point_idx.size()<<" out: "<<outliner.size()<<std::endl;
      if (outliner.size() > 50)
      {
        NDTVoxelNode tmp_node;
        tmp_node.mode = node->mode;
        tmp_node.idx = node->idx;
        tmp_node.centroid = node->centroid;
        tmp_node.covariance = node->covariance;
        tmp_node.point_idx = outliner;
        tmp_node.pnode = node->pnode;
        tmp_node.cnode = node->cnode;

        auto new_node2 = RecomputeNode<pcl::PointXYZ>(cloud_, &tmp_node);
        NodeAnalysis(new_node2, type, theta);
        if (theta < M_PI / 6){
          traversable_node_.push_back(new_node2);
        }
      }
    }
    else
    {
      obstacle_node_.push_back(node);
    }
  }
}

void MultiLevelGrid::makeTraversableGrid()
{
  std::unordered_map<Eigen::Vector2i, std::vector<double>, hashkey2i> z_map;
  for (auto node : traversable_node_)
  {
    double min_x, min_y, min_z, max_x, max_y, max_z;
    voxel_->getNodeBoundary(node, min_x, min_y, min_z, max_x, max_y, max_z);
    auto centroid = node->centroid;
    auto covariance = node->covariance;
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(covariance, Eigen::ComputeFullU | Eigen::ComputeFullV);
    auto Tinv = MakeTinv(svd.matrixU(), svd.singularValues(), centroid);
    for (double x = min_x; x < max_x; x += grid_resolution_)
    {
      for (double y = min_y; y < max_y; y += grid_resolution_)
      {
        double z = GlobalPlan::GetZOnSurface<double>(Tinv, x, y);
        if(std::isnan(z))
          continue;
        int idx_x = std::round(x / grid_resolution_);
        int idx_y = std::round(y / grid_resolution_);
        z_map[Eigen::Vector2i( idx_x, idx_y)].push_back(z);
      }
    }
  }



  for (auto element : z_map)
  {
    
    double idx_x = (double)element.first.x() + 0.5;
    double idx_y = (double)element.first.y() + 0.5;
    double x = idx_x * grid_resolution_;
    double y = idx_y * grid_resolution_;

    auto &z_list = element.second;
    std::cout<<z_list.size()<<std::endl;
    std::sort(z_list.begin(), z_list.end());
    std::reverse(z_list.begin(), z_list.end());
    double z = z_list[0];
    for (auto z_next : z_list)
    {
      if ((z - z_next) > ndt_voxel_resolution_)
      {
        Cell *cell = new Cell(x, y, z);
        cellmap2d_[Eigen::Vector2d(x, y)].push_back(cell);
        cellmap3d_[cell->position] = cell;
        z = z_next;
      }
      //std::cout<<z_next<<std::endl;
    }
    //std::cout<<"-------------"<<std::endl;
    Cell *cell = new Cell(x, y, z);
    cellmap2d_[Eigen::Vector2d(x, y)].push_back(cell);
    cellmap3d_[cell->position] =cell;
  }
  
  
}

const std::vector<NDTVoxelNode *> &MultiLevelGrid::traversable_node() const
{
  return traversable_node_;
}

const std::vector<NDTVoxelNode *> &MultiLevelGrid::obstacle_node() const
{
  return obstacle_node_;
}

const MultiLevelGrid::CellMap3d& MultiLevelGrid::cellmap3d() const
{
  return cellmap3d_;
}

} // namespace GlobalPlan
