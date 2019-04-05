
#include "MultiLevelGrid.h"
#include "ransac.h"

namespace GlobalPlan
{


MultiLevelGrid::MultiLevelGrid(const double ndt_voxel_resolution, const double grid_resolution)
    : voxel_(boost::make_shared<NDTVoxel<pcl::PointXYZ>>(ndt_voxel_resolution)),
      ndt_voxel_resolution_(ndt_voxel_resolution),
      grid_resolution_(grid_resolution){}

void MultiLevelGrid::setInput(const pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud)
{
  cloud_ = point_cloud;
  voxel_->setInput(point_cloud);
  std::cout << "Node classification.." << std::endl;
  NodeClassification();
  std::cout << "Make traversable grid.." << std::endl;
  makeTraversableGrid();
  std::cout << "Compute obstacle.." << std::endl;
  computeObstacle();
  std::cout << "Compute conection and edge.." << std::endl;
  computeConectionForAllCells();
  std::cout << "Compute cost inflation.." << std::endl;
  computeInflation();
}
/*
double MultiLevelGrid::findNearestObstDist(const Eigen::Vector3d &position)
{

  // k-NN search (O(log(N)))
  flann::Matrix<double> query = convertEigen2Flann(position);

  std::vector<int> i(query.rows);
  flann::Matrix<int> indices(i.data(), query.rows, 1);
  std::vector<double> d(query.rows);
  flann::Matrix<double> dists(d.data(), query.rows, 1);

  obst_kdtree_.knnSearch(query, indices, dists, 1, flann::SearchParams());

  return dists[0][0];
}*/

/*
void MultiLevelGrid::computeObstacleCost()
{
  for (auto &m : cellmap3d_)
  {
    auto &cell = m.second;
    auto dist = findNearestObstDist(cell->position);

    cell->untraversable[0] = dist > 0.1 ? 0 : 100;
  }
}*/

void MultiLevelGrid::computeInflation()
{

  std::set<std::pair<double, const Cell *>> front;

  for (auto &m : cellmap3d_)
  {
    auto &cell = m.second;
    if (cell->untraversable){
      cell->dist = 0; 
      front.insert(std::make_pair(0, cell));
    }
  }

		while (!front.empty())
		{
			auto top = front.begin();
			const Cell *cell = top->second;
      const double dist = cell->dist;

			front.erase(top);

      for (auto next : cell->neighbours)
      {


        if (next == nullptr)
          continue;

        double next_dist = dist + (next->position - cell->position).norm();
        //if(next_dist > 1.)
        //  continue;

        if (next->dist < next_dist)
          continue;
        next->dist = next_dist;
        front.insert(std::make_pair(next_dist, next));

      }
    }
}




void MultiLevelGrid::computeObstacle()
{
  //std::vector<int> obstacle_idx;
  for (auto &node : obstacle_node_)
  {
    auto inliers = computeInliers<pcl::PointXYZ>(cloud_, node);
    obstacle_idx.insert(obstacle_idx.end(), inliers.begin(), inliers.end());
  }



  for (int i : obstacle_idx)
  {
    
    auto& point = cloud_->points[i];
    int idx_x = std::round(point.x / grid_resolution_);
    int idx_y = std::round(point.y / grid_resolution_);

    auto it = cellmap2i_.find(Eigen::Vector2i(idx_x, idx_y));
    if (it == cellmap2i_.end())
      continue;
    std::vector<Cell*>& cells = it->second;
    for (auto cell : cells)
    {
      if (cell->position(2) < point.z)
      {
        if ( (point.z - cell->position(2)) < ndt_voxel_resolution_)
        {
          cell->untraversable = true;
          break;
        }
      }
    }
  }

  //z_map[Eigen::Vector2i(idx_x, idx_y)].push_back(z);

  /*
  int dim = 3;
  std::vector<double> targetData;
  targetData.resize(obstacle_idx.size() * dim);
  size_t position = 0;
  for (int i : obstacle_idx)
  {
    targetData[position * dim + 0] = cloud_->points[i].x;
    targetData[position * dim + 1] = cloud_->points[i].y;
    targetData[position * dim + 2] = cloud_->points[i].z;
    position++;
  }
  flann::Matrix<double> dataset(targetData.data(), obstacle_idx.size(), dim);
  obst_kdtree_ = flann::Index<flann::L2_Simple<double>>(
      dataset,
      flann::KDTreeSingleIndexParams());
  obst_kdtree_.buildIndex();
  */
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
  std::vector<NDTVoxelNode *> bad_nodes;

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
    else
    {
      bad_nodes.push_back(node);
    }
  }

  std::vector<NDTVoxelNode *> tmp_nodes;
  for (auto node : bad_nodes)
  {
    auto new_node = RecomputeNode<pcl::PointXYZ>(cloud_, node);
    if (new_node == nullptr)
      continue;
    tmp_nodes.push_back(new_node);
    std::vector<int> outliers;
    std::set_difference(node->point_idx.begin(),
                        node->point_idx.end(),
                        new_node->point_idx.begin(),
                        new_node->point_idx.end(),
                        std::inserter(outliers, outliers.begin()));
    if (outliers.size() < 150)
      continue;
    NDTVoxelNode tmp_node;
    tmp_node.mode = node->mode;
    tmp_node.idx = node->idx;
    tmp_node.centroid = node->centroid;
    tmp_node.covariance = node->covariance;
    tmp_node.point_idx = outliers;
    tmp_node.pnode = node->pnode;
    tmp_node.cnode = node->cnode;
    auto new_node2 = RecomputeNode<pcl::PointXYZ>(cloud_, &tmp_node);
    if (new_node2 == nullptr)
      continue;
    tmp_nodes.push_back(new_node2);
  }

  for (auto node : tmp_nodes)
  {

    NodeType type;
    double theta;
    NodeAnalysis(node, type, theta);

    if (type != NodeType::SPHERICAL && theta < M_PI / 6)
    {
      traversable_node_.push_back(node);
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
        if (std::isnan(z))
          continue;
        int idx_x = std::round(x / grid_resolution_);
        int idx_y = std::round(y / grid_resolution_);
        z_map[Eigen::Vector2i(idx_x, idx_y)].push_back(z);
      }
    }
  }

  for (auto element : z_map)
  {

    double idx_x = (double)element.first.x();
    double idx_y = (double)element.first.y();
    double x = idx_x * grid_resolution_;
    double y = idx_y * grid_resolution_;

    auto &z_list = element.second;
    std::sort(z_list.begin(), z_list.end());
    std::reverse(z_list.begin(), z_list.end());
    double z = z_list[0];
    for (auto z_next : z_list)
    {
      if ((z - z_next) > ndt_voxel_resolution_)
      {
        Cell *cell = new Cell(x, y, z);
        cellmap2i_[element.first].push_back(cell);
        cellmap3d_[cell->position] = cell;
        z = z_next;
      }
      //std::cout<<z_next<<std::endl;
    }
    //std::cout<<"-------------"<<std::endl;
    Cell *cell = new Cell(x, y, z);
    cellmap2i_[element.first].push_back(cell);
    cellmap3d_[cell->position] = cell;
  }
}

void MultiLevelGrid::computeConectionForAllCells()
{
  for (auto &m : cellmap3d_)
  {
    auto &cell = m.second;
    computeConection(cell);
  }
}

void MultiLevelGrid::computeConection(Cell *cell)
{

  double x = cell->position(0);
  double y = cell->position(1);
  double z = cell->position(2);

  int idx_x = std::round(x / grid_resolution_);
  int idx_y = std::round(y / grid_resolution_);

  std::vector<Eigen::Vector2i> idxs;
  idxs.push_back(Eigen::Vector2i(idx_x - 1, idx_y - 1));
  idxs.push_back(Eigen::Vector2i(idx_x - 1, idx_y - 0));
  idxs.push_back(Eigen::Vector2i(idx_x - 1, idx_y + 1));

  idxs.push_back(Eigen::Vector2i(idx_x - 0, idx_y - 1));
  idxs.push_back(Eigen::Vector2i(idx_x - 0, idx_y + 1));

  idxs.push_back(Eigen::Vector2i(idx_x + 1, idx_y - 1));
  idxs.push_back(Eigen::Vector2i(idx_x + 1, idx_y - 0));
  idxs.push_back(Eigen::Vector2i(idx_x + 1, idx_y + 1));
  int cont = 0;
  for (size_t i = 0; i < idxs.size(); i++)
  {
    cell->neighbours[i] = nullptr;
    auto &cells = cellmap2i_[idxs[i]];
    for (auto &cur : cells)
    {
      double cur_z = cur->position(2);
      if (std::abs(cur_z - z) < 2 * grid_resolution_)
      {
        cell->neighbours[i] = cur;
        cont++;
      }
    }
  }
  if (cont <= 6)
    cell->untraversable = true;
  
}

const std::vector<NDTVoxelNode *> &MultiLevelGrid::traversable_node() const
{
  return traversable_node_;
}

const std::vector<NDTVoxelNode *> &MultiLevelGrid::obstacle_node() const
{
  return obstacle_node_;
}

const MultiLevelGrid::CellMap3d &MultiLevelGrid::cellmap3d() const
{
  return cellmap3d_;
}

} // namespace GlobalPlan
