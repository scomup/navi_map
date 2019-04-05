
#include "NaviMap.h"

namespace GlobalPlan
{

flann::Matrix<double> static convertEigen2Flann(const Eigen::Vector3d &mat)
{
  flann::Matrix<double> out(new double[mat.rows() * mat.cols()],
                            mat.cols(), mat.rows());
  for (int i = 0; i < mat.cols(); i++)
  {
    for (int j = 0; j < mat.rows(); j++)
    {
      out[i][j] = mat(j, i);
    }
  }
  return out;
}

NaviMap::NaviMap(const double ndt_voxel_resolution, const double grid_resolution)
    : MultiLevelGrid(ndt_voxel_resolution, grid_resolution),
      kdtree_(flann::KDTreeSingleIndexParams()) {}

void NaviMap::setInput(const pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud)
{
  MultiLevelGrid::setInput(point_cloud);
  initKDtree();
}
Cell *NaviMap::FindNearestCell(const Eigen::Vector3d &position, double *distanceOut)
{

  // k-NN search (O(log(N)))
  flann::Matrix<double> query = convertEigen2Flann(position);

  std::vector<int> i(query.rows);
  flann::Matrix<int> indices(i.data(), query.rows, 1);
  std::vector<double> d(query.rows);
  flann::Matrix<double> dists(d.data(), query.rows, 1);

  kdtree_.knnSearch(query, indices, dists, 1, flann::SearchParams());
  int idx = indices[0][0];
  Eigen::Vector3d nearest = (Eigen::Vector3d)kdtree_.getPoint(idx);

  if (distanceOut)
  {
    *distanceOut = dists[0][0];
  }
  return cellmap3d_[nearest];

  return nullptr;
}

bool NaviMap::FindPath(const Cell *start,
                       const Cell *goal,
                       std::vector<Eigen::Vector3d> &path)
{
  if(start == nullptr){
    std::cout<<"bad start point."<<std::endl;
    return false;
  }

  if(goal == nullptr){
    std::cout<<"bad goal point."<<std::endl;
    return false;
  }

  CostTable cost_table;

  cost_table[start->position] = 0;

  std::set<std::pair<double, const Cell *>> front;
  front.insert(std::make_pair(cost_table[start->position], start));

  while (!front.empty())
  {
    auto top = front.begin();
    const Cell *cell = top->second;

    front.erase(top);

    for (auto next : cell->neighbours)
    {

      if (next == nullptr)
        continue;
      if (cost_table.find(next->position) == cost_table.end())
        cost_table[next->position] = inf;

      double weight = (next->position - cell->position).norm();
      if (cost_table[next->position] > cost_table[cell->position] + weight)
      {
        auto m = std::make_pair(cost_table[next->position], next);

        if (front.find(m) != front.end())
          front.erase(m);

        cost_table[next->position] = cost_table[cell->position] + weight;
        front.insert(std::make_pair(cost_table[next->position], next));

        /*
					if (front.find({cost_table[v], v}) != front.end())
						front.erase(front.find({cost_table[v], v}));
					cost_table[v] = cost_table[u] + weight;
					front.insert({cost_table[v], v});
					*/
      }
      if (next == goal)
      {
        std::cout << "OK goal!" << std::endl;

        setPath(cost_table, start, goal, path);
        return true;
      }
    }
  }
  return false;
}

void NaviMap::setPath(CostTable &cost_table,
                      const Cell *start,
                      const Cell *goal,
                      std::vector<Eigen::Vector3d> &path)
{
  const Cell *c = goal;

  double d = cost_table[goal->position];

  while (c != start)
  {
    path.push_back(c->position);
    for (const auto &next : c->neighbours)
    {
      if (next == nullptr)
        continue;

      if (cost_table.find(next->position) == cost_table.end())
        continue;

      if (d > cost_table[next->position])
      {
        d = cost_table[next->position];
        c = next;
      }
    }
  }
  path.push_back(start->position);
}

void NaviMap::initKDtree()
{
  assert(cellmap3d_.size() > 0);
  int dim = 3;
  std::vector<double> targetData;
  targetData.resize(cellmap3d_.size() * dim);
  int i = 0;
  for (auto &m : cellmap3d_)
  {
    auto &cell = m.second;
    targetData[i * dim + 0] = cell->position(0);
    targetData[i * dim + 1] = cell->position(1);
    targetData[i * dim + 2] = cell->position(2);
    i++;
  }
  flann::Matrix<double> dataset(targetData.data(), cellmap3d_.size(), dim);
  kdtree_ = flann::Index<flann::L2_Simple<double>>(
      dataset,
      flann::KDTreeSingleIndexParams());
  kdtree_.buildIndex();
  std::cout<<cellmap3d_.size()<<std::endl;

  for(int i = 0;i<cellmap3d_.size();i++){
    kdtree_.getPoint(i);
    std::cout<<i<<std::endl;
  }
}

} // namespace GlobalPlan
