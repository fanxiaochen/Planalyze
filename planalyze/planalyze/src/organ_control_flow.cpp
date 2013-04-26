#include <pcl/kdtree/kdtree_flann.h>

#include <boost/graph/adjacency_list.hpp>

#include "organ.h"
#include "color_map.h"
#include "cgal_types.h"
#include "main_window.h"
#include "cgal_utility.h"
#include "GCoptimization.h"
#include "parameter_manager.h"

#include "point_cloud.h"


void PointCloud::fillOrganPoints(void)
{
  int leaf_num = 0;
  int stem_num = 0;
  for (size_t i = 0; i < plant_points_num_; ++ i)
  {
    int organ_id = at(i).organ_id;
    if (organ_id == PclRichPoint::ID_UNINITIALIZED)
      continue;

    int label = at(i).label;
    if (label == PclRichPoint::LABEL_LEAF && organ_id < leaves_.size())
      leaves_[organ_id].addPoint(i);
    else if (label == PclRichPoint::LABEL_STEM && organ_id < stems_.size())
      stems_[organ_id].addPoint(i);
  }

  updateOrganIdAndFeature();

  return;
}

double PointCloud::computePointKdTreeDistance(const PclRichPoint& point, KdTreePtr kdtree)
{
  std::vector<int> neighbor_indices(1);
  std::vector<float> neighbor_squared_distances(1);
  int neighbor_num = kdtree->nearestKSearch(point, 1, neighbor_indices, neighbor_squared_distances);

  return neighbor_squared_distances[0];
}

double PointCloud::computeOrganKdTreeDistance(Organ& organ, KdTreePtr kdtree)
{
  double average_distance = 0.0;
  const std::vector<int>& indices = organ.getPointIndices();
  for (size_t i = 0, i_end = indices.size(); i < i_end; ++ i)
  {
    double distance = computePointKdTreeDistance(at(indices[i]), kdtree);
    average_distance += std::sqrt(distance);
  }
  average_distance /= indices.size();

  return average_distance;
}

PointCloud::KdTreePtr PointCloud::getStemKdTree(int id) const
{
  return getOrganKdTree(id, false);
}

PointCloud::KdTreePtr PointCloud::getLeafKdTree(int id) const
{
  return getOrganKdTree(id, true);
}

PointCloud::KdTreePtr PointCloud::getOrganKdTree(int id, bool leaf) const
{
  if ((leaf && id >= leaves_.size()) || (!leaf && id >= stems_.size()))
  {
    std::cout << "frame: " << getFrame() << "\tgetOrganKdTree: error id > num !" << std::endl;
    return KdTreePtr((pcl::KdTreeFLANN<PclRichPoint>*)(NULL));
  }
  
  const Organ& organ = (leaf)?(leaves_[id]):(stems_[id]);
  const std::vector<int>& organ_indices = organ.getPointIndices();
  boost::shared_ptr<std::vector<int> > indices(new std::vector<int>());
  indices->insert(indices->begin(), organ_indices.begin(), organ_indices.end());

  if (indices->empty())
  {
    std::cout << "frame: " << getFrame() << "\tgetOrganKdTree: empty indices !" << std::endl;
  }

  KdTreePtr kdtree(new pcl::KdTreeFLANN<PclRichPoint>(false));
  kdtree->setInputCloud(boost::shared_ptr<const PclRichPointCloud>(this, NullDeleter()), indices);

  return kdtree;
}

void PointCloud::visualizeOrgans(void)
{
  if (!show_organs_)
    return;

  for (size_t i = 0, i_end = stems_.size(); i < i_end; ++ i)
    stems_[i].visualize();
  for (size_t i = 0, i_end = leaves_.size(); i < i_end; ++ i)
    leaves_[i].visualize();

  return;
}

void PointCloud::printOrgans(void)
{
  std::cout << "frame: " << getFrame() << "\torgan num: " << stems_.size() + leaves_.size() << std::endl;
  for (size_t i = 0, i_end = stems_.size(); i < i_end; ++ i)
    std::cout << "\t\t" << (stems_[i].isLeaf()?("leaf"):("stem")) << "\tsize: " << stems_[i].getPointIndices().size() << std::endl;
  for (size_t i = 0, i_end = leaves_.size(); i < i_end; ++ i)
    std::cout << "\t\t" << (leaves_[i].isLeaf()?("leaf"):("stem")) << "\tsize: " << leaves_[i].getPointIndices().size() << std::endl;
  std::cout << std::endl;

  return;
}

void PointCloud::addOrgan(const std::vector<size_t>& organ_point_indices, bool leaf)
{
  size_t organ_id = ((leaf)?(leaves_.size()):(stems_.size()));
  if (organ_point_indices.empty())
  {
    std::cout << "frame: " << getFrame() << "\taddOrgan: error empty organ: "
      << ((leaf)?"leaf":"stem") << " " << organ_id << std::endl;
  }

  Organ organ(this, organ_id, leaf);
  for(size_t i = 0, i_end = organ_point_indices.size(); i < i_end; i++)
    organ.addPoint(organ_point_indices[i]);

  if (leaf)
  {
    leaves_.push_back(organ);
    leaves_.back().updateFeature();
  }
  else
  {
    stems_.push_back(organ);
    stems_.back().updateFeature();
  }

  return;
}

void PointCloud::initGcoGraphEdges(GCoptimizationGeneralGraph* gco, int smooth_cost)
{
  initPointGraph(ParameterManager::getInstance().getTriangleLength());

  double leaf_threshold = ParameterManager::getInstance().getCurvatureQuantize();

  boost::PointGraph& g_point = *point_graph_;
  typedef boost::PointGraphTraits::edge_iterator edge_iterator;
  std::pair<edge_iterator, edge_iterator> edges = boost::edges(g_point);
  for (edge_iterator it = edges.first; it != edges.second; ++ it)
  {
    size_t source_id = boost::source(*it, g_point);
    size_t target_id = boost::target(*it, g_point);
    assert (source_id < plant_points_num_ && target_id < plant_points_num_);
    float max_curvature = std::max(at(source_id).curvature, at(target_id).curvature);
    if (max_curvature < leaf_threshold)
      max_curvature = leaf_threshold;
    gco->setNeighbors(source_id, target_id, (int)(1/max_curvature));
  }

  // set smooth cost
  for (size_t i = 0; i < gco->numLabels(); ++ i)
  {
    for (size_t j = i+1; j < gco->numLabels(); ++ j)
    {
      gco->setSmoothCost(i, j, smooth_cost);
      gco->setSmoothCost(j, i, smooth_cost);
    }
  }

  return;
}

void PointCloud::initGcoGraphEdgesStems(GCoptimizationGeneralGraph* gco, int smooth_cost, const std::vector<size_t>& inverse_indices)
{
  initPointGraph(ParameterManager::getInstance().getTriangleLength());

  double leaf_threshold = ParameterManager::getInstance().getCurvatureQuantize();

  int site_num = gco->numSites();
  int label_num = gco->numLabels();

  boost::PointGraph& g_point = *point_graph_;
  typedef boost::PointGraphTraits::edge_iterator edge_iterator;
  std::pair<edge_iterator, edge_iterator> edges = boost::edges(g_point);
  for (edge_iterator it = edges.first; it != edges.second; ++ it)
  {
    size_t source_id = boost::source(*it, g_point);
    size_t target_id = boost::target(*it, g_point);
    assert (source_id < plant_points_num_ && target_id < plant_points_num_);

    if (at(source_id).label == PclRichPoint::LABEL_STEM
      && at(target_id).label == PclRichPoint::LABEL_STEM)
    {
      assert (inverse_indices[source_id] < site_num && inverse_indices[target_id] < site_num);
      float max_curvature = std::max(at(source_id).curvature, at(target_id).curvature);
      if (max_curvature < leaf_threshold)
        max_curvature = leaf_threshold;
      gco->setNeighbors(inverse_indices[source_id], inverse_indices[target_id], (int)(1/max_curvature));
    }
  }

  // set smooth cost
  for (size_t i = 0; i < gco->numLabels(); ++ i)
  {
    for (size_t j = i+1; j < gco->numLabels(); ++ j)
    {
      gco->setSmoothCost(i, j, smooth_cost);
      gco->setSmoothCost(j, i, smooth_cost);
    }
  }

  return;
}

void PointCloud::initGcoGraphEdgesLeaves(GCoptimizationGeneralGraph* gco, int smooth_cost, const std::vector<size_t>& inverse_indices)
{
  initPointGraph(ParameterManager::getInstance().getTriangleLength());

  double leaf_threshold = ParameterManager::getInstance().getCurvatureQuantize();

  int site_num = gco->numSites();
  int label_num = gco->numLabels();

  boost::PointGraph& g_point = *point_graph_;
  typedef boost::PointGraphTraits::edge_iterator edge_iterator;
  std::pair<edge_iterator, edge_iterator> edges = boost::edges(g_point);
  for (edge_iterator it = edges.first; it != edges.second; ++ it)
  {
    size_t source_id = boost::source(*it, g_point);
    size_t target_id = boost::target(*it, g_point);
    assert (source_id < plant_points_num_ && target_id < plant_points_num_);

    if (at(source_id).label == PclRichPoint::LABEL_LEAF
      && at(target_id).label == PclRichPoint::LABEL_LEAF)
    {
      assert (inverse_indices[source_id] < site_num && inverse_indices[target_id] < site_num);
      float max_curvature = std::max(at(source_id).curvature, at(target_id).curvature);
      if (max_curvature < leaf_threshold)
        max_curvature = leaf_threshold;
      gco->setNeighbors(inverse_indices[source_id], inverse_indices[target_id], (int)(1/max_curvature));
    }
  }

  // set smooth cost
  for (size_t i = 0; i < gco->numLabels(); ++ i)
  {
    for (size_t j = i+1; j < gco->numLabels(); ++ j)
    {
      gco->setSmoothCost(i, j, smooth_cost);
      gco->setSmoothCost(j, i, smooth_cost);
    }
  }

  return;
}

void PointCloud::updateOrganIdAndFeature(void)
{
  for (size_t i = 0; i < plant_points_num_; ++ i)
    at(i).organ_id = PclRichPoint::ID_UNINITIALIZED;

  for (size_t i = 0, i_end = stems_.size(); i < i_end; ++ i)
    stems_[i].setId(i);
  for (size_t i = 0, i_end = leaves_.size(); i < i_end; ++ i)
    leaves_[i].setId(i);

  for (size_t i = 0, i_end = stems_.size(); i < i_end; ++ i)
    stems_[i].updateFeature();
  for (size_t i = 0, i_end = leaves_.size(); i < i_end; ++ i)
    leaves_[i].updateFeature();

  return;
}

void PointCloud::backup(std::vector<int>& labels, std::vector<int>& ids, std::vector<Organ>& stems, std::vector<Organ>& leaves)
{
  labels.clear();
  ids.clear();
  for (size_t i = 0; i < plant_points_num_; ++ i)
  {
    labels.push_back(at(i).label);
    ids.push_back(at(i).organ_id);
  }
  stems = stems_;
  leaves = leaves_;

  return;
}
void PointCloud::rollback(std::vector<int>& labels, std::vector<int>& ids, std::vector<Organ>& stems, std::vector<Organ>& leaves)
{
  for (size_t i = 0; i < plant_points_num_; ++ i)
  {
    at(i).label = labels[i];
    at(i).organ_id = ids[i];
  }
  stems_ = stems;
  leaves_ = leaves;

  return;
}