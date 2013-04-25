#include <pcl/kdtree/kdtree_flann.h>

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/connected_components.hpp>

#include "organ.h"
#include "color_map.h"
#include "cgal_types.h"
#include "main_window.h"
#include "cgal_utility.h"
#include "GCoptimization.h"
#include "parameter_manager.h"

#include "point_cloud.h"

void PointCloud::collectLeaves(void)
{
  int leaf_num = 0;
  for (size_t i = 0; i < plant_points_num_; ++ i)
  {
    if (at(i).label != PclRichPoint::LABEL_LEAF)
      continue;

    int organ_id = at(i).organ_id;
    if (organ_id == PclRichPoint::ID_UNINITIALIZED)
      continue;

    leaf_num = std::max(leaf_num, organ_id+1);
  }

  leaves_.clear();
  for (size_t i = 0; i < leaf_num; ++ i)
    leaves_.push_back(Organ(this, i, true));

  for (size_t i = 0; i < plant_points_num_; ++ i)
  {
    if (at(i).label == PclRichPoint::LABEL_LEAF && at(i).organ_id != PclRichPoint::ID_UNINITIALIZED)
      leaves_[at(i).organ_id].addPoint(i);
  }

  std::vector<Organ> non_empty_leaves;
  for (size_t i = 0; i < leaves_.size(); ++ i)
  {
    if (leaves_[i].getPointIndices().empty())
      continue;
    non_empty_leaves.push_back(leaves_[i]);
    non_empty_leaves.back().setId(non_empty_leaves.size()-1);
  }

  leaves_ = non_empty_leaves;

  return;
}

void PointCloud::trimOrgans(bool is_leaf)
{
  boost::PlainGraph g_leaf_stem(plant_points_num_);
  {
    initPointGraph(ParameterManager::getInstance().getTriangleLength());
    boost::PointGraph& g_point = *point_graph_;
    typedef boost::PointGraphTraits::edge_iterator edge_iterator;
    std::pair<edge_iterator, edge_iterator> edges = boost::edges(g_point);
    for (edge_iterator it = edges.first; it != edges.second; ++ it)
    {
      size_t source_id = boost::source(*it, g_point);
      size_t target_id = boost::target(*it, g_point);

      if (is_leaf)
      {
        if (at(source_id).label != PclRichPoint::LABEL_LEAF || at(target_id).label != PclRichPoint::LABEL_LEAF)
          continue;
      }
      else
      {
        if (at(source_id).label != PclRichPoint::LABEL_STEM || at(target_id).label != PclRichPoint::LABEL_STEM)
          continue;
      }

      if (at(source_id).organ_id != at(target_id).organ_id || at(source_id).organ_id == PclRichPoint::ID_UNINITIALIZED)
        continue;

      boost::add_edge(source_id, target_id, g_leaf_stem);
    }
  }

  std::vector<boost::PlainGraphTraits::vertex_descriptor> component(plant_points_num_);
  size_t component_num = boost::connected_components(g_leaf_stem, &component[0]);

  std::vector<std::vector<size_t> > components(component_num);
  for (size_t i = 0; i < plant_points_num_; ++ i)
    components[component[i]].push_back(i);

  std::vector<size_t> organ_size(is_leaf?(getLeafNum()):(getStemNum()), 0);
  for (size_t i = 0; i < component_num; ++ i)
  {
    size_t organ_id = at(components[i][0]).organ_id;
    if (organ_id == PclRichPoint::ID_UNINITIALIZED)
      continue;

    organ_size[organ_id] = std::max(organ_size[organ_id], components[i].size());
  }

  for (size_t i = 0; i < component_num; ++ i)
  {
    size_t organ_id = at(components[i][0]).organ_id;
    if (organ_id == PclRichPoint::ID_UNINITIALIZED)
      continue;

    if (components[i].size() == organ_size[organ_id])
      continue;

    for (size_t j = 0, j_end = components[i].size(); j < j_end; ++ j)
    {
      if (is_leaf)
        at(components[i][j]).label = PclRichPoint::LABEL_STEM;
      at(components[i][j]).organ_id = PclRichPoint::ID_UNINITIALIZED;
    }
  }

  if (is_leaf)
    collectLeaves();
  else
    collectStems();

  return;
}

void PointCloud::smoothLeaves(double smooth_cost, bool forward)
{
  QMutexLocker locker(&mutex_);

  osg::ref_ptr<PointCloud> p_ngbr = forward?getPrevFrame():getNextFrame();
  if (!p_ngbr.valid() || p_ngbr->getLeafNum() == 0)
    return;

  size_t leaf_num_ngbr = p_ngbr->getLeafNum();
  std::vector<Organ>& leaves_prev = p_ngbr->getLeaves();
  if (leaf_num_ngbr < 2)
    return;

  std::cout << "frame: " << getFrame() << "\tsmoothing leaves" << (forward?("forward"):("backward")) << "..." << std::endl;

  double power = 2.0;
  int cost_scale = 50;

  std::vector<size_t> inverse_indices(plant_points_num_, plant_points_num_);
  std::vector<size_t> site_indices;
  for (size_t i = 0; i < plant_points_num_; ++ i)
  {
    if (at(i).label != PclRichPoint::LABEL_LEAF)
      continue;
    inverse_indices[i] = site_indices.size();
    site_indices.push_back(i);
  }

  size_t site_num = site_indices.size();
  size_t leaf_num = leaves_prev.size();
  size_t label_num = leaf_num;
  boost::shared_ptr<GCoptimizationGeneralGraph> gco(new GCoptimizationGeneralGraph(site_num, label_num));
  initGcoGraphEdgesLeaves(gco.get(), smooth_cost, inverse_indices);

  std::vector<KdTreePtr> leaf_kdtrees;
  for (size_t i = 0; i < leaf_num; ++ i)
  {
    leaf_kdtrees.push_back(p_ngbr->getLeafKdTree(i));
  }

  double distance_threshold = ParameterManager::getInstance().getStemSkeletonRadius()*3;
  for (size_t i = 0; i < site_num; ++ i)
  {
    PclRichPoint& point = at(site_indices[i]);
    for (size_t j = 0, j_end = leaf_num; j < j_end; ++ j)
    {
      double distance = computePointKdTreeDistance(point, leaf_kdtrees[j]);
      gco->setDataCost(i, j, distance*cost_scale);
    }
  }

  int max_num_iterations = 16;
  gco->expansion(max_num_iterations);

  for (size_t i = 0; i < site_num; ++ i)
  {
    int label = gco->whatLabel(i);
    if (leaf_num != label_num && label == label_num-1)
      continue;
    at(site_indices[i]).organ_id = label;
  }

  collectLeaves();

  locker.unlock();
  trimOrgans(true);
  locker.relock();

  save(filename_);
  expire();

  return;
}