#include <QFutureWatcher>
#include <QtConcurrentRun>

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/connected_components.hpp>

#include <pcl/kdtree/kdtree_flann.h>

#include "organ.h"
#include "color_map.h"
#include "cgal_types.h"
#include "main_window.h"
#include "cgal_utility.h"
#include "GCoptimization.h"
#include "parameter_manager.h"

#include "point_cloud.h"

double PointCloud::transformCurvature(double curvature)
{
  double leaf_threshold = ParameterManager::getInstance().getCurvatureQuantize();
  if (curvature < leaf_threshold)
    curvature = leaf_threshold;

  return std::log10(curvature);
}

double PointCloud::getGlobalLeafFlatness(void)
{
  double leaf_threshold = ParameterManager::getInstance().getCurvatureQuantize();
  return std::log10(leaf_threshold);
}

double PointCloud::getGlobalStemFlatness(void)
{
  return std::log10(1/3.0);
}

double PointCloud::getGlobalLeafFeature(void)
{
  return getGlobalLeafFlatness();
}

double PointCloud::getGlobalStemFeature(void)
{
  return getGlobalStemFlatness();
}

double PointCloud::computePointLeafDistance(const PclRichPoint& point)
{
  double power = 2.0;
  int cost_scale = 50;

  double point_feature = transformCurvature(point.curvature);
  double leaf_feature = getGlobalLeafFlatness();
  double diff_to_leaf = std::max(point_feature-leaf_feature, 0.0);
  int leaf_cost = std::pow(diff_to_leaf, power)*cost_scale;
  return leaf_cost;
}

double PointCloud::computePointStemDistance(const PclRichPoint& point)
{
  double power = 2.0;
  int cost_scale = 50;

  double point_feature = transformCurvature(point.curvature);
  double stem_feature = getGlobalStemFlatness();
  double diff_to_stem = std::max(stem_feature-point_feature, 0.0);
  int stem_cost = std::pow(diff_to_stem, power)*cost_scale;
  return stem_cost;
}

double PointCloud::computePointLeafDistance(const PclRichPoint& point, const Organ& leaf)
{
  double power = 2.0;
  int cost_scale = 50;

  double point_feature = transformCurvature(point.curvature);
  double leaf_feature = leaf.getFlatness();
  double diff_to_leaf = std::max(point_feature-leaf_feature, 0.0);
  int leaf_cost = std::pow(diff_to_leaf, power)*cost_scale;
  return leaf_cost;
}

double PointCloud::computePointStemDistance(const PclRichPoint& point, const Organ& stem)
{
  double power = 2.0;
  int cost_scale = 50;

  double point_feature = transformCurvature(point.curvature);
  double stem_feature = stem.getFlatness();
  double diff_to_stem = std::max(stem_feature-point_feature, 0.0);
  int stem_cost = std::pow(diff_to_stem, power)*cost_scale;
  return stem_cost;
}

void PointCloud::classifyLeafStem(double smooth_cost, bool forward)
{
  for (size_t i = 0; i < plant_points_num_; ++ i)
  {
    at(i).label = PclRichPoint::LABEL_UNINITIALIZED;
    at(i).organ_id = PclRichPoint::ID_UNINITIALIZED;
  }

  osg::ref_ptr<PointCloud> point_cloud_reference;
  if (forward)
    point_cloud_reference = getPrevFrame();
  else
    point_cloud_reference = getNextFrame();

  if (!point_cloud_reference.valid() || 
    (point_cloud_reference->getStemNum() == 0 && point_cloud_reference->getLeafNum() == 0))
  {
    absoluteClassify(smooth_cost);
    return;
  }

  std::cout << "frame: " << getFrame() << "\tclassifying " << (forward?("forward"):("backward")) << "..." << std::endl;

  int cost_scale = 50;

  boost::shared_ptr<GCoptimizationGeneralGraph> gco(new GCoptimizationGeneralGraph(plant_points_num_, 2));
  initGcoGraphEdges(gco.get(), smooth_cost);

  std::vector<Organ>& leaves_reference = point_cloud_reference->getLeaves();
  std::vector<KdTreePtr> leaves_kdtrees(leaves_reference.size());
  for (size_t i = 0, i_end = leaves_reference.size(); i < i_end; ++ i)
    leaves_kdtrees[i] = point_cloud_reference->getLeafKdTree(i);
  
  std::vector<Organ>& stems_reference = point_cloud_reference->getStems();
  std::vector<KdTreePtr> stems_kdtrees(stems_reference.size());
  for (size_t i = 0, i_end = stems_reference.size(); i < i_end; ++ i)
    stems_kdtrees[i] = point_cloud_reference->getStemKdTree(i);

  for (size_t i = 0; i < plant_points_num_; ++ i)
  {
    int stem_cost, leaf_cost;

    if (stems_reference.empty())
      stem_cost = computePointStemDistance(at(i));
    else
    {
      float min_stem_distance = std::numeric_limits<float>::max();
      size_t min_stem_idx = 0;
      for (size_t j = 0, j_end = stems_reference.size(); j < j_end; ++ j)
      {
        float distance = computePointKdTreeDistance(at(i), stems_kdtrees[j]);
        if (min_stem_distance > distance)
        {
          min_stem_distance = distance;
          min_stem_idx = j;
        }
      }
      stem_cost = computePointStemDistance(at(i), stems_reference[min_stem_idx]);
    }
    gco->setDataCost(i, 0, stem_cost);


    if (leaves_reference.empty())
      leaf_cost = computePointLeafDistance(at(i));
    else
    {
      float min_leaf_distance = std::numeric_limits<float>::max();
      size_t min_leaf_idx = 0;
      for (size_t j = 0, j_end = leaves_reference.size(); j < j_end; ++ j)
      {
        float distance = computePointKdTreeDistance(at(i), leaves_kdtrees[j]);
        if (min_leaf_distance > distance)
        {
          min_leaf_distance = distance;
          min_leaf_idx = j;
        }
      }
      leaf_cost = computePointLeafDistance(at(i), leaves_reference[min_leaf_idx]);
    }
    gco->setDataCost(i, 1, leaf_cost);

    double total = leaf_cost + stem_cost;
    at(i).probability = stem_cost/total;
  }

  int max_num_iterations = 16;
  gco->expansion(max_num_iterations);
  for (size_t i = 0; i < plant_points_num_; ++ i)
  {
    int label = gco->whatLabel(i);
    at(i).label = (label == 0)?(PclRichPoint::LABEL_STEM):(PclRichPoint::LABEL_LEAF);
  }

  std::vector<std::vector<size_t> > leaf_components = computeLeafComponents();
  int leaf_component_size = ParameterManager::getInstance().getLeafComponentSize();
  for (size_t i = 0, i_end = leaf_components.size(); i < i_end; ++ i)
  {
    if (leaf_components[i].size() < leaf_component_size)
    {
      for (size_t j = 0, j_end = leaf_components[i].size(); j < j_end; ++ j)
      {
        at(leaf_components[i][j]).label = PclRichPoint::LABEL_STEM;
      }
    }
  }

  return;
}

void PointCloud::absoluteClassify(double smooth_cost)
{
  std::cout << "frame: " << getFrame() << "\tabs classifying ..." << std::endl;

  double power = 2.0;
  int cost_scale = 50;

  boost::shared_ptr<GCoptimizationGeneralGraph> gco(new GCoptimizationGeneralGraph(plant_points_num_, 2));
  initGcoGraphEdges(gco.get(), smooth_cost);

  //////////////////////////////////////////////////////////////////////////
  // set data cost
  //////////////////////////////////////////////////////////////////////////
  double stem_feature = getGlobalStemFeature();
  double leaf_feature = getGlobalLeafFeature();
  for (size_t i = 0; i < plant_points_num_; ++ i)
  {
    int stem_cost = computePointStemDistance(at(i));
    gco->setDataCost(i, 0, stem_cost);

    int leaf_cost = computePointLeafDistance(at(i));
    gco->setDataCost(i, 1, leaf_cost);

    double total = leaf_cost + stem_cost;
    at(i).probability = stem_cost/total;
  }

  int max_num_iterations = 16;
  gco->expansion(max_num_iterations);
  for (size_t i = 0; i < plant_points_num_; ++ i)
  {
    int label = gco->whatLabel(i);
    at(i).label = (label == 0)?(PclRichPoint::LABEL_STEM):(PclRichPoint::LABEL_LEAF);
  }

  std::vector<std::vector<size_t> > leaf_components = computeLeafComponents();
  int leaf_component_size = ParameterManager::getInstance().getLeafComponentSize();
  for (size_t i = 0, i_end = leaf_components.size(); i < i_end; ++ i)
  {
    if (leaf_components[i].size() < leaf_component_size)
    {
      for (size_t j = 0, j_end = leaf_components[i].size(); j < j_end; ++ j)
      {
        at(leaf_components[i][j]).label = PclRichPoint::LABEL_STEM;
      }
    }
  }

  return;
}