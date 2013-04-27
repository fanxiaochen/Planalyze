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

void PointCloud::extendStems(void)
{
  QMutexLocker locker(&mutex_);

  double distance_threshold = ParameterManager::getInstance().getStemSkeletonRadius()/1.5;
  for (size_t round = 0; round < 5; ++ round)
  {
    for (size_t i = 0; i < plant_points_num_; ++ i)
    {
      PclRichPoint& point = at(i);
      if (point.label == PclRichPoint::LABEL_LEAF)
        continue;

      double min_distance = std::numeric_limits<double>::max();
      size_t min_idx = 0;
      for (size_t j = 0, j_end = stems_.size(); j < j_end; ++ j)
      {
        double distance = stems_[j].computeSkeletonToPointDistance(point.cast<CgalPoint>());
        if (min_distance > distance)
        {
          min_distance = distance;
          min_idx = j;
        }
      }
      if (std::sqrt(min_distance) > distance_threshold)
        continue;
      stems_[min_idx].addPoint(i);
    }

    updateOrganFeature();
  }

  save(filename_);
  expire();

  return;
}

void PointCloud::collectStems(void)
{
  int stem_num = 0;
  for (size_t i = 0; i < plant_points_num_; ++ i)
  {
    if (at(i).label != PclRichPoint::LABEL_STEM)
      continue;

    int organ_id = at(i).organ_id;
    if (organ_id == PclRichPoint::ID_UNINITIALIZED)
      continue;

    stem_num = std::max(stem_num, organ_id+1);
  }

  stems_.clear();
  for (size_t i = 0; i < stem_num; ++ i)
    stems_.push_back(Organ(this, i, false));

  for (size_t i = 0; i < plant_points_num_; ++ i)
  {
    if (at(i).label == PclRichPoint::LABEL_STEM && at(i).organ_id != PclRichPoint::ID_UNINITIALIZED)
      stems_[at(i).organ_id].addPoint(i);
  }

  for (size_t i = 0, i_end = stems_.size(); i < i_end; ++ i)
    stems_[i].updateFeature();

  double stem_length_threshold = ParameterManager::getInstance().getStemSkeletonLength();
  size_t component_size_threshold = ParameterManager::getInstance().getStemComponentSize()/3;
  std::vector<Organ> result_stems;
  for (size_t i = 0, i_end = stems_.size(); i < i_end; ++ i)
  {
    double length = stems_[i].getLength();
    size_t size = stems_[i].getPointIndices().size();
    if (length < stem_length_threshold && size < component_size_threshold)
      continue;

    result_stems.push_back(stems_[i]);
  }

  stems_ = result_stems;

  return;
}

void PointCloud::smoothStems(double smooth_cost, bool forward)
{
  QMutexLocker locker(&mutex_);

  osg::ref_ptr<PointCloud> p_ngbr = forward?getPrevFrame():getNextFrame();
  if (!p_ngbr.valid() || p_ngbr->getStemNum() == 0)
    return;

  size_t stem_num_ngbr = p_ngbr->getStemNum();
  std::vector<Organ>& stems_ngbr = p_ngbr->getStems();
  if (stem_num_ngbr < 2)
    return;

  double power = 2.0;
  int cost_scale = 50;
  int large_penalty = 2000;

  std::cout << "frame: " << getFrame() << "\tsmoothing stems" << (forward?("forward"):("backward")) << "..." << std::endl;

  std::vector<size_t> inverse_indices(plant_points_num_, plant_points_num_);
  std::vector<size_t> site_indices;
  for (size_t i = 0; i < plant_points_num_; ++ i)
  {
    if (at(i).label != PclRichPoint::LABEL_STEM)
      continue;
    inverse_indices[i] = site_indices.size();
    site_indices.push_back(i);
  }

  size_t site_num = site_indices.size();
  size_t stem_num = stems_ngbr.size();
  size_t label_num = stem_num + 1;
  boost::shared_ptr<GCoptimizationGeneralGraph> gco(new GCoptimizationGeneralGraph(site_num, label_num));
  initGcoGraphEdgesStems(gco.get(), smooth_cost, inverse_indices);

  double distance_threshold = ParameterManager::getInstance().getStemSkeletonRadius();
  for (size_t i = 0; i < site_num; ++ i)
  {
    PclRichPoint& point = at(site_indices[i]);
    CgalVector point_orientation(point.orientation_x, point.orientation_y, point.orientation_z);

    double min_distance = std::numeric_limits<double>::max();
    for (size_t j = 0, j_end = stem_num; j < j_end; ++ j)
    {
      double distance = stems_ngbr[j].computeSkeletonToPointDistance(point.cast<CgalPoint>());
      min_distance = std::min(min_distance, distance);
      if (distance > distance_threshold)
      {
        gco->setDataCost(i, j, large_penalty);
        continue;
      }

      CgalVector organ_orientation = stems_ngbr[j].getOrientation();
      int orientation_cost = (distance/distance_threshold+(point_orientation-organ_orientation).squared_length())*cost_scale;
      gco->setDataCost(i, j, orientation_cost);
    }

    if (stem_num < label_num)
      gco->setDataCost(i, label_num-1, (min_distance > distance_threshold)?(0):(large_penalty));
  }

  int max_num_iterations = 16;
  gco->expansion(max_num_iterations);
  for (size_t i = 0; i < site_num; ++ i)
  {
    int label = gco->whatLabel(i);
    if (stem_num != label_num && label == label_num-1)
      continue;
    at(site_indices[i]).organ_id = label;
  }

  collectStems();

  locker.unlock();
  trimOrgans(false);
  locker.relock();

  locker.unlock();
  extendStems();
  locker.relock();

  expire();

  return;
}