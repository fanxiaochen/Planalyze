#include <QMutexLocker>

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/connected_components.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/graph/kruskal_min_spanning_tree.hpp>

#include <pcl/kdtree/kdtree_flann.h>

#include "organ.h"
#include "color_map.h"
#include "cgal_types.h"
#include "main_window.h"
#include "osg_utility.h"
#include "cgal_utility.h"
#include "parameter_manager.h"
#include "point_cloud.h"

void PointCloud::sampleSkeletonPoints(osg::Vec3Array* center_points)
{
  QMutexLocker locker(&mutex_);

  center_points->clear();
  std::vector<size_t> point_indices;
  for (size_t i = 0; i < plant_points_num_; ++ i)
  {
    if (at(i).label == PclRichPoint::LABEL_LEAF)
      continue;
    point_indices.push_back(i);
  }
  std::random_shuffle(point_indices.begin(), point_indices.end());
  size_t sample_num = point_indices.size()/4;
  for (size_t i = 0; i < sample_num; ++ i)
    center_points->push_back(at(point_indices[i]).cast<osg::Vec3>());


  pcl::PointCloud<PclPoint>::Ptr stem_skeleton(new pcl::PointCloud<PclPoint>());
  for (size_t i = 0, i_end = center_points->size(); i < i_end; ++ i)
    stem_skeleton->push_back(PclPoint(center_points->at(i)));

  double search_radius = ParameterManager::getInstance().getStemSkeletonRadius();

  for (size_t round = 0; round < 8; ++ round)
  {
    boost::shared_ptr<pcl::KdTreeFLANN<PclPoint> > kdtree(new pcl::KdTreeFLANN<PclPoint>(false));
    kdtree->setInputCloud(stem_skeleton);

    for (size_t i = 0, i_end = center_points->size(); i < i_end; ++ i)
    {
      osg::Vec3 center(0.0f, 0.0f, 0.0f);
      osg::Vec3 orientation(0.0f, 0.0f, 0.0f);
      {
        std::vector<int> indices;
        std::vector<float> distances;
        int neighbor_num = kdtree_->radiusSearch(PclRichPoint(center_points->at(i)), search_radius, indices, distances);

        double total_weight = 0.0;
        for (size_t j = 0; j < neighbor_num; ++ j)
        {
          if (at(indices[j]).label != PclRichPoint::LABEL_STEM)
            continue;
          osg::Vec3 offset = at(indices[j]).cast<osg::Vec3>();
          double weight = std::exp(-distances[j]/std::pow(search_radius, 2.0));
          center = center + offset*weight;
          total_weight += weight;
        }
        if (total_weight == 0.0)
          center = center_points->at(i);
        else
          center = center/total_weight;
      }

      osg::Vec3 span(0.0f, 0.0f, 0.0f);
      {
        std::vector<int> indices;
        std::vector<float> distances;
        int neighbor_num = kdtree->radiusSearch(stem_skeleton->at(i), search_radius, indices, distances);
        double total_weight = 0.0;
        for (size_t j = 0; j < neighbor_num; ++ j)
        {
          osg::Vec3 offset = stem_skeleton->at(i).cast<osg::Vec3>()-stem_skeleton->at(indices[j]).cast<osg::Vec3>();
          double weight = std::exp(-distances[j]/std::pow(search_radius, 2.0));
          span = span + offset*weight;
          total_weight += weight;
        }
        if (total_weight != 0.0)
          span = span/total_weight;
      }
      center_points->at(i) = center + span*0.4;
    }

    for (size_t i = 0, i_end = center_points->size(); i < i_end; ++ i)
      stem_skeleton->at(i) = PclPoint(center_points->at(i));
  }

  expire();

  return;
}

struct CompareComponentBySize
{
  bool operator()(const std::vector<size_t>& a, const std::vector<size_t>& b)
  {
    return a.size() > b.size();
  }
};

void PointCloud::jointSkeleton(osg::Vec3 point_1, osg::Vec3 point_2)
{
  QMutexLocker locker(&mutex_);


  expire();

  return;
}

void PointCloud::breakSkeleton(osg::Vec3 point_1, osg::Vec3 point_2)
{
  QMutexLocker locker(&mutex_);


  expire();

  return;
}

void PointCloud::initializeStemSkeleton(const osg::Vec3Array* center_points)
{
  QMutexLocker locker(&mutex_);

  boost::SkeletonGraph g_stem_skeleton(center_points->size());
  for (size_t i = 0, i_end = boost::num_vertices(g_stem_skeleton); i < i_end; ++ i)
    g_stem_skeleton[i] = center_points->at(i);

  // compute MST
  pcl::PointCloud<PclPoint>::Ptr stem_skeleton(new pcl::PointCloud<PclPoint>());
  for (size_t i = 0, i_end = boost::num_vertices(g_stem_skeleton); i < i_end; ++ i)
    stem_skeleton->push_back(PclPoint(g_stem_skeleton[i]));
  boost::shared_ptr<pcl::KdTreeFLANN<PclPoint> > kdtree(new pcl::KdTreeFLANN<PclPoint>(false));
  kdtree->setInputCloud(stem_skeleton);

  boost::PointGraph g_skeleton_mst(boost::num_vertices(g_stem_skeleton));
  double distance_threshold_ = 2*ParameterManager::getInstance().getStemSkeletonRadius();
  for (size_t i = 0, i_end = boost::num_vertices(g_stem_skeleton); i < i_end; ++ i)
  {
    std::vector<int> indices;
    std::vector<float> distances;
    int neighbor_num = kdtree->radiusSearch(stem_skeleton->at(i), distance_threshold_, indices, distances);
    for (size_t j = 0; j < neighbor_num; ++ j)
    {
      WeightedEdge weighted_edge(std::sqrt(distances[j]));
      boost::add_edge(i, indices[j], weighted_edge, g_skeleton_mst);
    }
  }

  typedef boost::PointGraphTraits::edge_descriptor edge_descriptor;
  typedef boost::property_map<boost::PointGraph, double WeightedEdge::*>::type weight_map_type;
  weight_map_type w_map = boost::get(&WeightedEdge::length, g_skeleton_mst);

  std::vector<edge_descriptor> spanning_tree;
  boost::kruskal_minimum_spanning_tree(g_skeleton_mst, std::back_inserter(spanning_tree), boost::weight_map(w_map));
  for (size_t i = 0, i_end = spanning_tree.size(); i < i_end; ++ i)
  {
    size_t source = boost::source(spanning_tree[i], g_skeleton_mst);
    size_t target = boost::target(spanning_tree[i], g_skeleton_mst);
    const WeightedEdge& weighted_edge = g_skeleton_mst[spanning_tree[i]];
    boost::add_edge(source, target, weighted_edge, g_stem_skeleton);
  }

  // filter MST by degree
  for (size_t i = 0, i_end = boost::num_vertices(g_stem_skeleton); i < i_end; ++ i)
  {
    if (boost::degree(i, g_stem_skeleton) <= 2)
      continue;
    boost::clear_vertex(i, g_stem_skeleton);
  }

  // extract stems
  typedef boost::SkeletonGraph::out_edge_iterator out_edge_iterator;

  std::vector<bool> flags(boost::num_vertices(g_stem_skeleton), true);
  std::vector<std::pair<std::vector<size_t>, double> > skeleton_components;
  for (size_t i = 0, i_end = boost::num_vertices(g_stem_skeleton); i < i_end; ++ i)
  {
    if (boost::degree(i, g_stem_skeleton) != 1 || !flags[i])
      continue;

    skeleton_components.push_back(std::make_pair(std::vector<size_t>(), 0.0));
    std::pair<std::vector<size_t>, double>& skeleton_component = skeleton_components.back();

    flags[i] = false;
    skeleton_component.first.push_back(i);
    out_edge_iterator current_it = boost::out_edges(i, g_stem_skeleton).first;
    do
    {
      size_t prev_id = boost::source(*current_it, g_stem_skeleton);
      size_t this_id = boost::target(*current_it, g_stem_skeleton);
      flags[this_id] = false;
      skeleton_component.first.push_back(this_id);
      skeleton_component.second += g_stem_skeleton[*current_it].length;

      if (boost::degree(this_id, g_stem_skeleton) != 2)
        break;

      std::pair<out_edge_iterator, out_edge_iterator> out_edges = boost::out_edges(this_id, g_stem_skeleton);
      out_edge_iterator it_0 = out_edges.first;
      out_edge_iterator it_2 = ++ out_edges.first;
      current_it = (prev_id == boost::target(*it_0, g_stem_skeleton))?(it_2):(it_0);
    } while (true);
  }

  double stem_length_threshold = ParameterManager::getInstance().getStemSkeletonLength();
  stems_.clear();
  for (size_t i = 0, i_end = skeleton_components.size(); i < i_end; ++ i)
  {
    double length = skeleton_components[i].second;
    if (length < stem_length_threshold)
      continue;

    stems_.push_back(Organ(this, stems_.size(), false));
    std::vector<CgalPoint> skeleton;
    std::vector<size_t>& point_indices = skeleton_components[i].first;
    for (size_t j = 0, j_end = point_indices.size(); j < j_end; ++ j)
      skeleton.push_back(Caster<osg::Vec3, CgalPoint>(g_stem_skeleton[point_indices[j]]));
    stems_.back().setSkeleton(skeleton);
  }

  expire();

  return;
}