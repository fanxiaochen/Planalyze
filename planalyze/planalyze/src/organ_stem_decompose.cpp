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
#include "sketch_handler.h"
#include "parameter_manager.h"
#include "point_cloud.h"

void PointCloud::sampleSkeletonPoints(void)
{
  QMutexLocker locker(&mutex_);

  boost::SkeletonGraph& stem_graph = *(MainWindow::getInstance()->getSkeletonSketcher()->getSkeletonGraph());
  stem_graph.clear();

  boost::shared_ptr<std::vector<int> > point_indices(new std::vector<int>());
  for (size_t i = 0; i < plant_points_num_; ++ i)
  {
    if (at(i).label != PclRichPoint::LABEL_STEM)
      continue;
    point_indices->push_back(i);
  }
  std::random_shuffle(point_indices->begin(), point_indices->end());
  size_t sample_num = point_indices->size()/20;
  for (size_t i = 0; i < sample_num; ++ i)
    boost::add_vertex(at(point_indices->at(i)).cast<osg::Vec3>(), stem_graph);

  pcl::PointCloud<PclPoint>::Ptr stem_skeleton(new pcl::PointCloud<PclPoint>());
  for (size_t i = 0, i_end = sample_num; i < i_end; ++ i)
    stem_skeleton->push_back(PclPoint(stem_graph[i]));

  KdTreePtr kdtree_stem(new pcl::KdTreeFLANN<PclRichPoint>(false));
  kdtree_stem->setInputCloud(boost::shared_ptr<const PclRichPointCloud>(this, NullDeleter()), point_indices);

  double search_radius = ParameterManager::getInstance().getStemSkeletonRadius();

  for (size_t round = 0; round < 128; ++ round)
  {
    boost::shared_ptr<pcl::KdTreeFLANN<PclPoint> > kdtree(new pcl::KdTreeFLANN<PclPoint>(false));
    kdtree->setInputCloud(stem_skeleton);

    for (size_t i = 0, i_end = stem_skeleton->size(); i < i_end; ++ i)
    {
      osg::Vec3 center(0.0f, 0.0f, 0.0f);
      osg::Vec3 orientation(0.0f, 0.0f, 0.0f);
      {
        std::vector<int> indices;
        std::vector<float> distances;
        int neighbor_num = kdtree_stem->radiusSearch(PclRichPoint(stem_graph[i]), search_radius, indices, distances);

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
          center = stem_graph[i];
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
      stem_graph[i] = center + span*0.4;
    }

    {
      double distance_threshold = search_radius*search_radius/16;
      pcl::PointCloud<PclPoint>::Ptr points_delete(new pcl::PointCloud<PclPoint>());
      for (size_t i = 0, i_end = stem_skeleton->size(); i < i_end; ++ i)
        points_delete->push_back(PclPoint(stem_graph[i]));
      boost::shared_ptr<pcl::KdTreeFLANN<PclPoint> > kdtree_delete(new pcl::KdTreeFLANN<PclPoint>(false));
      kdtree_delete->setInputCloud(points_delete);

      std::vector<bool> flags(stem_skeleton->size(), true);
      stem_skeleton->clear();
      stem_graph.clear();
      for (size_t i = 0, i_end = flags.size(); i < i_end; ++ i)
      {
        if (flags[i])
        {
          stem_skeleton->push_back(points_delete->at(i));
          boost::add_vertex(points_delete->at(i).cast<osg::Vec3>(), stem_graph);
        }

        std::vector<int> indices(1);
        std::vector<float> distances(1);
        int neighbor_num = kdtree_delete->nearestKSearch(points_delete->at(i), 1, indices, distances);

        if (distances.front() < distance_threshold)
        {
          neighbor_num = kdtree_delete->radiusSearch(points_delete->at(i), distance_threshold, indices, distances);
          for (size_t j = 0, j_end = indices.size(); j < j_end; ++ j)
            flags[indices[j]] = false;
        }
      }
    }
  }

  MainWindow::getInstance()->getSkeletonSketcher()->expire();

  return;
}

void PointCloud::initializeSkeleton(void)
{
  sampleSkeletonPoints();

  QMutexLocker locker(&mutex_);

  boost::SkeletonGraph& stem_graph = *(MainWindow::getInstance()->getSkeletonSketcher()->getSkeletonGraph());

  // compute MST
  pcl::PointCloud<PclPoint>::Ptr stem_skeleton(new pcl::PointCloud<PclPoint>());
  for (size_t i = 0, i_end = boost::num_vertices(stem_graph); i < i_end; ++ i)
    stem_skeleton->push_back(PclPoint(stem_graph[i]));
  boost::shared_ptr<pcl::KdTreeFLANN<PclPoint> > kdtree(new pcl::KdTreeFLANN<PclPoint>(false));
  kdtree->setInputCloud(stem_skeleton);

  boost::PointGraph g_skeleton_mst(boost::num_vertices(stem_graph));
  double distance_threshold_ = 2*ParameterManager::getInstance().getStemSkeletonRadius();
  for (size_t i = 0, i_end = boost::num_vertices(stem_graph); i < i_end; ++ i)
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
    boost::add_edge(source, target, weighted_edge, stem_graph);
  }

  // filter MST by degree
  for (size_t i = 0, i_end = boost::num_vertices(stem_graph); i < i_end; ++ i)
  {
    if (boost::degree(i, stem_graph) <= 2)
      continue;
    boost::clear_vertex(i, stem_graph);
  }

  // filter by angle
  typedef boost::SkeletonGraph::out_edge_iterator out_edge_iterator;
  for (size_t i = 0, i_end = boost::num_vertices(stem_graph); i < i_end; ++ i)
  {
    if (boost::degree(i, stem_graph) <= 1)
      continue;

    std::pair<out_edge_iterator, out_edge_iterator> out_edges = boost::out_edges(i, stem_graph);
    out_edge_iterator it_0 = out_edges.first;
    out_edge_iterator it_2 = ++ out_edges.first;
    const osg::Vec3& point_0 = stem_graph[boost::target(*it_0, stem_graph)];
    const osg::Vec3& point_1 = stem_graph[i];
    const osg::Vec3& point_2 = stem_graph[boost::target(*it_2, stem_graph)];

    osg::Vec3 vector_0_1 = point_1-point_0;
    osg::Vec3 vector_1_2 = point_2-point_1;
    vector_0_1.normalize();
    vector_1_2.normalize();

    double angle = std::acos(vector_0_1*vector_1_2);
    if (angle < M_PI/6)
      continue;

    boost::clear_vertex(i, stem_graph);
  }

  // extract stems
  typedef boost::SkeletonGraph::out_edge_iterator out_edge_iterator;

  std::vector<bool> flags(boost::num_vertices(stem_graph), true);
  std::vector<std::pair<std::vector<size_t>, double> > skeleton_components;
  for (size_t i = 0, i_end = boost::num_vertices(stem_graph); i < i_end; ++ i)
  {
    if (boost::degree(i, stem_graph) != 1 || !flags[i])
      continue;

    skeleton_components.push_back(std::make_pair(std::vector<size_t>(), 0.0));
    std::pair<std::vector<size_t>, double>& skeleton_component = skeleton_components.back();

    flags[i] = false;
    skeleton_component.first.push_back(i);
    out_edge_iterator current_it = boost::out_edges(i, stem_graph).first;
    do
    {
      size_t prev_id = boost::source(*current_it, stem_graph);
      size_t this_id = boost::target(*current_it, stem_graph);
      flags[this_id] = false;
      skeleton_component.first.push_back(this_id);
      skeleton_component.second += stem_graph[*current_it].length;

      if (boost::degree(this_id, stem_graph) != 2)
        break;

      std::pair<out_edge_iterator, out_edge_iterator> out_edges = boost::out_edges(this_id, stem_graph);
      out_edge_iterator it_0 = out_edges.first;
      out_edge_iterator it_2 = ++ out_edges.first;
      current_it = (prev_id == boost::target(*it_0, stem_graph))?(it_2):(it_0);
    } while (true);
  }

  double stem_length_threshold = ParameterManager::getInstance().getStemSkeletonLength();
  for (size_t i = 0, i_end = skeleton_components.size(); i < i_end; ++ i)
  {
    double length = skeleton_components[i].second;
    if (length >= stem_length_threshold)
      continue;

    std::vector<size_t>& point_indices = skeleton_components[i].first;
    for (size_t j = 0, j_end = point_indices.size()-1; j < j_end; ++ j)
      boost::remove_edge(point_indices[j], point_indices[j+1], stem_graph);
  }

  MainWindow::getInstance()->getSkeletonSketcher()->expire();

  return;
}

void PointCloud::extractStemSkeleton(void)
{
  QMutexLocker locker(&mutex_);

  boost::SkeletonGraph& stem_graph = *(MainWindow::getInstance()->getSkeletonSketcher()->getSkeletonGraph());

  // extract stems
  typedef boost::SkeletonGraph::out_edge_iterator out_edge_iterator;

  std::vector<bool> flags(boost::num_vertices(stem_graph), true);
  std::vector<std::pair<std::vector<size_t>, double> > skeleton_components;
  for (size_t i = 0, i_end = boost::num_vertices(stem_graph); i < i_end; ++ i)
  {
    if (boost::degree(i, stem_graph) != 1 || !flags[i])
      continue;

    skeleton_components.push_back(std::make_pair(std::vector<size_t>(), 0.0));
    std::pair<std::vector<size_t>, double>& skeleton_component = skeleton_components.back();

    flags[i] = false;
    skeleton_component.first.push_back(i);
    out_edge_iterator current_it = boost::out_edges(i, stem_graph).first;
    do
    {
      size_t prev_id = boost::source(*current_it, stem_graph);
      size_t this_id = boost::target(*current_it, stem_graph);
      flags[this_id] = false;
      skeleton_component.first.push_back(this_id);
      skeleton_component.second += stem_graph[*current_it].length;

      if (boost::degree(this_id, stem_graph) != 2)
        break;

      std::pair<out_edge_iterator, out_edge_iterator> out_edges = boost::out_edges(this_id, stem_graph);
      out_edge_iterator it_0 = out_edges.first;
      out_edge_iterator it_2 = ++ out_edges.first;
      current_it = (prev_id == boost::target(*it_0, stem_graph))?(it_2):(it_0);
    } while (true);
  }

  double stem_length_threshold = ParameterManager::getInstance().getStemSkeletonLength();
  stems_.clear();
  for (size_t i = 0, i_end = skeleton_components.size(); i < i_end; ++ i)
  {
    double length = skeleton_components[i].second;
    //if (length < stem_length_threshold)
    //  continue;

    stems_.push_back(Organ(this, stems_.size(), false));
    std::vector<CgalPoint> skeleton;
    std::vector<size_t>& point_indices = skeleton_components[i].first;
    for (size_t j = 0, j_end = point_indices.size(); j < j_end; ++ j)
      skeleton.push_back(Caster<osg::Vec3, CgalPoint>(stem_graph[point_indices[j]]));
    stems_.back().setSkeleton(skeleton);
  }

  saveStatus();

  expire();

  return;
}

void PointCloud::absoluteDetectStems(void)
{
  QMutexLocker locker(&mutex_);

  double stem_radius = ParameterManager::getInstance().getStemSkeletonRadius();

  for (size_t i = 0; i < plant_points_num_; ++ i)
  {
    if (at(i).label == PclRichPoint::LABEL_LEAF)
      continue;

    double min_distance = std::numeric_limits<double>::max();
    size_t min_organ = 0;
    for (size_t j = 0, j_end = stems_.size(); j < j_end; ++ j)
    {
      double distance = stems_[j].computeSkeletonToPointDistance(at(i).cast<CgalPoint>());
      if (distance < min_distance)
      {
        min_distance = distance;
        min_organ = j;
      }
    }
    if (min_distance < stem_radius*stem_radius)
      stems_[min_organ].addPoint(i);
    else
    {
      at(i).organ_id = PclRichPoint::ID_UNINITIALIZED;
      at(i).label = PclRichPoint::LABEL_NOISE;
    }
  }

  locker.unlock();
  //trimOrgans(false);
  locker.relock();

  //updateOrganFeature();

  expire();

  return;
}