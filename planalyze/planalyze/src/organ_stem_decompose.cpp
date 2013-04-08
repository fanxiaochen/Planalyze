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
#include "registrator.h"
#include "cgal_utility.h"
#include "parameter_manager.h"
#include "point_cloud.h"

void PointCloud::sampleSkeletonPoints(void)
{
  QMutexLocker locker(&mutex_);

  boost::SkeletonGraph& g_stem_skeleton = *stem_skeleton_graph_;
  g_stem_skeleton.clear();
  {
    std::vector<size_t> stem_points;
    for (size_t i = 0; i < plant_points_num_; ++ i)
    {
      if (at(i).label == PclRichPoint::LABEL_LEAF)
        continue;
      stem_points.push_back(i);
    }
    std::random_shuffle(stem_points.begin(), stem_points.end());
    size_t sample_num = stem_points.size()/4;
    for (size_t i = 0; i < sample_num; ++ i)
      boost::add_vertex(at(stem_points[i]).cast<osg::Vec3>(), g_stem_skeleton);
  }

  Registrator* registrator = MainWindow::getInstance()->getRegistrator();
  osg::Vec3 point = registrator->getPivotPoint();
  // quantize the points
  int quantize = 4;
  for (size_t i = 0, i_end = boost::num_vertices(g_stem_skeleton); i < i_end; ++ i)
  {
    double distance = (g_stem_skeleton[i]-point).length();
    double quantize_distance = distance/quantize;
    quantize_distance = (int)(distance/quantize+0.5)*quantize;
    osg::Vec3 vector = g_stem_skeleton[i]-point;
    vector.normalize();
    g_stem_skeleton[i] = point + vector*quantize_distance;
  }

  expire();

  return;
}

void PointCloud::centerSkeletonPoints(void)
{
  boost::SkeletonGraph& g_stem_skeleton = *stem_skeleton_graph_;
  if (boost::num_vertices(g_stem_skeleton) == 0)
    return;

  QMutexLocker locker(&mutex_);

  Registrator* registrator = MainWindow::getInstance()->getRegistrator();
  CgalVector reg_normal = Caster<osg::Vec3, CgalVector>(registrator->getAxisNormal());

  pcl::PointCloud<PclPoint>::Ptr stem_skeleton(new pcl::PointCloud<PclPoint>());
  for (size_t i = 0, i_end = boost::num_vertices(g_stem_skeleton); i < i_end; ++ i)
    stem_skeleton->push_back(PclPoint(g_stem_skeleton[i]));

  double search_radius = ParameterManager::getInstance().getStemSkeletonRadius();

  for (size_t round = 0; round < 8; ++ round)
  {
    boost::shared_ptr<pcl::KdTreeFLANN<PclPoint> > kdtree(new pcl::KdTreeFLANN<PclPoint>(false));
    kdtree->setInputCloud(stem_skeleton);

    for (size_t i = 0, i_end = boost::num_vertices(g_stem_skeleton); i < i_end; ++ i)
    {
      osg::Vec3 center(0.0f, 0.0f, 0.0f);
      osg::Vec3 orientation(0.0f, 0.0f, 0.0f);
      {
        std::vector<int> neighbor_indices;
        std::vector<float> neighbor_squared_distances;
        int neighbor_num = kdtree_->radiusSearch(PclRichPoint(g_stem_skeleton[i]), search_radius, neighbor_indices, neighbor_squared_distances);

        double total_weight = 0.0;
        for (size_t j = 0; j < neighbor_num; ++ j)
        {
          if (at(neighbor_indices[j]).label != PclRichPoint::LABEL_STEM)
            continue;
          osg::Vec3 offset = at(neighbor_indices[j]).cast<osg::Vec3>();
          double weight = std::exp(-neighbor_squared_distances[j]/std::pow(search_radius, 2.0));
          center = center + offset*weight;
          total_weight += weight;
        }
        if (total_weight == 0.0)
          center = g_stem_skeleton[i];
        else
          center = center/total_weight;
      }

#define SH_METHOD
#ifdef SH_METHOD
      osg::Vec3 span(0.0f, 0.0f, 0.0f);
      {
        std::vector<int> neighbor_indices;
        std::vector<float> neighbor_squared_distances;
        int neighbor_num = kdtree->radiusSearch(stem_skeleton->at(i), search_radius, neighbor_indices, neighbor_squared_distances);
        double total_weight = 0.0;
        for (size_t j = 0; j < neighbor_num; ++ j)
        {
          osg::Vec3 offset = stem_skeleton->at(i).cast<osg::Vec3>()-stem_skeleton->at(neighbor_indices[j]).cast<osg::Vec3>();
          double weight = std::exp(-neighbor_squared_distances[j]/std::pow(search_radius, 2.0));
          span = span + offset*weight;
          total_weight += weight;
        }
        if (total_weight != 0.0)
          span = span/total_weight;
      }
      g_stem_skeleton[i] = center + span*0.4;
#else
      CgalPoint old_position = Caster<osg::Vec3, CgalPoint>(g_stem_skeleton[i]);
      CgalPoint new_position = Caster<osg::Vec3, CgalPoint>(center);
      CgalLine line(new_position, reg_normal);
      CgalPoint projection = line.projection(old_position);
      g_stem_skeleton[i] = Caster<CgalPoint, osg::Vec3>(projection);
#endif
    }

    for (size_t i = 0, i_end = i_end = boost::num_vertices(g_stem_skeleton); i < i_end; ++ i)
      stem_skeleton->at(i) = PclPoint(g_stem_skeleton[i]);
  }

  std::vector<bool> flags(boost::num_vertices(g_stem_skeleton), true);
  boost::shared_ptr<pcl::KdTreeFLANN<PclPoint> > kdtree(new pcl::KdTreeFLANN<PclPoint>(false));
  kdtree->setInputCloud(stem_skeleton);

  boost::SkeletonGraph g_filtered;
  double isolated_threshold = search_radius;
  double close_threshold = search_radius/4.0;
  close_threshold *= close_threshold;
  for (size_t i = 0, i_end = boost::num_vertices(g_stem_skeleton); i < i_end; ++ i)
  {
    if (!flags[i])
      continue;

    std::vector<int> neighbor_indices;
    std::vector<float> neighbor_squared_distances;
    int neighbor_num = kdtree->radiusSearch(stem_skeleton->at(i), isolated_threshold, neighbor_indices, neighbor_squared_distances);
    if (neighbor_num == 0)
    {
      flags[i] = false;
      continue;
    }

    boost::add_vertex(g_stem_skeleton[i], g_filtered);
    for (size_t j = 0; j < neighbor_num; ++ j)
    {
      if (neighbor_squared_distances[j] > close_threshold)
        continue;

      flags[neighbor_indices[j]] = false;
    }
  }
  g_stem_skeleton = g_filtered;

  expire();

  return;
}

void PointCloud::computeStemSkeletonMST(void)
{
  QMutexLocker locker(&mutex_);

  boost::SkeletonGraph& g_stem_skeleton = *stem_skeleton_graph_;

  pcl::PointCloud<PclPoint>::Ptr stem_skeleton(new pcl::PointCloud<PclPoint>());
  for (size_t i = 0, i_end = boost::num_vertices(g_stem_skeleton); i < i_end; ++ i)
    stem_skeleton->push_back(PclPoint(g_stem_skeleton[i]));
  boost::shared_ptr<pcl::KdTreeFLANN<PclPoint> > kdtree(new pcl::KdTreeFLANN<PclPoint>(false));
  kdtree->setInputCloud(stem_skeleton);

  boost::PointGraph g_skeleton_mst(boost::num_vertices(g_stem_skeleton));
  double distance_threshold_ = 2*ParameterManager::getInstance().getStemSkeletonRadius();
  for (size_t i = 0, i_end = boost::num_vertices(g_stem_skeleton); i < i_end; ++ i)
  {
    std::vector<int> neighbor_indices;
    std::vector<float> neighbor_squared_distances;
    int neighbor_num = kdtree->radiusSearch(stem_skeleton->at(i), distance_threshold_, neighbor_indices, neighbor_squared_distances);
    for (size_t j = 0; j < neighbor_num; ++ j)
    {
      WeightedEdge weighted_edge(std::sqrt(neighbor_squared_distances[j]), neighbor_squared_distances[j]);
      boost::add_edge(i, neighbor_indices[j], weighted_edge, g_skeleton_mst);
    }
  }

  typedef boost::PointGraphTraits::edge_descriptor edge_descriptor;
  typedef boost::property_map<boost::PointGraph, double WeightedEdge::*>::type weight_map_type;
  weight_map_type w_map = boost::get(&WeightedEdge::heat_kernel_distance, g_skeleton_mst);

  std::vector<edge_descriptor> spanning_tree;
  boost::kruskal_minimum_spanning_tree(g_skeleton_mst, std::back_inserter(spanning_tree), boost::weight_map(w_map));
  for (size_t i = 0, i_end = spanning_tree.size(); i < i_end; ++ i)
  {
    size_t source = boost::source(spanning_tree[i], g_skeleton_mst);
    size_t target = boost::target(spanning_tree[i], g_skeleton_mst);
    const WeightedEdge& weighted_edge = g_skeleton_mst[spanning_tree[i]];
    boost::add_edge(source, target, weighted_edge, g_stem_skeleton);
  }

  expire();

  return;
}

void PointCloud::filterStemSkeletonByDegree(void)
{
  QMutexLocker locker(&mutex_);

  boost::SkeletonGraph& g_stem_skeleton = *stem_skeleton_graph_;

  // filter by degree
  for (size_t i = 0, i_end = boost::num_vertices(g_stem_skeleton); i < i_end; ++ i)
  {
    if (boost::degree(i, g_stem_skeleton) <= 2)
      continue;

    boost::clear_vertex(i, g_stem_skeleton);
  }

  expire();

  return;
}

void PointCloud::filterStemSkeletonByAngle(void)
{
  QMutexLocker locker(&mutex_);

  boost::SkeletonGraph& g_stem_skeleton = *stem_skeleton_graph_;

  // filter by angle
  typedef boost::SkeletonGraph::out_edge_iterator out_edge_iterator;
  for (size_t i = 0, i_end = boost::num_vertices(g_stem_skeleton); i < i_end; ++ i)
  {
    if (boost::degree(i, g_stem_skeleton) <= 1)
      continue;

    std::pair<out_edge_iterator, out_edge_iterator> out_edges = boost::out_edges(i, g_stem_skeleton);
    out_edge_iterator it_0 = out_edges.first;
    out_edge_iterator it_2 = ++ out_edges.first;
    const osg::Vec3& point_0 = g_stem_skeleton[boost::target(*it_0, g_stem_skeleton)];
    const osg::Vec3& point_1 = g_stem_skeleton[i];
    const osg::Vec3& point_2 = g_stem_skeleton[boost::target(*it_2, g_stem_skeleton)];

    osg::Vec3 vector_0_1 = point_1-point_0;
    osg::Vec3 vector_1_2 = point_2-point_1;
    vector_0_1.normalize();
    vector_1_2.normalize();

    double angle = std::acos(vector_0_1*vector_1_2);
    if (angle < M_PI/6)
      continue;

    boost::clear_vertex(i, g_stem_skeleton);
  }

  expire();

  return;
}


void PointCloud::filterStemSkeletonByLength(void)
{
  QMutexLocker locker(&mutex_);

  std::vector<std::vector<size_t> > stem_components = extractStemComponents();
  for (size_t i = 0, i_end = stem_components.size(); i < i_end; ++ i)
    addOrgan(stem_components[i], false);

  expire();

  return;
}

std::vector<std::vector<size_t> > PointCloud::extractStemComponents(void)
{
  boost::SkeletonGraph& g_stem_skeleton = *stem_skeleton_graph_;

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
      skeleton_component.second += g_stem_skeleton[*current_it].geodesic_distance;

      if (boost::degree(this_id, g_stem_skeleton) != 2)
        break;

      std::pair<out_edge_iterator, out_edge_iterator> out_edges = boost::out_edges(this_id, g_stem_skeleton);
      out_edge_iterator it_0 = out_edges.first;
      out_edge_iterator it_2 = ++ out_edges.first;
      current_it = (prev_id == boost::target(*it_0, g_stem_skeleton))?(it_2):(it_0);
    } while (true);
  }

  std::vector<std::vector<CgalSegment> > skeletons(skeleton_components.size());
  for (size_t i = 0, i_end = skeleton_components.size(); i < i_end; ++ i)
  {
    std::vector<size_t>& skeleton_component = skeleton_components[i].first;
    for (size_t j = 0, j_end = skeleton_component.size()-1; j < j_end; ++ j)
    {
      CgalPoint source = Caster<osg::Vec3, CgalPoint>(g_stem_skeleton[skeleton_component[j]]);
      CgalPoint target = Caster<osg::Vec3, CgalPoint>(g_stem_skeleton[skeleton_component[j+1]]);
      skeletons[i].push_back(CgalSegment(source, target));

      // for visualization
      double id = -((double)i+1.0);
      g_stem_skeleton[boost::edge(skeleton_component[j], skeleton_component[j+1], g_stem_skeleton).first].heat_kernel_distance = id;
    }
  }

  double distance_threshold = ParameterManager::getInstance().getStemSkeletonRadius()/1.5;
  distance_threshold *= distance_threshold;
  std::vector<std::vector<size_t> > stem_components(skeleton_components.size());
  for (size_t i = 0; i < plant_points_num_; ++ i)
  {
    if (at(i).label == PclRichPoint::LABEL_LEAF)
      continue;

    CgalPoint point = at(i).cast<CgalPoint>();
    double min_distance = distance_threshold*5;
    size_t min_idx = 0;
    for (size_t j = 0, j_end = skeletons.size(); j < j_end; ++ j)
    {
      std::vector<CgalSegment>& segments = skeletons[j];
      for (size_t k = 0, k_end = segments.size(); k < k_end; ++ k)
      {
        double distance = CGAL::squared_distance(point, segments[k]);
        if (min_distance > distance)
        {
          min_distance = distance;
          min_idx = j;
        }
      }
    }

    if (min_distance < distance_threshold)
      stem_components[min_idx].push_back(i);
  }

  double stem_length_threshold = ParameterManager::getInstance().getStemSkeletonLength();
  size_t component_size_threshold = ParameterManager::getInstance().getStemComponentSize();

  std::vector<std::vector<size_t> > result_components;
  for (size_t i = 0, i_end = skeleton_components.size(); i < i_end; ++ i)
  {
    double length = skeleton_components[i].second;
    double size = stem_components[i].size();
    if (length < stem_length_threshold && size < component_size_threshold)
    {
      std::vector<size_t>& skeleton_component = skeleton_components[i].first;
      for (size_t j = 0, j_end = skeleton_component.size()-1; j < j_end; ++ j)
        boost::remove_edge(skeleton_component[j], skeleton_component[j+1], g_stem_skeleton);
      continue;
    }

    result_components.push_back(stem_components[i]);
  }
  stem_components = result_components;

  return stem_components;
}

std::vector<std::vector<size_t> > PointCloud::computeStemComponents(void)
{
  sampleSkeletonPoints();
  centerSkeletonPoints();
  computeStemSkeletonMST();
  filterStemSkeletonByDegree();
  //filterStemSkeletonByAngle();

  return extractStemComponents();
}

struct CompareComponentBySize
{
  bool operator()(const std::vector<size_t>& a, const std::vector<size_t>& b)
  {
    return a.size() > b.size();
  }
};

void PointCloud::absoluteDetectStems(void)
{
  QMutexLocker locker(&mutex_);

  std::cout << "frame: " << getFrame() << "\tabs detecting stems..." << std::endl;

  stems_.clear();

  locker.unlock();
  std::vector<std::vector<size_t> > stem_conponents = computeStemComponents();
  locker.relock();

  std::sort(stem_conponents.begin(), stem_conponents.end(), CompareComponentBySize());
  for (size_t i = 0, i_end = stem_conponents.size(); i < i_end; ++ i)
    addOrgan(stem_conponents[i], false);

  save(filename_);
  expire();

  return;
}

void PointCloud::visualizeStemGraph(void)
{
  if (!show_stem_graph_)
    return;

  int mod = ParameterManager::getInstance().getColorizeMod();
  osg::Vec4 node_color = osg::Vec4(0.55f, 0.40f, 0.03f, 1.0f);
  osg::Vec4 edge_color = osg::Vec4(0.85f, 0.65f, 0.13f, 1.0f);

  double node_radius = ParameterManager::getInstance().getStemSkeletonRadius()/6;
  double edge_radius = node_radius/2.0;

  boost::SkeletonGraph& g_stem_skeleton = *stem_skeleton_graph_;
  for (size_t i = 0, i_end = boost::num_vertices(g_stem_skeleton); i < i_end; ++ i)
  {
    osg::Vec4 color = node_color;
    if (boost::degree(i, g_stem_skeleton) != 0)
    {
      int id = -g_stem_skeleton[*boost::out_edges(i, g_stem_skeleton).first].heat_kernel_distance;
      if (id > 0)
      {
        id --;
        std::srand(id+mod);
        size_t color_idx = std::abs(std::rand())%mod;
        color = ColorMap::Instance().getColor(ColorMap::JET, color_idx, 0, mod-1);
      }
    }
    addChild(OSGUtility::drawSphere(g_stem_skeleton[i], node_radius, color));
  }

  typedef boost::SkeletonGraphTraits::edge_iterator edge_iterator_skeleton;
  std::pair<edge_iterator_skeleton, edge_iterator_skeleton> edges_skeleton = boost::edges(g_stem_skeleton);
  for (edge_iterator_skeleton it = edges_skeleton.first; it != edges_skeleton.second; ++ it)
  {
    osg::Vec4 color = edge_color;
    const osg::Vec3& source = g_stem_skeleton[boost::source(*it, g_stem_skeleton)];
    const osg::Vec3& target = g_stem_skeleton[boost::target(*it, g_stem_skeleton)];
    int id = -g_stem_skeleton[*it].heat_kernel_distance;
    if (id > 0)
    {
      id --;
      std::srand(id+mod);
      size_t color_idx = std::abs(std::rand())%mod;
      color = ColorMap::Instance().getColor(ColorMap::JET, color_idx, 0, mod-1);
    }

    addChild(OSGUtility::drawCylinder(source, target, edge_radius, color));
  }

  return;
}

void PointCloud::jointSkeleton(osg::Vec3 point_1, osg::Vec3 point_2)
{
  QMutexLocker locker(&mutex_);

  boost::SkeletonGraph& g_skeleton = *stem_skeleton_graph_;
  size_t skeleton_size = boost::num_vertices(g_skeleton);

  double min_distance_1 = std::numeric_limits<double>::max();
  double min_distance_2 = std::numeric_limits<double>::max();
  size_t min_idx_1 = 0;
  size_t min_idx_2 = 0;
  for (size_t i = 0; i < skeleton_size; ++ i)
  {
    double distance_1 = (g_skeleton[i]-point_1).length2();
    double distance_2 = (g_skeleton[i]-point_2).length2();

    if (min_distance_1 > distance_1)
    {
      min_distance_1 = distance_1;
      min_idx_1 = i;
    }
    if (min_distance_2 > distance_2)
    {
      min_distance_2 = distance_2;
      min_idx_2 = i;
    }
  }

  boost::add_edge(min_idx_1, min_idx_2, g_skeleton);

  stems_.clear();
  std::vector<std::vector<size_t> > stem_conponents = extractStemComponents();
  std::sort(stem_conponents.begin(), stem_conponents.end(), CompareComponentBySize());
  for (size_t i = 0, i_end = stem_conponents.size(); i < i_end; ++ i)
    addOrgan(stem_conponents[i], false);

  expire();

  return;
}