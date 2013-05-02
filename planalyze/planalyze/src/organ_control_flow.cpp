#include <QFileDialog>
#include <QTextStream>

#include <pcl/kdtree/kdtree_flann.h>

#include <boost/graph/adjacency_list.hpp>

#include <CGAL/Delaunay_triangulation_3.h>
#include <CGAL/Triangulation_vertex_base_with_info_3.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>

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
  if (stems_.empty() || leaves_.empty())
  {
    int stem_num = 0;
    int leaf_num = 0;
    for (size_t i = 0; i < plant_points_num_; ++ i)
    {
      int organ_id = at(i).organ_id;
      if (organ_id == PclRichPoint::ID_UNINITIALIZED)
        continue;

      int label = at(i).label;
      if (label == PclRichPoint::LABEL_LEAF)
        leaf_num = std::max(organ_id, leaf_num);
      else if (label == PclRichPoint::LABEL_STEM && organ_id < stems_.size())
        stem_num = std::max(organ_id, stem_num);
    }
    stems_.clear();
    for (size_t i = 0; i < stem_num+1; ++ i)
      stems_.push_back(Organ(this, i, false));
    leaves_.clear();
    for (size_t i = 0; i < leaf_num+1; ++ i)
      leaves_.push_back(Organ(this, i, true));
  }

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

  //updateOrganFeature();

  return;
}

double PointCloud::computePointKdTreeDistanceL2(const PclRichPoint& point, KdTreePtr kdtree)
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
    double distance = computePointKdTreeDistanceL2(at(indices[i]), kdtree);
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
  const Organ* organ = NULL;
  if (leaf)
  {
    for (size_t i = 0, i_end = leaves_.size(); i < i_end; ++ i)
    {
      if (leaves_[i].getId() == id)
      {
        organ = &leaves_[i];
        break;
      }
    }
  }
  else
  {
    for (size_t i = 0, i_end = stems_.size(); i < i_end; ++ i)
    {
      if (stems_[i].getId() == id)
      {
        organ = &stems_[i];
        break;
      }
    }
  }

  if (organ == NULL)
  {
    std::cout << "frame: " << getFrame() << "\tgetOrganKdTree: error id > num !" << std::endl;
    return KdTreePtr((pcl::KdTreeFLANN<PclRichPoint>*)(NULL));
  }

  const std::vector<int>& organ_indices = organ->getPointIndices();
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

void PointCloud::updateOrganFeature(void)
{
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

void PointCloud::reorderOrgans(bool forward)
{
  QMutexLocker locker(&mutex_);

  osg::ref_ptr<PointCloud> p_ngbr = forward?getPrevFrame():getNextFrame();
  if (!p_ngbr.valid())
    return;

  //std::vector<int> mapping;
  //std::vector<double> distances;

  //{
  //  size_t leaf_num = getLeafNum();
  //  size_t leaf_num_ngbr = p_ngbr->getLeafNum();
  //  if (leaf_num_ngbr != 0 && leaf_num != 0)
  //  {
  //    mapping.assign(leaf_num_ngbr, -1);
  //    distances.assign(leaf_num_ngbr, std::numeric_limits<double>::max());

  //    for (size_t i = 0; i < leaf_num_ngbr; ++ i)
  //    {
  //      double distance_min = std::numeric_limits<double>::max();
  //      size_t idx_min = 0;
  //      for (size_t j = 0; j < leaf_num; ++ j)
  //      {
  //        double distance = computeOrganKdTreeDistance(leaves_[j], p_ngbr->getOrganKdTree(p_ngbr->getLeaves()[i].getId(), true));;
  //        if (distance_min > distance)
  //        {
  //          distance_min = distance;
  //          idx_min = j;
  //        }
  //      }

  //      if (mapping[i] == -1 || distances[i] > distance_min)
  //      {
  //        mapping[i] = idx_min;
  //        distances[i] = distance_min;
  //      }
  //    }

  //    std::vector<bool> indices(leaf_num, true);
  //    size_t max_id = 0;
  //    for (size_t i = 0; i < leaf_num_ngbr; ++ i)
  //    {
  //      leaves_[mapping[i]].setId(p_ngbr->getLeaves()[i].getId());
  //      indices[mapping[i]] = false;
  //      max_id = std::max(max_id, p_ngbr->getLeaves()[i].getId());
  //      std::cout << mapping[i] << "->" << p_ngbr->getLeaves()[i].getId() << std::endl;
  //    }
  //    std::cout << std::endl;

  //    for (size_t i = 0; i < leaf_num; ++ i)
  //    {
  //      if (indices[i] == false)
  //        continue;
  //      leaves_[i].setId(++max_id);
  //    }
  //  }
  //}

  osg::Vec3 pivot_point(-77.158821, 66.510941, 980.320374);
  osg::Vec3 axis_normal(-0.128121, -0.815076, -0.565009);
  osg::Matrix transformation = osg::Matrix::translate(-pivot_point)*osg::Matrix::rotate(axis_normal, osg::Vec3(0, 0, 1));

  std::vector<CgalPoint> roots(5, CgalPoint(0, 0, std::numeric_limits<float>::max()));
  std::vector<CgalPoint> real_roots(5);
  for (size_t i = 0; i < 5; ++ i)
  {
    std::vector<CgalPoint>& skeleton = p_ngbr->getStems()[i].getSkeleton();
    for (size_t j = 0, j_end = skeleton.size(); j < j_end; ++ j)
    {
      osg::Vec3 point = Caster<CgalPoint, osg::Vec3>(skeleton[j]);
      point = transformation.preMult(point);

      if (roots[i].z() > point.z())
      {
        roots[i] = Caster<osg::Vec3, CgalPoint>(point);
        real_roots[i] = skeleton[j];
      }
    }
  }

  for (size_t i = 0; i < 5; ++ i)
  {
    std::vector<CgalPoint>& skeleton = stems_[i].getSkeleton();
    CgalPoint root(0, 0, std::numeric_limits<float>::max());
    size_t root_idx = 0;
    for (size_t j = 0, j_end = skeleton.size(); j < j_end; ++ j)
    {
      osg::Vec3 point = Caster<CgalPoint, osg::Vec3>(skeleton[j]);
      point = transformation.preMult(point);

      if (root.z() > point.z())
      {
        root = Caster<osg::Vec3, CgalPoint>(point);
        root_idx = j;
      }
    }

    double min_distance = std::numeric_limits<double>::max();
    size_t min_idx = 0;
    for (size_t j = 0; j < 5; ++ j)
    {
      double distance = CGAL::squared_distance(root, roots[j]);
      if (min_distance > distance)
      {
        min_distance = distance;
        min_idx = j;
      }
    }
    stems_[i].setId(p_ngbr->getStems()[min_idx].getId());

    if (root.z() > roots[min_idx].z())
    {
      skeleton[root_idx] = real_roots[min_idx];
    }
  }

  save(filename_);

  expire();

  return;
}

void PointCloud::saveTetObj(void)
{
  MainWindow* main_window = MainWindow::getInstance();
  QString filename = QFileDialog::getSaveFileName(main_window, "Save Tet Obj", main_window->getWorkspace(), "*.obj");
  if (filename.isEmpty())
    return;

  QString mtl_filename = QFileInfo(filename).path()+"/"+QFileInfo(filename).baseName()+".mtl";
  QFile mtl_file(mtl_filename);
  mtl_file.open(QIODevice::WriteOnly | QIODevice::Text);
  QTextStream mtl_file_stream(&mtl_file);

  QFile obj_file(filename);
  obj_file.open(QIODevice::WriteOnly | QIODevice::Text);
  QTextStream obj_file_stream(&obj_file);
  obj_file_stream << "mtllib " << mtl_filename << "\n";

  for (size_t i = 0; i < plant_points_num_; ++ i)
  {
    PclRichPoint& point = at(i);
    obj_file_stream << "v " << point.x << " " << point.y << " " << point.z << "\n";
  }
  obj_file_stream << "#vertices end\n\n";

  for (size_t i = 0; i < plant_points_num_; ++ i)
  {
    PclRichPoint& point = at(i);
    obj_file_stream << "vn " << point.normal_x << " " << point.normal_y << " " << point.normal_z << "\n";
  }
  obj_file_stream << "#normals end\n\n";

  double triangle_length = ParameterManager::getInstance().getTriangleLength();
  double triangle_length_threshold = triangle_length*triangle_length;

  for (size_t i = 0, i_end = leaves_.size(); i < i_end; ++ i)
  {
    size_t id = leaves_[i].getId();

    obj_file_stream << "usemtl leaf_" << i << "\n";
    obj_file_stream << "g leaf_" << i << "\n";
    int count = 0;
    double r, g, b;
    r = g = b = 0;
    for (CGAL::Delaunay::Finite_facets_iterator it = triangulation_->finite_facets_begin();
      it != triangulation_->finite_facets_end(); ++ it)
    {
      CGAL::Delaunay::Cell_handle cell_handle  = it->first;
      int                   vertex_index = it->second;

      bool this_leaf = true;
      for (size_t i = 1; i < 4; ++ i)
      {
        int source_index = (vertex_index+i)%4;
        size_t point_idx = cell_handle->vertex(source_index)->info();
        if (at(point_idx).organ_id != id || at(point_idx).label != PclRichPoint::LABEL_LEAF)
        {
          this_leaf = false;
          break;
        }
      }
      if (!this_leaf)
        continue;

      bool small_polygon = true;
      for (size_t i = 1; i < 4; ++ i)
      {
        int source_index = (vertex_index+i)%4;
        int target_index = (i == 3)?((vertex_index+1)%4):((vertex_index+i+1)%4);

        const CGAL::Delaunay::Point& source = cell_handle->vertex(source_index)->point();
        const CGAL::Delaunay::Point& target = cell_handle->vertex(target_index)->point();
        if (CGAL::squared_distance(source, target) > triangle_length_threshold)
        {
          small_polygon = false;
          break;
        }
      }
      if (!small_polygon)
        continue;

      obj_file_stream << "f ";
      for (size_t i = 1; i < 4; ++ i)
      {
        int source_index = (vertex_index+i)%4;
        size_t point_idx = cell_handle->vertex(source_index)->info();
        obj_file_stream << point_idx+1 << "//" << point_idx+1 << " ";

        r += at(point_idx).r;
        g += at(point_idx).g;
        b += at(point_idx).b;
      }
      count += 3;
      obj_file_stream << "\n";
    }

    mtl_file_stream << "newmtl leaf_" << i << "\n";
    count *= 255;
    mtl_file_stream << "Kd " << r/count << " " << g/count << " " << b/count << "\n\n";
  }

  for (size_t i = 0, i_end = stems_.size(); i < i_end; ++ i)
  {
    size_t id = stems_[i].getId();

    obj_file_stream << "usemtl stem_" << i << "\n";
    obj_file_stream << "g stem_" << i << "\n";
    int count = 0;
    double r, g, b;
    r = g = b = 0;
    for (CGAL::Delaunay::Finite_facets_iterator it = triangulation_->finite_facets_begin();
      it != triangulation_->finite_facets_end(); ++ it)
    {
      CGAL::Delaunay::Cell_handle cell_handle  = it->first;
      int                   vertex_index = it->second;

      bool this_stem = true;
      for (size_t i = 1; i < 4; ++ i)
      {
        int source_index = (vertex_index+i)%4;
        size_t point_idx = cell_handle->vertex(source_index)->info();
        if (at(point_idx).organ_id != id || at(point_idx).label != PclRichPoint::LABEL_STEM)
        {
          this_stem = false;
          break;
        }
      }
      if (!this_stem)
        continue;

      bool small_polygon = true;
      for (size_t i = 1; i < 4; ++ i)
      {
        int source_index = (vertex_index+i)%4;
        int target_index = (i == 3)?((vertex_index+1)%4):((vertex_index+i+1)%4);

        const CGAL::Delaunay::Point& source = cell_handle->vertex(source_index)->point();
        const CGAL::Delaunay::Point& target = cell_handle->vertex(target_index)->point();
        if (CGAL::squared_distance(source, target) > triangle_length_threshold)
        {
          small_polygon = false;
          break;
        }
      }
      if (!small_polygon)
        continue;

      obj_file_stream << "f ";
      for (size_t i = 1; i < 4; ++ i)
      {
        int source_index = (vertex_index+i)%4;
        size_t point_idx = cell_handle->vertex(source_index)->info();
        obj_file_stream << point_idx+1 << "//" << point_idx+1 << " ";

        r += at(point_idx).r;
        g += at(point_idx).g;
        b += at(point_idx).b;
      }
      count += 3;
      obj_file_stream << "\n";
    }

    mtl_file_stream << "newmtl stem_" << i << "\n";
    count *= 255;
    mtl_file_stream << "Kd " << r/count << " " << g/count << " " << b/count << "\n\n";
  }

  return;
}