#include <QFile>
#include <QFileInfo>
#include <QFileDialog>
#include <QTextStream>
#include <QMessageBox>
#include <QDomElement>
#include <QDomDocument>

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/kruskal_min_spanning_tree.hpp>

#include <pcl/kdtree/kdtree_flann.h>

#include "color_map.h"
#include "point_cloud.h"
#include "main_window.h"
#include "osg_utility.h"
#include "parameter_manager.h"
#include "file_system_model.h"

#include "sketch_handler.h"

static osg::Vec3 computeIntersection(osgViewer::View* view, const osgGA::GUIEventAdapter& ea, bool first)
{
  osgUtil::LineSegmentIntersector::Intersections intersections;

  float x = ea.getX();
  float y = ea.getY();

  if (!view->computeIntersections(x,y,intersections))
    return osg::Vec3(std::numeric_limits<float>::quiet_NaN(), 0.0f, 0.0f);

  if (first)
    return intersections.begin()->getWorldIntersectPoint();

  osg::Vec3 position(0.0f, 0.0f, 0.0f);
  for(osgUtil::LineSegmentIntersector::Intersections::iterator it = intersections.begin(); it != intersections.end(); ++it) {
    position = position + it->getWorldIntersectPoint();
  }
  position = position/intersections.size();

  return position;
}

SkeletonSketcher::SkeletonSketcher(void)
  :current_path_(new osg::Vec3Array),
  skeleton_graph_(new boost::SkeletonGraph())
{
  hidden_ = true;
}

SkeletonSketcher::~SkeletonSketcher(void)
{
  delete skeleton_graph_;
}

void SkeletonSketcher::toggle(bool toggled)
{
  if (hidden_)
  {
    FileSystemModel* file_system_model = MainWindow::getInstance()->getFileSystemModel();
    osg::ref_ptr<PointCloud> point_cloud = file_system_model->getDisplayFirstFrame();
    if (point_cloud.valid())
      point_cloud->sampleSkeletonPoints(skeleton_graph_);
  }
  else
    skeleton_graph_->clear();

  toggleRendering();

  return;
}

void SkeletonSketcher::initializeSkeleton(void)
{
  boost::SkeletonGraph& stem_graph = *skeleton_graph_;

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

  return;
}


void SkeletonSketcher::updateImpl(void)
{
  osg::Vec4 joint_color = osg::Vec4(0.55f, 0.40f, 0.03f, 1.0f);
  double joint_radius = 0.5;

  boost::SkeletonGraph& stem_graph = *skeleton_graph_;
  for (size_t i = 0, i_end = boost::num_vertices(*skeleton_graph_); i < i_end; ++ i)
    addChild(OSGUtility::drawSphere(stem_graph[i], joint_radius, joint_color));

  osg::Vec4 current_color = osg::Vec4(0.8f, 0.2f, 0.2f, 1.0f);
  double current_radius = 0.8;
  if(!current_path_->empty())
    addChild(OSGUtility::drawSphere(current_path_->back(), current_radius, current_color));

  return;
}

bool SkeletonSketcher::handle(const osgGA::GUIEventAdapter& ea,osgGA::GUIActionAdapter& aa)
{
  if (hidden_)
    return false;

  switch(ea.getEventType())
  {
  case(osgGA::GUIEventAdapter::PUSH):
    {
      osgViewer::View* view = dynamic_cast<osgViewer::View*>(&aa);
      if (view)
      {
        if(ea.getModKeyMask()==osgGA::GUIEventAdapter::MODKEY_CTRL)
          addPoint(view, ea);
        else if(ea.getModKeyMask()==osgGA::GUIEventAdapter::MODKEY_SHIFT)
          removeEdge(view, ea);
        return false;
      }
    }
    break;
  default:
    return false;
  }

  return false;
}

void SkeletonSketcher::addPoint(osgViewer::View* view, const osgGA::GUIEventAdapter& ea)
{
  osg::Vec3 position = computeIntersection(view, ea, false);
  if (position.isNaN())
    return;

  current_path_->push_back(position);
  if (current_path_->size() == 2)
  {
    osg::ref_ptr<PointCloud> point_cloud = MainWindow::getInstance()->getFileSystemModel()->getDisplayFirstFrame();
    point_cloud->jointSkeleton(current_path_->at(0), current_path_->at(1));

    current_path_->clear();
  }

  expire();

  return;
}

void SkeletonSketcher::removeEdge(osgViewer::View* view, const osgGA::GUIEventAdapter& ea)
{
  osg::Vec3 position = computeIntersection(view, ea, false);
  if (position.isNaN())
    return;

  current_path_->push_back(position);
  if (current_path_->size() == 2)
  {
    osg::ref_ptr<PointCloud> point_cloud = MainWindow::getInstance()->getFileSystemModel()->getDisplayFirstFrame();
    point_cloud->breakSkeleton(current_path_->at(0), current_path_->at(1));

    current_path_->clear();
  }

  expire();

  return;
}

SketchHandler::SketchHandler(Sketcher* sketcher)
  :sketcher_(sketcher)
{}

SketchHandler::~SketchHandler(void)
{}

bool SketchHandler::handle(const osgGA::GUIEventAdapter& ea,osgGA::GUIActionAdapter& aa)
{
  return sketcher_->handle(ea, aa);
}