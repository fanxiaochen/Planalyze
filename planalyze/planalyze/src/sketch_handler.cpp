#include <QFile>
#include <QFileInfo>
#include <QFileDialog>
#include <QTextStream>
#include <QMessageBox>
#include <QDomElement>
#include <QDomDocument>

#include <boost/graph/adjacency_list.hpp>

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
  if (!hidden_)
    skeleton_graph_->clear();

  toggleRendering();

  return;
}

void SkeletonSketcher::updateImpl(void)
{
  osg::Vec4 joint_color = osg::Vec4(0.55f, 0.40f, 0.03f, 1.0f);
  double joint_radius = 0.5;

  boost::SkeletonGraph& stem_graph = *skeleton_graph_;
  for (size_t i = 0, i_end = boost::num_vertices(*skeleton_graph_); i < i_end; ++ i)
    addChild(OSGUtility::drawSphere(stem_graph[i], joint_radius, joint_color));

  typedef boost::SkeletonGraph::edge_iterator edge_iterator;
  std::pair<edge_iterator, edge_iterator> edges = boost::edges(stem_graph);
  for (edge_iterator it = edges.first; it != edges.second; ++ it)
    addChild(OSGUtility::drawCylinder(stem_graph[boost::source(*it, stem_graph)], stem_graph[boost::target(*it, stem_graph)], joint_radius/1.5, joint_color));

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
          jointSkeleton(view, ea);
        else if(ea.getModKeyMask()==osgGA::GUIEventAdapter::MODKEY_SHIFT)
          breakSkeleton(view, ea);
        return false;
      }
    }
    break;
  default:
    return false;
  }

  return false;
}

void SkeletonSketcher::jointSkeleton(osgViewer::View* view, const osgGA::GUIEventAdapter& ea)
{
  QMutexLocker locker(&mutex_);

  osg::Vec3 position = computeIntersection(view, ea, false);
  if (position.isNaN())
    return;

  current_path_->push_back(position);
  if (current_path_->size() == 2)
  {
    boost::SkeletonGraph& skeleton_graph = *skeleton_graph_;
    size_t skeleton_size = boost::num_vertices(skeleton_graph);

    double min_distance_1 = std::numeric_limits<double>::max();
    double min_distance_2 = std::numeric_limits<double>::max();
    size_t min_idx_1 = 0;
    size_t min_idx_2 = 0;
    for (size_t i = 0; i < skeleton_size; ++ i)
    {
      double distance_1 = (skeleton_graph[i]-current_path_->at(0)).length2();
      double distance_2 = (skeleton_graph[i]-current_path_->at(1)).length2();

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

    boost::add_edge(min_idx_1, min_idx_2, skeleton_graph);

    current_path_->clear();
  }

  //if (current_path_->size() == 2)
  //{
  //  boost::SkeletonGraph& skeleton_graph = *skeleton_graph_;
  //  boost::add_vertex(current_path_->at(0), skeleton_graph);
  //  boost::add_vertex(current_path_->at(1), skeleton_graph);

  //  boost::add_edge(boost::num_vertices(skeleton_graph)-1, boost::num_vertices(skeleton_graph)-2, skeleton_graph);

  //  current_path_->clear();
  //}

  expire();

  return;
}

void SkeletonSketcher::breakSkeleton(osgViewer::View* view, const osgGA::GUIEventAdapter& ea)
{
  QMutexLocker locker(&mutex_);

  osg::Vec3 position = computeIntersection(view, ea, false);
  if (position.isNaN())
    return;

  current_path_->push_back(position);
  if (current_path_->size() == 2)
  {
    boost::SkeletonGraph& skeleton_graph = *skeleton_graph_;
    size_t skeleton_size = boost::num_vertices(skeleton_graph);

    double min_distance_1 = std::numeric_limits<double>::max();
    double min_distance_2 = std::numeric_limits<double>::max();
    size_t min_idx_1 = 0;
    size_t min_idx_2 = 0;
    for (size_t i = 0; i < skeleton_size; ++ i)
    {
      double distance_1 = (skeleton_graph[i]-current_path_->at(0)).length2();
      double distance_2 = (skeleton_graph[i]-current_path_->at(1)).length2();

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

    boost::remove_edge(min_idx_1, min_idx_2, skeleton_graph);

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