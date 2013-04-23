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
  :current_path_(new osg::Vec3Array)
{
  hidden_ = true;
}

SkeletonSketcher::~SkeletonSketcher(void)
{
}

void SkeletonSketcher::toggle(bool toggled)
{
  hidden_ = !toggled;

  expire();

  return;
}


void SkeletonSketcher::updateImpl(void)
{
  osg::Vec4 joint_color = osg::Vec4(0.8f, 0.8f, 0.8f, 1.0f);
  double joint_radius = 0.5;

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
    point_cloud->deleteSkeleton(current_path_->at(0), current_path_->at(1));

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