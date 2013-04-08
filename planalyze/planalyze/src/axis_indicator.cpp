#include <osg/LineSegment>

#include "main_window.h"
#include "osg_utility.h"
#include "osg_viewer_widget.h"
#include "axis_indicator.h"


AxisIndicator::AxisIndicator(void)
{
  toggleRendering();
}


AxisIndicator::~AxisIndicator(void)
{
}

void AxisIndicator::updateImpl()
{
  osg::BoundingSphere boundingSphere = MainWindow::getInstance()->getOSGViewerWidget()->getBound();
  osg::Vec3d center = boundingSphere.center();
  double length = boundingSphere.radius();
  double cylinder_thickness = 1;
  double cone_thickness = 2;
  osg::ref_ptr<osg::LineSegment> x(new osg::LineSegment(center, osg::Vec3(length, 0, 0)+center));
  osg::ref_ptr<osg::LineSegment> xArrow(new osg::LineSegment(osg::Vec3(length, 0, 0)+center, osg::Vec3(1.2*length, 0, 0)+center));
  addChild(OSGUtility::drawCylinder(*x, cylinder_thickness, osg::Vec4(1.0f, 0.0f, 0.0f, 1.0f)));
  addChild(OSGUtility::drawCone(*xArrow, cone_thickness, osg::Vec4(1.0f, 0.0f, 0.0f, 1.0f)));

  osg::ref_ptr<osg::LineSegment> y(new osg::LineSegment(center, osg::Vec3(0, length, 0)+center));
  osg::ref_ptr<osg::LineSegment> yArrow(new osg::LineSegment(osg::Vec3(0, length, 0)+center, osg::Vec3(0, 1.2*length, 0)+center));
  addChild(OSGUtility::drawCylinder(*y, cylinder_thickness, osg::Vec4(0.0f, 1.0f, 0.0f, 1.0f)));
  addChild(OSGUtility::drawCone(*yArrow, cone_thickness, osg::Vec4(0.0f, 1.0f, 0.0f, 1.0f)));

  osg::ref_ptr<osg::LineSegment> z(new osg::LineSegment(center, osg::Vec3(0, 0, length)+center));
  osg::ref_ptr<osg::LineSegment> zArrow(new osg::LineSegment(osg::Vec3(0, 0, length)+center, osg::Vec3(0, 0, 1.2*length)+center));
  addChild(OSGUtility::drawCylinder(*z, cylinder_thickness, osg::Vec4(0.0f, 0.0f, 1.0f, 1.0f)));
  addChild(OSGUtility::drawCone(*zArrow, cone_thickness, osg::Vec4(0.0f, 0.0f, 1.0f, 1.0f)));

  return;
}