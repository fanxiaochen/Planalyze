#include <QFileInfo>

#include <boost/random.hpp>
#include <boost/random/normal_distribution.hpp>

#include <osgDB/ReadFile>
#include <osgUtil/IntersectionVisitor>
#include <osgUtil/LineSegmentIntersector>

#include "point_cloud.h"
#include "osg_utility.h"
#include "cgal_utility.h"
#include "parameter_manager.h"

#include "mesh_model.h"

MeshModel::MeshModel(void)
{

}

MeshModel::~MeshModel(void)
{

}

void MeshModel::virtualScan(double noise, double distance, double resolution)
{
  QMutexLocker locker(&mutex_);

  osg::Matrix switch_yz(osg::Matrix::rotate(osg::Vec3(0, 1, 0), osg::Vec3(0, 0, 1)));

#define MELON
  // for the synthetic melon
#ifdef MELON
  osg::Vec3 scale(8, 8, 8);
  osg::Vec3 pivot_point(-0.496659, 1.20558, 0.538896);
  osg::Vec3 axis_normal(0, -1, 0);
  osg::Matrix transformation = switch_yz*osg::Matrix::translate(pivot_point)*osg::Matrix::scale(scale)*osg::Matrix::translate(pivot_point);
#else
  // for the anthurium data set
  osg::Vec3 pivot_point(-13.382786, 50.223461, 917.477600);
  osg::Vec3 axis_normal(-0.054323, -0.814921, -0.577020);
  osg::Matrix transformation = osg::Matrix::rotate(osg::Vec3(0, 0, 1), axis_normal)*osg::Matrix::translate(pivot_point);
#endif

  osg::ref_ptr<osg::MatrixTransform> transform = new osg::MatrixTransform;
  transform->setMatrix(transformation);
  transform->addChild(model_node_);
  addChild(transform);

  boost::mt19937 rng;
  boost::normal_distribution<> nd(0.0, noise);
  boost::variate_generator<boost::mt19937&, boost::normal_distribution<> > var_nor(rng, nd);

  osg::ref_ptr<osgUtil::IntersectionVisitor> virtual_scanner = new osgUtil::IntersectionVisitor;

  PointCloud point_cloud;
  PclRichPoint point;
  point.label = PclRichPoint::LABEL_PLANT;
  point.r = 120;
  point.g = 120;
  point.b = 120;

  osg::Vec3 center = getBound().center();
  double radius = getBound().radius();

  axis_normal.normalize();
  CgalPlane plane((CgalPoint)Caster<osg::Vec3, CgalPoint>(center), Caster<osg::Vec3, CgalVector>(axis_normal));
  osg::Vec3 base1 = Caster<CgalVector, osg::Vec3>(plane.base1());
  base1.normalize();
  osg::Vec3 eye_offset = axis_normal*(0.5*distance) + base1*0.5*distance*std::sqrt(3);

  plane = CgalPlane((CgalPoint)Caster<osg::Vec3, CgalPoint>(center), Caster<osg::Vec3, CgalVector>(eye_offset));
  osg::Vec3 z_offset = Caster<CgalVector, osg::Vec3>(plane.base1());
  osg::Vec3 xy_offset = Caster<CgalVector, osg::Vec3>(plane.base2());
  z_offset.normalize(); z_offset = z_offset*resolution;
  xy_offset.normalize(); xy_offset = xy_offset*resolution;

  int grid_num = std::max(1, (int)(radius/resolution));
  for (size_t i = 0; i <= 13; ++ i)
  {
    osg::Vec3 view_direction = eye_offset*osg::Matrix::rotate(M_PI/6*i, axis_normal);
    osg::Vec3 eye = view_direction + center;
    osg::Vec3 z = z_offset*osg::Matrix::rotate(M_PI/6*i, axis_normal);
    osg::Vec3 xy = xy_offset*osg::Matrix::rotate(M_PI/6*i, axis_normal);

    for (int j = -grid_num; j < grid_num; ++ j)
    {
      for (int k = -grid_num; k < grid_num; ++ k)
      {
        osg::Vec3 ray = center + z*j + xy*k;
        osg::Vec3 ray_direction = ray-eye;
        ray_direction.normalize();
        ray = ray + ray_direction*radius;
        osg::ref_ptr<osgUtil::LineSegmentIntersector> virtual_ray = new osgUtil::LineSegmentIntersector(eye, ray);
        virtual_scanner->setIntersector(virtual_ray);
        this->accept(*virtual_scanner);
        if (virtual_ray->getIntersections().empty())
          continue;

        osgUtil::LineSegmentIntersector::Intersection intersection = virtual_ray->getFirstIntersection();
        const osg::Vec3& position = intersection.getWorldIntersectPoint();
        const osg::Vec3& normal = intersection.getWorldIntersectNormal();
        point.x = position.x() + var_nor();
        point.y = position.y() + var_nor();
        point.z = position.z() + var_nor();
        point.normal_x = normal.x();
        point.normal_y = normal.y();
        point.normal_z = normal.z();
        point_cloud.push_back(point);
      }
    }
  }
  std::cout << "Saving point cloud: " << QFileInfo(filename_.c_str()).path().toStdString()+"/points.pcd..." << std::endl;
  point_cloud.save(QFileInfo(filename_.c_str()).path().toStdString()+"/points.pcd");

  return;
}

bool MeshModel::open(const std::string& filename)
{
  model_node_ = osgDB::readNodeFile(filename);

  if (!model_node_.valid())
    return false;

  filename_ = filename;

  expire();

  return true;
}

void MeshModel::updateImpl()
{
  osg::Matrix switch_yz(osg::Matrix::rotate(osg::Vec3(0, 1, 0), osg::Vec3(0, 0, 1)));

#define MELON
#ifdef MELON
  // for the synthetic melon
  osg::Vec3 scale(8, 8, 8);
  osg::Vec3 pivot_point(-0.496659, 1.20558, 0.538896);
  osg::Vec3 axis_normal(0, -1, 0);
  osg::Matrix transformation = switch_yz*osg::Matrix::translate(pivot_point)*osg::Matrix::scale(scale)*osg::Matrix::translate(pivot_point);
#else
  // for the anthurium data set
  osg::Vec3 pivot_point(-13.382786, 50.223461, 917.477600);
  osg::Vec3 axis_normal(-0.054323, -0.814921, -0.577020);
  osg::Matrix transformation = osg::Matrix::rotate(osg::Vec3(0, 0, 1), axis_normal)*osg::Matrix::translate(pivot_point);
#endif

  osg::ref_ptr<osg::MatrixTransform> transform = new osg::MatrixTransform;
  transform->setMatrix(transformation);
  transform->addChild(model_node_);

  addChild(transform);

  osg::Vec3 center = getBound().center();
  double radius = getBound().radius();
  double distance = ParameterManager::getInstance().getVirtualScanDistance();

  axis_normal.normalize();
  CgalPlane plane((CgalPoint)Caster<osg::Vec3, CgalPoint>(center), Caster<osg::Vec3, CgalVector>(axis_normal));
  osg::Vec3 base1 = Caster<CgalVector, osg::Vec3>(plane.base1());
  base1.normalize();
  osg::Vec3 eye_offset = axis_normal*(0.5*distance) + base1*0.5*distance*std::sqrt(3);

  for (size_t i = 0; i < 12; ++ i)
  {
    osg::Vec3 view_direction = eye_offset*osg::Matrix::rotate(M_PI/6*i, axis_normal);
    osg::Vec3 eye = view_direction + center;
    this->addChild(OSGUtility::drawSphere(eye, 1.0, osg::Vec4(1.0, 0.0, 0.0, 1.0)));
  }

  return;
}