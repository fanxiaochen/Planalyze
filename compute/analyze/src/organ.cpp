#include <osg/Geode>
#include <osg/Geometry>

#include <QDomElement>
#include <QDomDocument>

#include <boost/graph/adjacency_list.hpp>

#include <pcl/kdtree/kdtree_flann.h>

#include "forward.h"
#include "color_map.h"
#include "main_window.h"
#include "point_cloud.h"
#include "osg_utility.h"
#include "cgal_utility.h"
#include "povray_visitor.h"
#include "parameter_manager.h"

#include "organ.h"

Organ::Organ(void)
  :point_cloud_(NULL),
  id_(0),
  is_leaf_(false)
{
}

Organ::Organ(PointCloud* point_cloud, size_t id, bool is_leaf)
  :point_cloud_(point_cloud),
  id_(id),
  is_leaf_(is_leaf)
{
}

Organ::Organ(PointCloud* point_cloud, QDomElement* element)
  :point_cloud_(point_cloud)
{
  id_ = element->attribute("Id", "").toUInt();
  is_leaf_ = bool(element->attribute("IsLeaf", "").toInt());
  flatness_ = element->attribute("flatness", "").toDouble();
  thickness_ = element->attribute("thickness", "").toDouble();
  double ox = element->attribute("OrientationX", "").toDouble();
  double oy = element->attribute("OrientationY", "").toDouble();
  double oz = element->attribute("OrientationZ", "").toDouble();

  QDomElement skeleton_element = element->firstChildElement("Skeleton");
  QDomElement point_element = skeleton_element.firstChildElement("Point");
  while (!point_element.isNull()) {
    double x = point_element.attribute("X", "").toDouble();
    double y = point_element.attribute("Y", "").toDouble();
    double z = point_element.attribute("Z", "").toDouble();
    skeleton_.push_back(CgalPoint(x, y, z));
    point_element = point_element.nextSiblingElement("Point");
  }

  return;
}

Organ::Organ(PointCloud* point_cloud, const Organ& organ)
  :point_cloud_(point_cloud)
{
  id_ = organ.id_;
  is_leaf_ = organ.is_leaf_;
  flatness_ = organ.flatness_;
  thickness_ = organ.thickness_;
  orientation_ = organ.orientation_;
  skeleton_ = organ.skeleton_;

  return;
}

Organ::~Organ()
{
}

void Organ::save(QDomDocument* doc, QDomElement* element)
{
  element->setAttribute("Id", id_);
  element->setAttribute("IsLeaf", (int)(is_leaf_));
  element->setAttribute("flatness", flatness_);
  element->setAttribute("thickness", thickness_);
  element->setAttribute("OrientationX", orientation_.x());
  element->setAttribute("OrientationY", orientation_.y());
  element->setAttribute("OrientationZ", orientation_.z());

  QDomElement skeleton_element = doc->createElement("Skeleton");
  for (size_t i = 0, i_end = skeleton_.size(); i < i_end; ++ i)
  {
    QDomElement point_element = doc->createElement("Point");
    const CgalPoint& point = skeleton_[i];
    point_element.setAttribute("X", (double)point.x());
    point_element.setAttribute("Y", (double)point.y());
    point_element.setAttribute("Z", (double)point.z());
    skeleton_element.appendChild(point_element);
  }
  element->appendChild(skeleton_element);

  return;
}

osg::Vec4 Organ::getColor(void) const
{
  osg::Vec4 color;
  if (point_indices_.empty())
    color = ColorMap::Instance().getColor(ColorMap::LIGHT_BLUE);
  else
  {
    int mod = ParameterManager::getInstance().getColorizeMod();
    std::srand(id_+mod);
    size_t color_idx = std::abs(std::rand())%mod;
    color = ColorMap::Instance().getColor(ColorMap::JET, color_idx, 0, mod-1);
  }

  return color;
}

void Organ::visualize(void)
{
  if (skeleton_.empty())
    return;

  if (is_leaf_)
    return;

  osg::Vec4 color = getColor();

  for (size_t i = 0, i_end = skeleton_.size()-1; i < i_end; ++ i)
  {
    osg::Vec3 source_center = Caster<CgalPoint, osg::Vec3>(skeleton_[i]);
    osg::Vec3 target_center = Caster<CgalPoint, osg::Vec3>(skeleton_[i+1]);
    point_cloud_->addChild(OSGUtility::drawCylinder(source_center, target_center, 0.5, color));
  }

  return;
}

void Organ::visualize(PovRayVisitor* povray_visitor)
{
  osg::Vec4 color = getColor();

  const QString& edge_radius = povray_visitor->getNamedFloatIdentifier("primitive_radius", 0.5);
  for (size_t i = 0, i_end = skeleton_.size()-1; i < i_end; ++ i)
    povray_visitor->drawCylinder(skeleton_[i], skeleton_[i+1], edge_radius, color);

  return;
}


void Organ::addPoint(size_t index)
{
  point_cloud_->at(index).organ_id = id_;
  point_indices_.push_back(index);

  return;
}

std::vector<CgalPoint> Organ::getPoints(void) const
{
  std::vector<CgalPoint> points;
  for (size_t i = 0, i_end = point_indices_.size(); i < i_end; ++ i)
    points.push_back(point_cloud_->at(point_indices_[i]).cast<CgalPoint>());

  return points;
}

void Organ::updateFlatnessFeature(void)
{
  flatness_ = 0;
  for (size_t i = 0, i_end = point_indices_.size(); i < i_end; ++ i)
  {
    double curvature = point_cloud_->at(point_indices_[i]).curvature;
    flatness_ += point_cloud_->transformCurvature(curvature);
  }

  flatness_ /= point_indices_.size();

  return;
}

void Organ::updateThicknessFeature(void)
{
  thickness_ = 0;
  for (size_t i = 0, i_end = point_indices_.size(); i < i_end; ++ i)
    thickness_ += point_cloud_->at(point_indices_[i]).thickness;

  thickness_ /= point_indices_.size();

  return;
}

void Organ::updateOrientationFeature(void)
{
  double bone_length = 2.0;
  double search_radius_threshold = ParameterManager::getInstance().getStemSkeletonRadius();

  orientation_ = CgalVector(0.0, 0.0, 0.0);
  for (size_t i = 0, i_end = point_indices_.size(); i < i_end; ++ i)
  {
    PclRichPoint& point = point_cloud_->at(point_indices_[i]);
    orientation_ = orientation_ + CgalVector(point.orientation_x, point.orientation_y, point.orientation_z);
  }
  orientation_ = orientation_/point_indices_.size();
  orientation_ = CGALUtility::normalize(orientation_);

  CgalSegment segment;
  CGALUtility::computeSegment(getPoints(), segment);


  double segment_length = std::sqrt(segment.squared_length());

  skeleton_.clear();
  if (segment_length < 2*bone_length)
  {
    skeleton_.push_back(segment.source());
    skeleton_.push_back(segment.target());
    return;
  }

  size_t bone_num = std::floor(segment_length/bone_length);
  bone_length = segment_length/(double)(bone_num);
  CgalVector delta = CGALUtility::normalize(CgalVector(segment.source(), segment.target()))*bone_length;

  PointCloud::KdTreePtr kdtree = is_leaf_?point_cloud_->getLeafKdTree(id_):point_cloud_->getStemKdTree(id_);

  for (size_t i = 0; i <= bone_num; ++ i)
    skeleton_.push_back(segment.source()+delta*i);

  for (size_t round = 0; round < 5; ++ round)
  {
    for (size_t i = 0; i <= bone_num; ++ i)
    {
      PclRichPoint point(skeleton_[i]);

      std::vector<int> neighbor_indices(1);
      std::vector<float> neighbor_squared_distances(1);
      int neighbor_num = kdtree->nearestKSearch(point, 1, neighbor_indices, neighbor_squared_distances);

      double search_radius = search_radius_threshold;
      if (neighbor_squared_distances[0] > search_radius*search_radius)
        search_radius = 1.1*std::sqrt(neighbor_squared_distances[0]);

      neighbor_num = kdtree->radiusSearch(point, search_radius, neighbor_indices, neighbor_squared_distances);

      osg::Vec3 center(0.0f, 0.0f, 0.0f);
      double total_weight = 0.0;
      for (size_t j = 0; j < neighbor_num; ++ j)
      {
        osg::Vec3 offset = point_cloud_->at(neighbor_indices[j]).cast<osg::Vec3>();
        double weight = std::exp(-neighbor_squared_distances[j]/std::pow(search_radius, 2.0));
        center = center + offset*weight;
        total_weight += weight;
      }
      if (total_weight == 0.0)
        center = point.cast<osg::Vec3>();
      else
        center = center/total_weight;

      CgalPlane plane(skeleton_[i], segment.to_vector());
      skeleton_[i] = (Caster<osg::Vec3, CgalPoint>(center));
      skeleton_[i] = plane.projection(skeleton_[i]);
    }
  }

  length_ = 0;
  for (size_t i = 0, i_end = skeleton_.size()-1; i < i_end; ++ i)
    length_ += std::sqrt(CGAL::squared_distance(skeleton_[i], skeleton_[i+1]));

  return;
}

void Organ::updateFeature(void)
{
  if (point_indices_.empty())
  {
    std::cout << "frame: " << point_cloud_->getFrame()
      << "\tupdateFeature:: empty organ:\t" << id_ << "!"
      << std::endl;
    return;
  }

  updateFlatnessFeature();
  updateThicknessFeature();
  updateOrientationFeature();

  return;
}


void Organ::setId(size_t id)
{
  for (size_t j = 0, j_end = point_indices_.size(); j < j_end; ++ j)
    point_cloud_->at(point_indices_[j]).organ_id = id;

  id_ = id;
  return;
}

double Organ::distance(const CgalPoint& point)
{
  double min_distance = std::numeric_limits<double>::max();
  for (size_t i = 0, i_end = skeleton_.size()-1; i < i_end; ++ i)
  {
    CgalSegment segment(skeleton_[i], skeleton_[i+1]);
    double distance = CGAL::squared_distance(point, segment);
    min_distance = std::min(min_distance, distance);
  }

  return std::sqrt(min_distance);
}

double Organ::distance(const Organ& organ)
{
  if (skeleton_.empty() || organ.skeleton_.empty())
  {
    std::cout << "fuck organ organ distance" << std::endl;
    return 0;
  }

  CgalPoint top = skeleton_[0];
  for (size_t i = 0, i_end = skeleton_.size(); i < i_end; ++ i)
  {
    if (top.y() > skeleton_[i].y())
    {
      top = skeleton_[i];
    }
  }

  double min_distance = std::numeric_limits<double>::max();
  for (size_t i = 0, i_end = organ.skeleton_.size(); i < i_end; ++ i)
  {
    double distance = CGAL::squared_distance(top, organ.skeleton_[i]);
    min_distance = std::min(min_distance, distance);
  }

  return std::sqrt(min_distance);
}