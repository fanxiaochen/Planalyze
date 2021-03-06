#include <osg/Geode>
#include <osg/Geometry>

#include <QDomElement>
#include <QDomDocument>

#include <CGAL/Delaunay_triangulation_3.h>
#include <CGAL/Triangulation_vertex_base_with_info_3.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>

#include <boost/graph/adjacency_list.hpp>

#include <pcl/kdtree/kdtree_flann.h>

#include "forward.h"
#include "color_map.h"
#include "main_window.h"
#include "point_cloud.h"
#include "osg_utility.h"
#include "cgal_utility.h"
#include "povray_visitor.h"
#include "file_system_model.h"
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

  if (!skeleton_.empty())
  {
    osg::Vec3 pivot_point(-77.158821, 66.510941, 980.320374);
    osg::Vec3 axis_normal(-0.128121, -0.815076, -0.565009);
    osg::Matrix transformation = osg::Matrix::translate(-pivot_point)*osg::Matrix::rotate(axis_normal, osg::Vec3(0, 0, 1));
    osg::Vec3 front = Caster<CgalPoint, osg::Vec3>(skeleton_.front());
    osg::Vec3 back = Caster<CgalPoint, osg::Vec3>(skeleton_.back());
    if (front.z() > back.z())
      std::reverse(skeleton_.begin(), skeleton_.end());
  }


  return;
}

Organ::Organ(PointCloud* point_cloud, const Organ& organ)
  :point_cloud_(point_cloud)
{
  id_ = organ.id_;
  is_leaf_ = organ.is_leaf_;
  flatness_ = organ.flatness_;
  orientation_ = organ.orientation_;
  skeleton_ = organ.skeleton_;

  return;
}

Organ::~Organ()
{
}

void Organ::save(QDomDocument* doc, QDomElement* element)
{
  if (!is_leaf_ && skeleton_.empty())
    return;

  element->setAttribute("Id", id_);
  element->setAttribute("IsLeaf", (int)(is_leaf_));
  element->setAttribute("flatness", flatness_);
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
    color = ColorMap::Instance().getColor(ColorMap::DISCRETE_KEY, is_leaf_?(2*id_+1):(2*id_));

  return color;
}

void Organ::visualize(void)
{
  osg::Vec4 color = getColor();

  //if (point_cloud_->getFrame() == 501)
  //{
  //  std::vector<size_t> stems;
  //  stems.push_back(9);
  //  stems.push_back(7);
  //  stems.push_back(2);
  //  stems.push_back(3);
  //  stems.push_back(6);

  //  std::vector<size_t> leaves;
  //  leaves.push_back(3);
  //  leaves.push_back(0);
  //  leaves.push_back(2);
  //  leaves.push_back(5);
  //  leaves.push_back(1);

  //  osg::ref_ptr<PointCloud> frame = MainWindow::getInstance()->getFileSystemModel()->getDisplayFirstFrame();
  //  if (frame.valid() && frame->getFrame() != 501)
  //  {
  //    if (is_leaf_)
  //    {
  //      int idx = -1;
  //      int stem_idx = -1;
  //      for (size_t i = 0; i < 5; ++ i)
  //      {
  //        if (id_ == leaves[i])
  //        {
  //          idx = i;
  //          stem_idx = stems[i];
  //          break;
  //        }
  //      }

  //      osg::Matrix transform;

  //      if (idx != -1)
  //      {
  //        std::vector<CgalPoint>& source_skeleton = frame->getStems()[idx].getSkeleton();
  //        osg::Vec3 source_vector(0.0, 0.0, 0.0);
  //        double length_threshold = 8;
  //        int count = 0;
  //        double length = 0;
  //        for (int i = 0, i_end = source_skeleton.size(); i < i_end-1; ++i)
  //        {
  //          source_vector = source_vector + Caster<CgalVector, osg::Vec3>(source_skeleton[i+1]-source_skeleton[i]);
  //          count ++;
  //          length += std::sqrt(CGAL::squared_distance(source_skeleton[i+1], source_skeleton[i]));
  //          if (length > length_threshold)
  //            break;
  //        }
  //        source_vector = source_vector/count;

  //        std::vector<CgalPoint>& target_skeleton = point_cloud_->getStems()[stem_idx].getSkeleton();
  //        osg::Vec3 target_vector(0.0, 0.0, 0.0);
  //        count = 0;
  //        length = 0;
  //        for (int i = 0, i_end = target_skeleton.size(); i < i_end-1; ++i)
  //        {
  //          target_vector = target_vector + Caster<CgalVector, osg::Vec3>(target_skeleton[i+1]-target_skeleton[i]);
  //          count ++;
  //          length += std::sqrt(CGAL::squared_distance(target_skeleton[i+1], target_skeleton[i]));
  //          if (length > length_threshold)
  //            break;
  //        }
  //        target_vector = target_vector/count;

  //        //{
  //          osg::Vec3 ssource_vector(0.0, 0.0, 0.0);
  //          count = 0;
  //          length = 0;
  //          for (int i = source_skeleton.size()-1; i > 0; i--)
  //          {
  //            ssource_vector = ssource_vector + Caster<CgalVector, osg::Vec3>(source_skeleton[i]-source_skeleton[i-1]);
  //            count ++;
  //            length += std::sqrt(CGAL::squared_distance(source_skeleton[i], source_skeleton[i-1]));
  //            if (length > length_threshold)
  //              break;
  //          }
  //          source_vector = source_vector/count;

  //          osg::Vec3 ttarget_vector(0.0, 0.0, 0.0);
  //          count = 0;
  //          length = 0;
  //          for (int i = 0, i_end = target_skeleton.size(); i < i_end-1; ++i)
  //          {
  //            ttarget_vector = ttarget_vector + Caster<CgalVector, osg::Vec3>(target_skeleton[i+1]-target_skeleton[i]);
  //            count ++;
  //            length += std::sqrt(CGAL::squared_distance(target_skeleton[i+1], target_skeleton[i]));
  //            if (length > length_threshold)
  //              break;
  //          }
  //          ttarget_vector = ttarget_vector/count;
  //        //}

  //        osg::Vec3 offset = Caster<CgalVector, osg::Vec3>(source_skeleton.front()-source_skeleton.back());
  //        offset = osg::Matrix::rotate(ssource_vector, ttarget_vector).preMult(offset);
  //        osg::Vec3 end_point = Caster<CgalPoint, osg::Vec3>(target_skeleton.front());
  //        transform = osg::Matrix::translate(-end_point)*osg::Matrix::rotate(target_vector, source_vector)*osg::Matrix::translate(end_point)*osg::Matrix::translate(offset);
  //      }

  //      osg::ref_ptr<osg::Vec3Array>  vertices = new osg::Vec3Array;
  //      osg::ref_ptr<osg::Vec3Array>  normals = new osg::Vec3Array;
  //      osg::ref_ptr<osg::Vec4Array>  colors = new osg::Vec4Array;

  //      for (size_t i = 0, i_end = point_indices_.size(); i < i_end; ++ i)
  //      {
  //        const PclRichPoint& point = point_cloud_->at(point_indices_[i]);

  //        vertices->push_back(osg::Vec3(point.x, point.y, point.z));

  //        if (idx != -1)
  //        {
  //          vertices->back() = transform.preMult(vertices->back());
  //        }

  //        normals->push_back(osg::Vec3(point.normal_x, point.normal_y, point.normal_z));
  //        colors->push_back(osg::Vec4(point.r/255.0, point.g/255.0, point.b/255.0, 1.0));
  //      }

  //      osg::Geode* geode = new osg::Geode;
  //      osg::Geometry* geometry = new osg::Geometry;
  //      geometry->setUseDisplayList(true);
  //      geometry->setUseVertexBufferObjects(true);
  //      geometry->setVertexData(osg::Geometry::ArrayData(vertices, osg::Geometry::BIND_PER_VERTEX));
  //      geometry->setNormalData(osg::Geometry::ArrayData(normals, osg::Geometry::BIND_PER_VERTEX));
  //      geometry->setColorData(osg::Geometry::ArrayData(colors, osg::Geometry::BIND_PER_VERTEX));
  //      geometry->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::POINTS, 0, vertices->size()));
  //      geode->addDrawable(geometry);
  //      point_cloud_->addChild(geode);
  //    }
  //    else
  //    {
  //      int idx = -1;
  //      for (size_t i = 0; i < 5; ++ i)
  //      {
  //        if (id_ == stems[i])
  //        {
  //          idx = i;
  //          break;
  //        }
  //      }
  //      if (idx != -1)
  //      {
  //        std::vector<CgalPoint>& skeleton = frame->getStems()[idx].getSkeleton();
  //        osg::Vec3 source_vector(0.0, 0.0, 0.0);
  //        double length_threshold = 8;
  //        int count = 0;
  //        double length = 0;
  //        for (int i = skeleton.size()-1; i > 0; i--)
  //        {
  //          source_vector = source_vector + Caster<CgalVector, osg::Vec3>(skeleton[i]-skeleton[i-1]);
  //          count ++;
  //          length += std::sqrt(CGAL::squared_distance(skeleton[i], skeleton[i-1]));
  //          if (length > length_threshold)
  //            break;
  //        }
  //        source_vector = source_vector/count;

  //        osg::Vec3 target_vector(0.0, 0.0, 0.0);
  //        count = 0;
  //        length = 0;
  //        for (int i = 0, i_end = skeleton_.size(); i < i_end-1; ++i)
  //        {
  //          target_vector = target_vector + Caster<CgalVector, osg::Vec3>(skeleton_[i+1]-skeleton_[i]);
  //          count ++;
  //          length += std::sqrt(CGAL::squared_distance(skeleton_[i+1], skeleton_[i]));
  //          if (length > length_threshold)
  //            break;
  //        }
  //        target_vector = target_vector/count;

  //        osg::Vec3 offset1 = Caster<CgalPoint, osg::Vec3>(skeleton.back());
  //        osg::Vec3 offset2 = Caster<CgalPoint, osg::Vec3>(skeleton_.front());
  //        osg::Matrix transform = osg::Matrix::translate(-offset1)*osg::Matrix::rotate(source_vector, target_vector)*osg::Matrix::translate(offset2);

  //        for (size_t i = 0, i_end = skeleton.size()-1; i < i_end; ++ i)
  //        {
  //          osg::Vec3 source_center = Caster<CgalPoint, osg::Vec3>(skeleton[i]);
  //          osg::Vec3 target_center = Caster<CgalPoint, osg::Vec3>(skeleton[i+1]);
  //          source_center = transform.preMult(source_center);
  //          target_center = transform.preMult(target_center);
  //          point_cloud_->addChild(OSGUtility::drawCylinder(source_center, target_center, 3, color));
  //        }
  //      }
  //    }
  //  }
  //}

  if (skeleton_.empty())
    return;

  if (is_leaf_)
    return;

  for (size_t i = 0, i_end = skeleton_.size()-1; i < i_end; ++ i)
  {
    osg::Vec3 source_center = Caster<CgalPoint, osg::Vec3>(skeleton_[i]);
    osg::Vec3 target_center = Caster<CgalPoint, osg::Vec3>(skeleton_[i+1]);
    point_cloud_->addChild(OSGUtility::drawCylinder(source_center, target_center, 3, color));
  }

  //osg::Vec3 pivot_point(-77.158821, 66.510941, 980.320374);
  //osg::Vec3 axis_normal(-0.128121, -0.815076, -0.565009);
  //osg::Matrix transformation = osg::Matrix::translate(-pivot_point)*osg::Matrix::rotate(axis_normal, osg::Vec3(0, 0, 1));

  //for (size_t i = 0, i_end = skeleton_.size()-1; i < i_end; ++ i)
  //{
  //  osg::Vec3 source_center = Caster<CgalPoint, osg::Vec3>(skeleton_[i]);
  //  osg::Vec3 target_center = Caster<CgalPoint, osg::Vec3>(skeleton_[i+1]);
  //  source_center = transformation.preMult(source_center);
  //  target_center = transformation.preMult(target_center);
  //  point_cloud_->addChild(OSGUtility::drawCylinder(source_center, target_center, 0.5, color));
  //}

  //osg::Vec3 root(0, 0, std::numeric_limits<float>::max());
  //CgalPoint real_root;
  //for (size_t j = 0, j_end = skeleton_.size(); j < j_end; ++ j)
  //{
  //  osg::Vec3 point = Caster<CgalPoint, osg::Vec3>(skeleton_[j]);
  //  point = transformation.preMult(point);

  //  if (root.z() > point.z())
  //  {
  //    root = point;
  //    real_root = skeleton_[j];
  //  }
  //}

  //point_cloud_->addChild(OSGUtility::drawSphere(Caster<CgalPoint, osg::Vec3>(real_root), 1.0, color));

  return;
}

void Organ::visualize(PovRayVisitor* povray_visitor)
{
  if (!is_leaf_ && skeleton_.empty())
    return;

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

      std::vector<int> indices(1);
      std::vector<float> distances(1);
      int neighbor_num = kdtree->nearestKSearch(point, 1, indices, distances);

      double search_radius = search_radius_threshold;
      if (distances[0] > search_radius*search_radius)
        search_radius = 1.1*std::sqrt(distances[0]);

      neighbor_num = kdtree->radiusSearch(point, search_radius, indices, distances);

      osg::Vec3 center(0.0f, 0.0f, 0.0f);
      double total_weight = 0.0;
      for (size_t j = 0; j < neighbor_num; ++ j)
      {
        osg::Vec3 offset = point_cloud_->at(indices[j]).cast<osg::Vec3>();
        double weight = std::exp(-distances[j]/std::pow(search_radius, 2.0));
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
    if (point_cloud_.get() != NULL)
    {
      std::cout << "frame: " << point_cloud_->getFrame()
        << "\tupdateFeature:: empty organ:\t" << id_ << "!"
        << std::endl;
    }

    return;
  }

  updateFlatnessFeature();
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

double Organ::computeSkeletonToPointDistance(const CgalPoint& point)
{
  if (skeleton_.size() == 0)
  {
    return 100000000;
  }
  double min_distance = std::numeric_limits<double>::max();
  for (size_t i = 0, i_end = skeleton_.size()-1; i < i_end; ++ i)
  {
    CgalSegment segment(skeleton_[i], skeleton_[i+1]);
    double distance = CGAL::squared_distance(point, segment);
    min_distance = std::min(min_distance, distance);
  }

  return std::sqrt(min_distance);
}

double Organ::computeArea(void)
{
  double volume = 0;
  if (!is_leaf_)
  {
    volume = std::sqrt(CGAL::squared_distance(skeleton_.front(), skeleton_.back()));
  }

  return volume;
}