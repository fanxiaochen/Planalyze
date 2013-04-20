#pragma once
#ifndef ORGAN_H
#define ORGAN_H

#include <osg/Vec4>
#include <osg/ref_ptr>

#include "cgal_types.h"

class PointCloud;
class QDomElement;
class QDomDocument;
class PovRayVisitor;

class Organ
{
public:
  Organ(void);
  Organ(PointCloud* point_cloud, const Organ& organ);
  Organ(PointCloud* point_cloud, QDomElement* element);
  Organ(PointCloud* point_cloud, size_t organ_id, bool is_leaf);
  virtual ~Organ();
  void save(QDomDocument* doc, QDomElement* element);

  size_t getId(void) const {return id_;}
  void setId(size_t id);
  const std::vector<int>& getPointIndices(void) const {return point_indices_;}
  double getLength(void) const {return length_;}

  void addPoint(size_t index);

  void updateFeature(void);

  void visualize(void);
  void visualize(PovRayVisitor* povray_visitor);

  bool isLeaf(void) const {return is_leaf_;}
  bool isStem(void) const {return !is_leaf_;}

  double getFlatness(void) const {return flatness_;}
  void setFlatness(double flatness) {flatness_ = flatness;}

  double getThickness(void) const {return thickness_;}
  void setThickness(double thickness) {thickness_ = thickness;}

  CgalVector getOrientation(void) const {return orientation_;}
  void setOrientation(CgalVector orientation) {orientation_ = orientation;}
  std::vector<CgalPoint>& getSkeleton(void) {return skeleton_;}
  double distance(const CgalPoint& point);
  double distance(const Organ& organ);

  double computeArea(void);

private:
  osg::Vec4 getColor(void) const;
  std::vector<CgalPoint> getPoints(void) const;

  void updateFlatnessFeature(void);
  void updateThicknessFeature(void);
  void updateOrientationFeature(void);

private:
  osg::ref_ptr<PointCloud>    point_cloud_;
  std::vector<int>            point_indices_;
  size_t                      id_;
  bool                        is_leaf_;

  // for leaf
  double                      flatness_;
  double                      thickness_;

  // for stem
  CgalVector                  orientation_;
  std::vector<CgalPoint>      skeleton_;
  double                      length_;
};

struct CompareOrganBySize
{
  bool operator()(const Organ& a, const Organ& b)
  {
    return a.getPointIndices().size() > b.getPointIndices().size();
  }
};


#endif // ORGAN_H