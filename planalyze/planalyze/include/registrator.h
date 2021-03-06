#pragma once
#ifndef REGISTRATOR_H
#define REGISTRATOR_H

#include <QObject>
#include "pcl_wrapper_types.h"
#include "renderable.h"

namespace osgManipulator
{
  class TranslateAxisDragger;
  class TrackballDragger;
}

class Registrator : public QObject, public Renderable
{
  Q_OBJECT
public:
  Registrator(void);
  virtual ~Registrator(void);

  virtual const char* className() const {return "Registrator";}

  void init(void);
  bool load(void);
  void reset(void);

  osg::Vec3 getPivotPoint() const;
  osg::Vec3 getAxisNormal() const;
  osg::Matrix getRotationMatrix(double angle) const;

  void setPivotPoint(const osg::Vec3& pivot_point);
  void setAxisNormal(const osg::Vec3& axis_normal);

  virtual void toggleRendering(void);

  void saveRegisteredPoints(int frame);
  void refineAxis(int frame);
  void registrationLUM(int segment_threshold, int max_iterations, double max_distance, int frame);
  void registrationICP(int max_iterations, double max_distance, int frame, bool isUsingPot, int times);
  void registrationICP(int max_iterations, double max_distance, int frame, bool isUsingPot);

public slots:
  void save(void);
  void saveRegisteredPoints(void);
  void refineAxis(void);
  void registrationICP(void);
  void registrationLUM(void);

protected:
  virtual void clear();
  virtual void updateImpl();
  void computeError(int frame);
  void visualizeError(void);
  void visualizeAxis(void);
  void save(const QString& filename);
  bool load(const QString& filename);

protected:
  osg::ref_ptr<osg::MatrixTransform>                  pivot_point_;
  osg::ref_ptr<osg::MatrixTransform>                  normal_point_;
  osg::ref_ptr<osg::MatrixTransform>                  visualization_;
  osg::ref_ptr<osgManipulator::TranslateAxisDragger>  pivot_dragger_;
  osg::ref_ptr<osgManipulator::TrackballDragger>      normal_dragger_;

protected:
  osg::ref_ptr<osg::Vec3Array>  error_vertices_;
  osg::ref_ptr<osg::Vec4Array>  error_colors_;

private:
  bool              initilized_;
  bool              show_axis_;
  bool              show_error_;
};

#endif // REGISTRATOR_H