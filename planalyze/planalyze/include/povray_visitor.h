#pragma once
#ifndef POVRAY_VISITOR_H
#define POVRAY_VISITOR_H

#include <map>
#include <QString>
#include <QMutex>
#include <osg/NodeVisitor>

#include "cgal_types.h"

namespace osg
{
  class AnimationPath;
}

class OSGViewerWidget;

class PovRayVisitor : public osg::NodeVisitor
{
public:
  PovRayVisitor();
  PovRayVisitor(OSGViewerWidget* osg_viewer_widget);
  ~PovRayVisitor(void);

  virtual void apply(osg::Node& node);

  void appendText(QString povray_text);
  virtual const QString& getTextureColorIdentifier(const osg::Vec4& color);
  virtual const QString& getNamedFloatIdentifier(const QString& name, double value);
  void save(const QString& folder);
  void save(const QString& folder, const osg::AnimationPath* animation_path, int frame_number);

  void saveLightFile(const QString& light_filename);
  void saveTextureFile(const QString& texture_filename);
  void saveTextureFileOrgan(const QString& texture_filename, size_t frame, size_t leaf_num, size_t stem_num);
  void saveCameraFile(const QString& camera_filename, const osg::AnimationPath* animation_path, int frame_number);
  void saveCameraFile(const QString& camera_filename, size_t frame_number, size_t camera_number, 
    size_t stop_delta, size_t camera_delta, size_t frame_delta, size_t frame_multiple, size_t end_frame);
  void initCameraLocation(osg::Vec3& location, osg::Vec3& axis_normal, osg::Vec3& center, osg::Vec3& vector,
    size_t end_frame);
  void saveSizeFile(const QString& size_filename);

  void drawPolygon(const std::vector<CgalPoint>& polygon, const osg::Vec4& color);
  void drawSphere(const CgalPoint& center, const QString& radius, const osg::Vec4& color);
  void drawSphere(const CgalPoint& center, const QString& radius, int frame, int id, bool is_leaf);
  void drawBox(const CgalPoint& center, double width, const osg::Vec4& color);
  void drawTetrahedron(const CgalPoint& center, double radius, const osg::Vec4& color);
  void drawTriangle(const CgalPoint& p1, const CgalPoint& p2, const CgalPoint& p3, const osg::Vec4& color);
  void drawCylinder(const CgalPoint& top, const CgalPoint& base, const QString& thickness, const osg::Vec4& color);

  QString getPovrayText(void) const;
  void clearPovrayText(void);
  void mergeMap(PovRayVisitor& povray_visitor);
  void setOSGViewerWidget(OSGViewerWidget* osg_viewer_widget);
  OSGViewerWidget* getOSGViewerWidget() const;
  size_t& getFrameNumber(void){return frame_number_;}

private:
  OSGViewerWidget*              osg_viewer_widget_;
  QString                       povray_text_;

  typedef std::map<osg::Vec4, QString>  TextureColorMap;
  TextureColorMap                       texture_color_map_;

  typedef std::map<double, QString>    SizeMap;
  typedef std::map<QString, SizeMap>   NamedSizeMap;
  NamedSizeMap                         named_size_map_;

  QMutex                               mutex_;
  
  size_t                               frame_number_;

  TextureColorMap& getTextColorMap(void){return texture_color_map_;}
  NamedSizeMap& getNameSizeMap(void){return named_size_map_;}
};

class SlavePovRayVisitor : public PovRayVisitor
{
public:
  SlavePovRayVisitor();
  SlavePovRayVisitor(OSGViewerWidget* osg_viewer_widget);
  ~SlavePovRayVisitor();

  PovRayVisitor* getMaster(void){return master_;};
  void setMaster(PovRayVisitor* master){master_ = master;};

private:
  PovRayVisitor* master_;
  

};

#endif // POVRAY_VISITOR_H