#include <QFile>
#include <QTextStream>

#include <osg/Geode>
#include <osg/Point>
#include <osg/LineWidth>
#include <osg/AnimationPath>

#include <CGAL/linear_least_squares_fitting_3.h>

#include "point_cloud.h"
#include "cgal_types.h"
#include "renderable.h"
#include "cgal_utility.h"
#include "light_source.h"
#include "osg_viewer_widget.h"
#include "povray_visitor.h"
#include "file_system_model.h"
#include "main_window.h"
#include "registrator.h"

PovRayVisitor::PovRayVisitor(OSGViewerWidget* osg_viewer_widget)
  : osg::NodeVisitor(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN),
  osg_viewer_widget_(osg_viewer_widget), frame_number_(0)
{
}

PovRayVisitor::PovRayVisitor(void)
  :osg_viewer_widget_(NULL), frame_number_(0)
{

}
PovRayVisitor::~PovRayVisitor(void)
{
}


void PovRayVisitor::apply( osg::Node& node )
{
  Renderable* renderable = dynamic_cast<Renderable*>(&node);
  if (renderable != NULL) {
    renderable->povray(this);
  }

  traverse( node );

  return;
}

void PovRayVisitor::appendText(QString povray_text)
{
  povray_text_ += povray_text;

  return;
}

const QString& PovRayVisitor::getTextureColorIdentifier(const osg::Vec4& color)
{
  QMutexLocker locker(&mutex_);
  TextureColorMap::iterator it = texture_color_map_.find(color);
  if (it != texture_color_map_.end())
    return it->second;

  QString& identifier = texture_color_map_[color];
  identifier = QString("texture{texture_identifier_%1}").arg(texture_color_map_.size()-1, 3, 10, QChar('0'));
  return identifier;
}

const QString& PovRayVisitor::getNamedFloatIdentifier(const QString& name, double value)
{
  QMutexLocker locker(&mutex_);
  NamedSizeMap::iterator named_size_map_it = named_size_map_.find(name);
  SizeMap& size_map = (named_size_map_it != named_size_map_.end())?
    (named_size_map_it->second):(named_size_map_[name]);

  SizeMap::iterator size_map_it = size_map.find(value);
  if (size_map_it != size_map.end())
    return size_map_it->second;

  QString& identifier = size_map[value];
  identifier = QString("%1_identifier_%2").arg(name).arg(size_map.size()-1, 3, 10, QChar('0'));
  return identifier;
}

void PovRayVisitor::saveLightFile(const QString& light_filename)
{
  QFile light_file(light_filename);
  light_file.open(QIODevice::WriteOnly | QIODevice::Text);
  QTextStream light_file_stream(&light_file);
  // prepare light string
  std::vector<osg::ref_ptr<LightSource> >& light_sources = osg_viewer_widget_->getLightSources();
  for (size_t i = 0, i_end = light_sources.size(); i < i_end; ++ i)
  {
    LightSource* light_source = light_sources[i].get();
    if (light_source->isHidden())
      continue;

    double light_color = 1.2;
    double light_vector = 64;
    int light_resolution = 16;
    light_file_stream << QString("\n//light_source %1\n").arg(i);
    light_file_stream << QString("#declare light_color_%1 = %2;\n").arg(i).arg(light_color);
    light_file_stream << QString("#declare light_vector_%1 = %2;\n").arg(i).arg(light_vector);
    light_file_stream << QString("#declare light_resolution_%1 = %2;\n").arg(i).arg(light_resolution);
    light_file_stream << "light_source {\n";

    osg::Light *light = light_source->getLight();
    const osg::Vec4& position = light->getPosition();
    light_file_stream << QString("    <%1, %2, %3>\n").arg(position[0]).arg(position[1]).arg(position[2]);

    light_file_stream << QString("    color rgb <light_color_%1, light_color_%1, light_color_%1>\n").arg(i);

    osg::BoundingSphere bound = osg_viewer_widget_->getSceneRoot()->getBound();
    CgalPoint point = Caster<osg::Vec4, CgalPoint>(position);
    CgalVector vector(point, Caster<osg::Vec3, CgalPoint>(bound.center()));
    CgalPlane plane(point, vector);
    CgalVector vector_1 = CGALUtility::normalize(plane.base1());
    CgalVector vector_2 = CGALUtility::normalize(plane.base2());
    light_file_stream << QString("    area_light <%1, %2, %3>*light_vector_%4, <%5, %6, %7>*light_vector_%4, light_resolution_%4 light_resolution_%4\n")
      .arg(vector_1.x()).arg(vector_1.y()).arg(vector_1.z()).arg(i)
      .arg(vector_2.x()).arg(vector_2.y()).arg(vector_2.z());

    light_file_stream << "    adaptive 1\n}\n";
  }

  return;
}
void PovRayVisitor::saveTextureFile(const QString& texture_filename)
{
  QFile texture_file(texture_filename);
  texture_file.open(QIODevice::WriteOnly | QIODevice::Text);
  QTextStream texture_file_stream(&texture_file);
  texture_file_stream << QString("#declare ambient_value = 0.5;\n");
  texture_file_stream << QString("#declare diffuse_value = 0.5;\n");
  texture_file_stream << QString("#declare phong_value = 0.5;\n");
  texture_file_stream << QString("background {rgbft<0.0, 0.0, 0.0, 1.0, 1.0>}\n");
  QString finish_string("finish { ambient ambient_value diffuse diffuse_value phong phong_value}");
  for (TextureColorMap::iterator it = texture_color_map_.begin(); it != texture_color_map_.end(); ++ it)
  {
    const osg::Vec4& color = it->first;
    QRegExp identifier("\\{(.*)\\}");
    identifier.indexIn(it->second);
    texture_file_stream << QString("#declare %1 = texture { pigment { rgb <%2, %3, %4> filter %5 } %6};\n")
      .arg(identifier.cap(1)).arg(color.r()).arg(color.g()).arg(color.b()).arg(1-color.a()).arg(finish_string);
  }

  return;
}

void PovRayVisitor::saveTextureFileOrgan(const QString& texture_filename, size_t frame, size_t leaf_num, size_t stem_num)
{
  QFile texture_file(texture_filename);
  texture_file.open(QIODevice::WriteOnly | QIODevice::Text);
  QTextStream texture_file_stream(&texture_file);
  texture_file_stream << QString("background {rgbft<0.0, 0.0, 0.0, 1.0, 1.0>}\n");

  texture_file_stream << "\n";
  texture_file_stream << QString("#declare frame_%1_unknown = global_unknown;\n").arg(frame);
  texture_file_stream << "\n";

  for (size_t i = 0; i < leaf_num; ++ i)
    texture_file_stream << QString("#declare frame_%1_leaf_%2 = global_leaf_%2;\n").arg(frame).arg(i);
  texture_file_stream << "\n";

  for (size_t i = 0; i < stem_num; ++ i)
    texture_file_stream << QString("#declare frame_%1_stem_%2 = global_stem_%2;\n").arg(frame).arg(i);
  texture_file_stream << "\n";

  return;
}


void PovRayVisitor::saveCameraFile(const QString& camera_filename, const osg::AnimationPath* animation_path, int frame_number)
{
  QFile camera_file(camera_filename);
  camera_file.open(QIODevice::WriteOnly | QIODevice::Text);
  QTextStream camera_file_stream(&camera_file);

  osg::Vec3 eye, center, up;
  double fovy, aspect_ratio, z_near, z_far;
  bool animation = (animation_path != NULL);
  double time_gap = animation?(animation_path->getPeriod()/frame_number):0.0;
  for (int i = 1; i <= frame_number; ++ i)
  {
    osg::Matrix matrix;

    if (animation)
    {
      osg::AnimationPath::ControlPoint control_point;
      animation_path->getInterpolatedControlPoint(time_gap*i, control_point);
      control_point.getInverse(matrix);
    }
    else
      matrix = osg_viewer_widget_->getCamera()->getViewMatrix();

    matrix.getLookAt(eye, center, up);
    matrix.getPerspective(fovy, aspect_ratio, z_near, z_far);
    camera_file_stream << QString("%1#if(frame_number=%2)\n").arg(animation?"":"//").arg(i);
    camera_file_stream << QString("\tcamera\n\t{\n\t    perspective\n\t    up -y\n\t    right x*image_width/image_height\n\t %1\t %2\t %3\t}\n")
      .arg(QString("   location <%1, %2, %3>\n").arg(eye[0]).arg(eye[1]).arg(eye[2]))
      .arg(QString("   sky <%1, %2, %3>\n").arg(up[0]).arg(up[1]).arg(up[2]))
      .arg(QString("   look_at <%1, %2, %3>\n").arg(center[0]).arg(center[1]).arg(center[2]));
    camera_file_stream << QString("%1#end\n\n").arg(animation?"":"//");
  }

  return;
}

void PovRayVisitor::initCameraLocation(osg::Vec3& location , osg::Vec3& axis_normal, osg::Vec3& center, osg::Vec3& vector,
                                       size_t end_frame)
{
  FileSystemModel* model = MainWindow::getInstance()->getFileSystemModel();
  osg::ref_ptr<PointCloud> point_cloud = model->getPointCloud(end_frame);
  point_cloud->update();

  Registrator* registrator = MainWindow::getInstance()->getRegistrator();
  axis_normal = registrator->getAxisNormal();
  osg::Vec3 center_pivot = registrator->getPivotPoint();
  double radius = point_cloud->getBound().radius();
  center = point_cloud->getBound().center();

  CgalPoint cgal_center = Caster<osg::Vec3, CgalPoint>(center);
  CgalVector normal = Caster<osg::Vec3, CgalVector>(axis_normal);
  CgalPlane plane = CgalPlane(cgal_center, normal);

  CgalVector base1 = plane.base1();
  base1 = base1/std::sqrt(base1.squared_length());
  normal = normal/std::sqrt(normal.squared_length());
  CgalVector cgal_vector = (base1*1.5*radius + normal*1*radius);
  CgalPoint start_location = cgal_center + cgal_vector;

  /*location = Caster<CgalPoint, osg::Vec3>(start_location);
  vector = Caster<CgalVector, osg::Vec3>(cgal_vector);*/

  center[0] = -14.3204;
  center[1] = 11.0375;
  center[2] = 875.609;         

  location[0] = 127.84;
  location[1] = 144.218;
  location[2] = 695.158;

  /*center = center_pivot;         

  location[0] = -343.991;
  location[1] = -136.566;               
  location[2] = 924.052;*/      

  vector[0] = location[0] - center[0];
  vector[1] = location[1] - center[1];
  vector[2] = location[2] - center[2];
}

void PovRayVisitor::saveCameraFile(const QString& camera_filename, size_t frame_number, size_t camera_number, 
                                   size_t stop_delta, size_t camera_delta, size_t frame_delta, size_t frame_multiple, size_t end_frame)
{
   QFile camera_file(camera_filename);
   camera_file.open(QIODevice::WriteOnly | QIODevice::Text);
   QTextStream camera_file_stream(&camera_file);
   
   double angle = (2*M_PI)/(camera_number*camera_delta); 
   osg::Vec3 location, axis_normal, center, vector;
   initCameraLocation(location,axis_normal,center,vector,end_frame);
 
   /*angle = M_PI/180 * 15;
   for(size_t i = 0; i < frame_number; i ++)
   {
   for(int j = -1; j <= 1; j ++)
   {
   osg::Matrix rotation = osg::Matrix::rotate(-angle*j, axis_normal);
   location = center + rotation.preMult(vector);
   camera_file_stream << QString("#if(frame_number=%1)\n").arg(i*3+j+1);
   camera_file_stream << QString("\tcamera\n\t{\n\t    perspective\n\t    up -y\n\t    right x*image_width/image_height\n\t %1\t %2\t %3\t}\n")
   .arg(QString("   location <%1, %2, %3>\n").arg(location[0]).arg(location[1]).arg(location[2]))
   .arg(QString("   sky <%1, %2, %3>\n").arg(axis_normal[0]).arg(axis_normal[1]).arg(axis_normal[2]))
   .arg(QString("   look_at <%1, %2, %3>\n").arg(center[0]).arg(center[1]).arg(center[2]));
   camera_file_stream << QString("#end\n\n");
   }
   }*/

   for(size_t i = 0; i < frame_number; i++)
   {
     for(size_t j = 0; j < frame_multiple; j++)
     {
       osg::Matrix rotation = osg::Matrix::rotate(angle*(i*frame_multiple+j), axis_normal);
       location = center + rotation.preMult(vector);
       camera_file_stream << QString("#if(frame_number=%1)\n").arg(i*frame_multiple+j);
       camera_file_stream << QString("\tcamera\n\t{\n\t    perspective\n\t    up -y\n\t    right x*image_width/image_height\n\t %1\t %2\t %3\t}\n")
         .arg(QString("   location <%1, %2, %3>\n").arg(location[0]).arg(location[1]).arg(location[2]))
         .arg(QString("   sky <%1, %2, %3>\n").arg(axis_normal[0]).arg(axis_normal[1]).arg(axis_normal[2]))
         .arg(QString("   look_at <%1, %2, %3>\n").arg(center[0]).arg(center[1]).arg(center[2]));
       camera_file_stream << QString("#end\n\n");
     }
   }

   /*for(size_t i = 0; i < frame_number;)
   {
   size_t j = 0;
   while(j < camera_number && i < frame_number)
   {
   size_t k = 0;
   while(k < stop_delta && i < frame_number)
   {
   camera_file_stream << QString("#if(frame_number=%1)\n").arg(i);
   camera_file_stream << QString("\tcamera\n\t{\n\t    perspective\n\t    up -y\n\t    right x*image_width/image_height\n\t %1\t %2\t %3\t}\n")
   .arg(QString("   location <%1, %2, %3>\n").arg(location[0]).arg(location[1]).arg(location[2]))
   .arg(QString("   sky <%1, %2, %3>\n").arg(axis_normal[0]).arg(axis_normal[1]).arg(axis_normal[2]))
   .arg(QString("   look_at <%1, %2, %3>\n").arg(center[0]).arg(center[1]).arg(center[2]));
   camera_file_stream << QString("#end\n\n");

   k++;
   i++;
   }

   k = 0;
   while(k < camera_delta && i < frame_number)
   {
   osg::Matrix rotation = osg::Matrix::rotate(angle*(j*camera_delta+k+1), axis_normal);
   location = center + rotation.preMult(vector);

   camera_file_stream << QString("#if(frame_number=%1)\n").arg(i);
   camera_file_stream << QString("\tcamera\n\t{\n\t    perspective\n\t    up -y\n\t    right x*image_width/image_height\n\t %1\t %2\t %3\t}\n")
   .arg(QString("   location <%1, %2, %3>\n").arg(location[0]).arg(location[1]).arg(location[2]))
   .arg(QString("   sky <%1, %2, %3>\n").arg(axis_normal[0]).arg(axis_normal[1]).arg(axis_normal[2]))
   .arg(QString("   look_at <%1, %2, %3>\n").arg(center[0]).arg(center[1]).arg(center[2]));
   camera_file_stream << QString("#end\n\n");

   k++;
   i++;
   }
   j++;
   }
   }*/

   return;

}


void PovRayVisitor::saveSizeFile(const QString& size_filename)
{
  QFile size_file(size_filename);
  size_file.open(QIODevice::WriteOnly | QIODevice::Text);
  QTextStream size_file_stream(&size_file);
  osg::StateSet* state_Set = osg_viewer_widget_->getSceneRoot()->getOrCreateStateSet();

  for (NamedSizeMap::iterator named_size_map_it = named_size_map_.begin();
    named_size_map_it != named_size_map_.end(); named_size_map_it ++)
  {
    const QString& name = named_size_map_it->first;
    const SizeMap& size_map = named_size_map_it->second;

    double scale = 1.0;
    if (name == QString("point_size"))
    {
      osg::Point* point = dynamic_cast<osg::Point*>(state_Set->getAttribute(osg::StateAttribute::POINT));
      scale = (point != NULL)?(point->getSize()):scale;
    }
    else if (name == QString("line_width"))
    {
      osg::LineWidth* line_width = dynamic_cast<osg::LineWidth*>(state_Set->getAttribute(osg::StateAttribute::LINEWIDTH));
      scale = (line_width != NULL)?(line_width->getWidth()):scale;
    }

    for (SizeMap::const_iterator size_map_it = size_map.begin();
      size_map_it != size_map.end(); ++ size_map_it)
      size_file_stream << QString("#declare %1 = %2;\n").arg(size_map_it->second).arg(size_map_it->first*scale);
  }

  return;
}

void PovRayVisitor::save(const QString& folder)
{
  save(folder, NULL, 1);

  return;
}

void PovRayVisitor::save(const QString& folder, const osg::AnimationPath* animation_path, int frame_number)
{
  QFile data_file(folder+"/data.inc");
  data_file.open(QIODevice::WriteOnly | QIODevice::Text);
  QTextStream data_file_stream(&data_file);
  data_file_stream << povray_text_;

  QString texture_filename(folder+"/texture.inc");
  saveTextureFile(texture_filename);

  QString size_filename(folder+"/size.inc");
  saveSizeFile(size_filename);

  QString camera_filename(folder+"/camera.inc");
  saveCameraFile(camera_filename, animation_path, frame_number);

  QString light_filename(folder+"/light.inc");
  saveLightFile(light_filename);

  QFile povray_file(folder+"/scene.pov");
  povray_file.open(QIODevice::WriteOnly | QIODevice::Text);
  QTextStream povray_file_stream(&povray_file);
  povray_file_stream << "#version 3.7;\n\n";
  povray_file_stream << "global_settings { assumed_gamma 1.0 }\n\n";
  povray_file_stream << "#include \"camera.inc\"\n";
  povray_file_stream << "#include \"texture.inc\"\n";
  povray_file_stream << "#include \"light.inc\"\n";
  povray_file_stream << "#include \"size.inc\"\n";
  povray_file_stream << "#include \"data.inc\"\n";

  QFile ini_file(folder+"/scene.ini");
  ini_file.open(QIODevice::WriteOnly | QIODevice::Text);
  QTextStream ini_file_stream(&ini_file);
  ini_file_stream << "Input_File_Name = \"scene.pov\"\nOutput_File_Name = scene_\n\n";
  ini_file_stream << "Antialias = On\nAntialias_Threshold = 0.3\nAntialias_Depth = 2\n\n";
  ini_file_stream << "Output_Alpha = On\n\n";
  ini_file_stream << QString("Height = %1\nWidth = %2\n\n").arg(osg_viewer_widget_->height()).arg(osg_viewer_widget_->width());
  if(animation_path != NULL)
    ini_file_stream << QString("Initial_Frame = 1\nFinal_Frame= %1\n\n").arg(frame_number);

  return;
}

void PovRayVisitor::drawPolygon(const std::vector<CgalPoint>& polygon, const osg::Vec4& color)
{
  QString polygon_text = QString("polygon{%1, ").arg(polygon.size()+1);
  CgalPlane fitting_plane;
  CGAL::linear_least_squares_fitting_3(polygon.begin(), polygon.end(), fitting_plane, CGAL::Dimension_tag<0>());
  osg::Vec3 fitting_axis(Caster<CgalVector, osg::Vec3>(fitting_plane.orthogonal_vector()));
  osg::Vec3 z_axis(0.0f, 0.0f, 1.0f);
  osg::Vec3 center = Caster<CgalPoint, osg::Vec3>(CGAL::centroid(polygon.begin(), polygon.end()));

  osg::Matrix translation = osg::Matrix::translate(-center);
  osg::Matrix rotation = osg::Matrix::rotate(fitting_axis, z_axis);
  osg::Matrix matrix = translation*rotation;
  for (size_t i = 0, i_end = polygon.size(); i <= i_end; ++ i)
  {
    osg::Vec3 point = Caster<CgalPoint, osg::Vec3>(polygon[(i==i_end)?0:i]);
    osg::Vec3 xy_point = matrix.preMult(point);
    polygon_text += QString("<%1, %2>%3 ").arg(xy_point.x()).arg(xy_point.y()).arg((i == i_end)?' ':',');
  }

  osg::Matrix invert_translation = osg::Matrix::translate(center);
  osg::Matrix invert_rotation = osg::Matrix::rotate(z_axis, fitting_axis);
  osg::Matrix invert_matrix = invert_rotation*invert_translation;
  polygon_text += " matrix<";
  for (size_t i = 0; i < 4; ++ i)
    for (size_t j = 0; j < 3; ++ j)
      polygon_text += QString("%1%2").arg(invert_matrix(i, j)).arg((i==3&&j==2)?'>':',');
  polygon_text += QString(" %1}\n").arg(getTextureColorIdentifier(color));

  povray_text_.append(polygon_text);
}

void PovRayVisitor::drawSphere(const CgalPoint& center, const QString& radius, const osg::Vec4& color)
{
  QString sphere = QString("sphere{<%1, %2, %3>, %4 %5}\n")
    .arg(center.x()).arg(center.y()).arg(center.z()).arg(radius).arg(getTextureColorIdentifier(color));
  povray_text_.append(sphere);
}

void PovRayVisitor::drawSphere(const CgalPoint& center, const QString& radius, int frame, int id, bool is_leaf)
{
  QString sphere;
  if (id == PclRichPoint::ID_UNINITIALIZED)
  {
    sphere = QString("sphere{<%1, %2, %3>, %4 texture{frame_%5_unknown}}\n")
      .arg(center.x()).arg(center.y()).arg(center.z()).arg(radius).arg(frame);
  }
  else
  {
    sphere = QString("sphere{<%1, %2, %3>, %4 texture{frame_%5_%6_%7}}\n")
      .arg(center.x()).arg(center.y()).arg(center.z()).arg(radius).arg(frame).arg(is_leaf?("leaf"):("stem")).arg(id);
  }
  povray_text_.append(sphere);
}

void PovRayVisitor::drawBox(const CgalPoint& center, double width, const osg::Vec4& color)
{
  double offset = std::sqrt(3)*width/2;
  QString box = QString("box{<%1, %2, %3>+<%4, %4, %4>, <%1, %2, %3>-<%4, %4, %4> %5}\n")
    .arg(center.x()).arg(center.y()).arg(center.z())
    .arg(getNamedFloatIdentifier("stable_node_radius", width/2)).arg(getTextureColorIdentifier(color));
  povray_text_.append(box);
}

void PovRayVisitor::drawTriangle(const CgalPoint& p1, const CgalPoint& p2, const CgalPoint& p3, const osg::Vec4& color)
{
  QString triangle = QString("triangle{<%1, %2, %3>, <%4, %5, %6>, <%7, %8, %9> %10}\n")
    .arg(p1.x()).arg(p1.y()).arg(p1.z()).arg(p2.x()).arg(p2.y()).arg(p2.z())
    .arg(p3.x()).arg(p3.y()).arg(p3.z()).arg(getTextureColorIdentifier(color));
  povray_text_.append(triangle);
}

void PovRayVisitor::drawTetrahedron(const CgalPoint& center, double radius, const osg::Vec4& color)
{
  double offset = 2*radius/std::sqrt(3);
  CgalPoint corner_1 = center + CgalVector(-offset, -offset, -offset);
  CgalPoint corner_2 = center + CgalVector(+offset, +offset, -offset);
  CgalPoint corner_3 = center + CgalVector(+offset, -offset, +offset);
  CgalPoint corner_4 = center + CgalVector(-offset, +offset, +offset);

  drawTriangle(corner_1, corner_2, corner_3, color);
  drawTriangle(corner_1, corner_2, corner_4, color);
  drawTriangle(corner_1, corner_4, corner_4, color);
  drawTriangle(corner_2, corner_3, corner_4, color);

  return;
}

void PovRayVisitor::drawCylinder(const CgalPoint& top, const CgalPoint& base, const QString& radius, const osg::Vec4& color)
{
  QString cylinder = QString("cylinder{<%1, %2, %3>, <%4, %5, %6>, %7 %8}\n")
    .arg(top.x()).arg(top.y()).arg(top.z()).arg(base.x()).arg(base.y()).arg(base.z()).arg(radius).arg(getTextureColorIdentifier(color));
  povray_text_.append(cylinder);

  return;
}

QString PovRayVisitor::getPovrayText() const
{
   return povray_text_;
}

void PovRayVisitor::mergeMap(PovRayVisitor& povray_visitor)
{
  TextureColorMap text_color_map = povray_visitor.getTextColorMap();
  texture_color_map_.insert(text_color_map.begin(), text_color_map.end());

  NamedSizeMap named_size_map = povray_visitor.getNameSizeMap();
  named_size_map_.insert(named_size_map.begin(), named_size_map.end());

  return;
}

void PovRayVisitor::setOSGViewerWidget(OSGViewerWidget* osg_viewer_widget)
{
  osg_viewer_widget_ = osg_viewer_widget;

  return;
}

void PovRayVisitor::clearPovrayText()
{
  povray_text_.clear();

  return;
}

OSGViewerWidget* PovRayVisitor::getOSGViewerWidget() const
{
  return osg_viewer_widget_;
}

SlavePovRayVisitor::SlavePovRayVisitor(OSGViewerWidget* osg_viewer_widget):
  PovRayVisitor(osg_viewer_widget)
{

}

SlavePovRayVisitor::SlavePovRayVisitor()
{

}

SlavePovRayVisitor::~SlavePovRayVisitor()
{

}

