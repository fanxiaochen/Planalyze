#include <fstream>

#include <QRegExp>
#include <QFileInfo>
#include <QFileDialog>
#include <QColorDialog>
#include <QFutureWatcher>
#include <QtConcurrentRun>

#include <osg/Geode>
#include <osg/io_utils>
#include <osg/Geometry>
#include <osgManipulator/TrackballDragger>
#include <osgManipulator/TranslateAxisDragger>

#include <CGAL/Delaunay_triangulation_3.h>
#include <CGAL/Triangulation_vertex_base_with_info_3.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/connected_components.hpp>

#include <pcl/Vertices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/common/pca.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree_flann.h>

#include "organ.h"
#include "color_map.h"
#include "parameter.h"
#include "cgal_types.h"
#include "registrator.h"
#include "main_window.h"
#include "osg_utility.h"
#include "cgal_utility.h"
#include "povray_visitor.h"
#include "parameter_dialog.h"
#include "file_system_model.h"
#include "parameter_manager.h"
#include "point_cloud.h"
#include "osg_viewer_widget.h"


PointCloud::PointCloud(void)
  :color_mode_((ColorMode)(MainWindow::getInstance()->getColorMode()->currentIndex())),
  color_(MainWindow::getInstance()->getFileSystemModel()->getColor()),
  translate_dragger_(new osgManipulator::TranslateAxisDragger),
  trackball_dragger_(new osgManipulator::TrackballDragger),
  show_draggers_(false),
  registered_(false),
  show_plant_(MainWindow::getInstance()->getUI().actionRenderPlant->isChecked()),
  show_pot_(MainWindow::getInstance()->getUI().actionRenderPot->isChecked()),
  show_noise_(MainWindow::getInstance()->getUI().actionRenderNoise->isChecked()),
  show_normals_(MainWindow::getInstance()->getUI().actionRenderNormals->isChecked()),
  show_orientations_(MainWindow::getInstance()->getUI().actionRenderOrientations->isChecked()),
  show_leaves_(MainWindow::getInstance()->getUI().actionRenderLeaves->isChecked()),
  show_stems_(MainWindow::getInstance()->getUI().actionRenderStems->isChecked()),
  show_triangles_(MainWindow::getInstance()->getUI().actionRenderTriangles->isChecked()),
  show_organs_(MainWindow::getInstance()->getUI().actionRenderOrgans->isChecked()),
  show_stem_graph_(MainWindow::getInstance()->getUI().actionRenderStemGraph->isChecked()),
  triangulation_(new CGAL::Delaunay()),
  point_graph_(new boost::PointGraph()),
  stem_skeleton_graph_(new boost::SkeletonGraph()),
  point_graph_threshold_(-1.0),
  kdtree_(new pcl::KdTreeFLANN<PclRichPoint>()),
  plant_points_num_(0),
  pot_points_num_(0),
  noise_points_num_(0)
{
  translate_dragger_->setupDefaultGeometry();
  translate_dragger_->setHandleEvents(true);
  translate_dragger_->setActivationModKeyMask(osgGA::GUIEventAdapter::MODKEY_CTRL);
  translate_dragger_->setActivationKeyEvent('d');

  trackball_dragger_->setupDefaultGeometry();
  trackball_dragger_->setHandleEvents(true);
  trackball_dragger_->setActivationModKeyMask(osgGA::GUIEventAdapter::MODKEY_CTRL);
  trackball_dragger_->setActivationKeyEvent('d');

  translate_dragger_->addTransformUpdating(this);
  translate_dragger_->addTransformUpdating(trackball_dragger_);

  trackball_dragger_->addTransformUpdating(this);
  trackball_dragger_->addTransformUpdating(translate_dragger_);
}

PointCloud::~PointCloud(void)
{
  saveStatus();

  delete triangulation_;
  delete point_graph_;
  delete stem_skeleton_graph_;
}

void PointCloud::setRegisterState(bool registered)
{
  registered_=registered;
  if (registered_ && getView() != 0)
    saveTransformation();

  return;
}

bool PointCloud::open(const std::string& filename)
{
  clearData();

  QMutexLocker locker(&mutex_);

  if (pcl::io::loadPCDFile(filename, *this) != 0)
    return false;

  kdtree_->setInputCloud(boost::shared_ptr<const PclRichPointCloud>(this, NullDeleter()));

  filename_ = filename;
  loadTransformation();

  registered_ = (getView() == 0) || (!(getMatrix().isIdentity()));

  if (!this->empty() && this->at(0).label != PclRichPoint::LABEL_UNINITIALIZED)
  {
    plant_points_num_ = 0;
    pot_points_num_ = 0;
    noise_points_num_ = 0;
    for (size_t i = 0, i_end = size(); i < i_end; ++ i)
    {
      uint8_t label = at(i).label;
      if (label >= PclRichPoint::LABEL_PLANT)
        plant_points_num_ ++;
      else if (label == PclRichPoint::LABEL_POT)
        pot_points_num_ ++;
      else if (label == PclRichPoint::LABEL_NOISE)
        noise_points_num_ ++;
    }
  }

  if (getView() == 12 && plant_points_num_ == 0)
  {
    for (size_t i = 0; i < plant_points_num_; ++ i)
      at(i).label = PclRichPoint::LABEL_PLANT;
    plant_points_num_ = size();
    pot_points_num_ = 0;
    noise_points_num_ = 0;
  }

  for (size_t i = 0; i < plant_points_num_; ++ i)
    at(i).segment_id = PclRichPoint::ID_UNINITIALIZED;

  initMinMaxCurvature();
  initMinMaxThickness();

  locker.unlock();
  loadStatus();
  locker.relock();
  fillOrganPoints();
  updateStatistics();

  expire();

  return true;
}

bool PointCloud::save(const std::string& filename)
{
  if (QString(filename.c_str()).right(3) == "ply")
  {
    PclPointCloud point_cloud;
    osg::Vec3 pivot_point(-13.382786, 50.223461, 917.477600);
    osg::Vec3 axis_normal(-0.054323, -0.814921, -0.577020);
    osg::Matrix transformation = osg::Matrix::translate(-pivot_point)*osg::Matrix::rotate(axis_normal, osg::Vec3(0, 0, 1));
    for (size_t i = 0; i < plant_points_num_; ++ i)
    {
      osg::Vec3 point = at(i).cast<osg::Vec3>();
      point = transformation.preMult(point);
      point_cloud.push_back(PclPoint(point.x(), point.y(), point.z()));
    }
    pcl::PLYWriter ply_writer;
    if (ply_writer.write<PclPoint>(filename, point_cloud) != 0)
      return false;
  }
  else
  {
    pcl::PCDWriter pcd_writer;
    if (pcd_writer.writeBinaryCompressed<PclRichPoint>(filename, *this) != 0)
      return false;
  }

  return true;
}

void PointCloud::save(void)
{
  MainWindow* main_window = MainWindow::getInstance();
  QString filename = QFileDialog::getSaveFileName(main_window, "Save Point Cloud",
    main_window->getWorkspace(), "Point Cloud (*.pcd *.ply)");
  if (filename.isEmpty())
    return;

  save(filename.toStdString());

  return;
}

void PointCloud::reload(void)
{
  clearData();
  open(filename_);

  return;
}

void PointCloud::clearData()
{
  QMutexLocker locker(&mutex_);

  Renderable::clear();
  PclRichPointCloud::clear();
  triangulation_->clear();
  stems_.clear();
  leaves_.clear();

  return;
}

void PointCloud::visualizePoints(size_t start, size_t end)
{
  osg::ref_ptr<osg::Vec3Array>  vertices = new osg::Vec3Array;
  osg::ref_ptr<osg::Vec3Array>  normals_vertices = new osg::Vec3Array;
  osg::ref_ptr<osg::Vec3Array>  orientations_vertices = new osg::Vec3Array;
  osg::ref_ptr<osg::Vec3Array>  normals = new osg::Vec3Array;
  osg::ref_ptr<osg::Vec4Array>  colors = new osg::Vec4Array;

  getColors(colors, start, end);

  for (size_t i=start, i_end = end;i<i_end;i++)
  {
    const PclRichPoint& point = at(i);

    if ((show_stems_ || show_leaves_)
      && (show_leaves_ && point.label != PclRichPoint::LABEL_LEAF))
      continue;
    if ((show_stems_ || show_leaves_)
      && (show_stems_ && point.label != PclRichPoint::LABEL_STEM))
      continue;

    vertices->push_back(osg::Vec3(point.x, point.y, point.z));
    normals->push_back(osg::Vec3(point.normal_x, point.normal_y, point.normal_z));
    if (show_normals_)
    {
      normals_vertices->push_back(vertices->back());
      normals_vertices->push_back(vertices->back()+osg::Vec3(point.normal_x, point.normal_y, point.normal_z));
    }
    if (show_orientations_)
    {
      orientations_vertices->push_back(vertices->back());
      orientations_vertices->push_back(vertices->back()+osg::Vec3(point.orientation_x, point.orientation_y, point.orientation_z));
    }
  }

  size_t partition_size = 10000;
  size_t item_num = vertices->size();
  size_t partition_num = (item_num+partition_size-1)/partition_size;
  osg::Geode* geode = new osg::Geode;
  for (size_t i = 0, i_end = partition_num; i < i_end; ++ i) {
    osg::UIntArray* indices = OSGUtility::generateIndices(partition_size, i, item_num);

    osg::Geometry* geometry = new osg::Geometry;
    geometry->setUseDisplayList(true);
    geometry->setUseVertexBufferObjects(true);
    geometry->setVertexData(osg::Geometry::ArrayData(vertices, indices, osg::Geometry::BIND_PER_VERTEX));
    geometry->setNormalData(osg::Geometry::ArrayData(normals, indices, osg::Geometry::BIND_PER_VERTEX));
    if (color_mode_ == UNIFORM)
      geometry->setColorData(osg::Geometry::ArrayData(colors, osg::Geometry::BIND_OVERALL));
    else
      geometry->setColorData(osg::Geometry::ArrayData(colors, indices, osg::Geometry::BIND_PER_VERTEX));
    geometry->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::POINTS, 0, indices->size()));

    geode->addDrawable(geometry);

    if (show_normals_)
    {
      osg::UIntArray* normals_indices = OSGUtility::generateIndices(partition_size, i, item_num, 2);

      osg::Geometry* normals_geometry = new osg::Geometry;
      normals_geometry->setUseDisplayList(true);
      normals_geometry->setUseVertexBufferObjects(true);
      normals_geometry->setVertexData(osg::Geometry::ArrayData(normals_vertices, normals_indices, osg::Geometry::BIND_PER_PRIMITIVE));
      if (color_mode_ == UNIFORM)
        normals_geometry->setColorData(osg::Geometry::ArrayData(colors, osg::Geometry::BIND_OVERALL));
      else
        normals_geometry->setColorData(osg::Geometry::ArrayData(colors, indices, osg::Geometry::BIND_PER_PRIMITIVE));
      normals_geometry->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINES, 0, normals_indices->size()));

      geode->addDrawable(normals_geometry);
    }

    if (show_orientations_)
    {
      osg::UIntArray* orientations_indices = OSGUtility::generateIndices(partition_size, i, item_num, 2);

      osg::Geometry* orientations_geometry = new osg::Geometry;
      orientations_geometry->setUseDisplayList(true);
      orientations_geometry->setUseVertexBufferObjects(true);
      orientations_geometry->setVertexData(osg::Geometry::ArrayData(orientations_vertices, orientations_indices, osg::Geometry::BIND_PER_PRIMITIVE));
      if (color_mode_ == UNIFORM)
        orientations_geometry->setColorData(osg::Geometry::ArrayData(colors, osg::Geometry::BIND_OVERALL));
      else
        orientations_geometry->setColorData(osg::Geometry::ArrayData(colors, indices, osg::Geometry::BIND_PER_PRIMITIVE));
      orientations_geometry->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINES, 0, orientations_indices->size()));

      geode->addDrawable(orientations_geometry);
    }
  }
  addChild(geode);

  return;
}

void PointCloud::visualizeTriangles()
{
  triangulate();

  osg::ref_ptr<osg::Vec3Array>  vertices = new osg::Vec3Array;
  osg::ref_ptr<osg::Vec4Array>  colors = new osg::Vec4Array;
  osg::Vec4 color = ColorMap::Instance().getColor(ColorMap::LIGHT_BLUE);
  color.a() = 0.4;
  colors->push_back(color);

  double thickness_threshold = ParameterManager::getInstance().getThicknessRadius()/2;

  std::vector<pcl::Vertices> polygons;
  double triangle_length = ParameterManager::getInstance().getTriangleLength();
  double triangle_length_threshold = triangle_length*triangle_length;
  for (CGAL::Delaunay::Finite_facets_iterator it = triangulation_->finite_facets_begin();
    it != triangulation_->finite_facets_end(); ++ it)
  {
    CGAL::Delaunay::Cell_handle cell_handle  = it->first;
    int                   vertex_index = it->second;

    bool small_polygon = true;
    for (size_t i = 1; i < 4; ++ i)
    {
      int source_index = (vertex_index+i)%4;
      int target_index = (i == 3)?((vertex_index+1)%4):((vertex_index+i+1)%4);

      const CGAL::Delaunay::Point& source = cell_handle->vertex(source_index)->point();
      const CGAL::Delaunay::Point& target = cell_handle->vertex(target_index)->point();
      if (CGAL::squared_distance(source, target) > triangle_length_threshold)
      {
        small_polygon = false;
        break;
      }
    }
    if (!small_polygon)
      continue;

    polygons.push_back(pcl::Vertices());
    std::vector<uint32_t>& vertices = polygons.back().vertices;
    for (size_t i = 1; i < 4; ++ i)
      vertices.push_back((unsigned int)(cell_handle->vertex((vertex_index+i)%4)->info()));
  }

  for (size_t i=0, i_end = polygons.size();i<i_end;i++)
  {
    const std::vector<uint32_t>& polygons_vertices = polygons[i].vertices;
    for (size_t j = 0, j_end = polygons_vertices.size(); j < j_end; ++ j)
    {
      const PclRichPoint& point = at(polygons_vertices[j]);
      vertices->push_back(osg::Vec3(point.x, point.y, point.z));
    }
  }

  size_t partition_size = 10000;
  size_t item_num = vertices->size()/3;
  size_t partition_num = (item_num+partition_size-1)/partition_size;
  osg::Geode* geode = new osg::Geode;
  for (size_t i = 0, i_end = partition_num; i < i_end; ++ i) {
    osg::UIntArray* vertex_indices = OSGUtility::generateIndices(partition_size, i, item_num, 3);

    osg::Geometry* geometry = new osg::Geometry;
    geometry->setUseDisplayList(true);
    geometry->setUseVertexBufferObjects(true);
    geometry->setVertexData(osg::Geometry::ArrayData(vertices, vertex_indices, osg::Geometry::BIND_PER_PRIMITIVE));
    geometry->setColorData(osg::Geometry::ArrayData(colors, osg::Geometry::BIND_OVERALL));
    geometry->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::TRIANGLES, 0, vertex_indices->size()));
    geode->addDrawable(geometry);
  }

  addChild(geode);

  return;
}

void PointCloud::updateImpl()
{
  if (show_plant_ && plant_points_num_ != 0)
    visualizePoints(0, plant_points_num_);

  if (show_noise_ && noise_points_num_ != 0)
    visualizePoints(plant_points_num_, plant_points_num_+noise_points_num_);

  if (show_pot_ && pot_points_num_ != 0)
    visualizePoints(plant_points_num_+noise_points_num_, size());

  if (plant_points_num_ == 0 && noise_points_num_ == 0 && pot_points_num_ == 0
    && (show_plant_ || show_noise_ || show_pot_))
    visualizePoints(0, size());

  if (show_triangles_)
    visualizeTriangles();

  if (show_organs_)
    visualizeOrgans();

  if (show_stem_graph_)
    visualizeStemGraph();

  if (show_draggers_)
  {
    osg::BoundingSphere boundingSphere = getBound();
    osg::Matrix trans = osg::Matrix::translate(boundingSphere.center());
    double radius = boundingSphere.radius();
    float t_scale = radius/4;
    float r_scale = radius/8;
    osg::Matrix flip(osg::Matrix::rotate(osg::Vec3(0, 1, 0), osg::Vec3(0, -1, 0)));
    translate_dragger_->setMatrix(flip*osg::Matrix::scale(t_scale, t_scale, t_scale)*trans);
    trackball_dragger_->setMatrix(osg::Matrix::scale(r_scale, r_scale, r_scale)*trans);

    addChild(translate_dragger_);
    addChild(trackball_dragger_);
  }

  return;
}

osg::Vec4 PointCloud::getColor(size_t i) const
{
  int mod = ParameterManager::getInstance().getColorizeMod();

  const PclRichPoint& point = at(i);
  osg::Vec4 color(0.0f, 0.0f, 0.0f, 1.0f);
  switch (color_mode_)
  {
  case UNIFORM:
    color = color_;
    break;
  case ORIGINAL:
    color = osg::Vec4(point.r/255.0, point.g/255.0, point.b/255.0, 1.0);
    break;
  case THICKNESS:
    {
      color = ColorMap::Instance().getColor(ColorMap::JET, point.thickness, min_thickness_, max_thickness_);
    }
    break;
  case FLATNESS:
    {
      color = ColorMap::Instance().getColor(ColorMap::JET, transformCurvature(point.curvature), getGlobalLeafFlatness(), getGlobalStemFlatness());
    }
    break;
  case SEGMENT:
    {
      if (point.segment_id == PclRichPoint::ID_UNINITIALIZED)
        color = osg::Vec4(point.r/255.0, point.g/255.0, point.b/255.0, 1.0);
      else
      {
        std::srand(point.segment_id+mod);
        size_t color_idx = std::abs(std::rand())%mod;
        color = ColorMap::Instance().getColor(ColorMap::JET, color_idx, 0, mod-1);
      }
    }
    break;
  case LABEL:
    {
      if (point.label < PclRichPoint::LABEL_LEAF)
        color = osg::Vec4(point.r/255.0, point.g/255.0, point.b/255.0, 1.0);
      else
      {
        if (point.label < PclRichPoint::LABEL_STEM)
          color = osg::Vec4(0.2, 0.8, 0.2, 1.0);
        else
          color = osg::Vec4(0.3, 0.3, 0.3, 1.0);
      }
    }
    break;
  case PROBABILITY:
    {
      color = ColorMap::Instance().getColor(ColorMap::JET, std::pow(point.probability, 5.0), 0.0, 1.01);
    }
    break;
  case ORGAN:
    {
      if (point.organ_id == PclRichPoint::ID_UNINITIALIZED)
        color = osg::Vec4(point.r/255.0, point.g/255.0, point.b/255.0, 1.0);
      else
      {
        std::srand(point.organ_id+mod);
        size_t color_idx = std::abs(std::rand())%mod;
        if (at(i).label == PclRichPoint::LABEL_STEM)
          color_idx += mod;
        color = osg::Vec4(ColorMap::Instance().getColor(ColorMap::JET, color_idx, 0, 2*mod+1));
      }
    }
    break;
  default:
    break;
  }

  return color;
}

void PointCloud::getColors(osg::Vec4Array* colors, size_t start, size_t end) const
{
  int mod = ParameterManager::getInstance().getColorizeMod();

  switch (color_mode_)
  {
  case UNIFORM:
    colors->push_back(color_);
    break;
  case ORIGINAL:
    for (size_t i=start, i_end = end;i<i_end;i++)
    {
      const PclRichPoint& point = at(i);
      if ((show_stems_ || show_leaves_)
        && (show_leaves_ && point.label != PclRichPoint::LABEL_LEAF))
        continue;
      if ((show_stems_ || show_leaves_)
        && (show_stems_ && point.label != PclRichPoint::LABEL_STEM))
        continue;

      colors->push_back(osg::Vec4(point.r/255.0, point.g/255.0, point.b/255.0, 1.0));
    }
    break;
  case THICKNESS:
    {
      for (size_t i=start, i_end = end;i<i_end;i++)
      {
        const PclRichPoint& point = at(i);
        if ((show_stems_ || show_leaves_)
          && (show_leaves_ && point.label != PclRichPoint::LABEL_LEAF))
          continue;
        if ((show_stems_ || show_leaves_)
          && (show_stems_ && point.label != PclRichPoint::LABEL_STEM))
          continue;
        colors->push_back(ColorMap::Instance().getColor(ColorMap::JET, point.thickness, min_thickness_, max_thickness_));
      }
    }
    break;
  case FLATNESS:
    {
      for (size_t i=start, i_end = end;i<i_end;i++)
      {
        const PclRichPoint& point = at(i);
        if ((show_stems_ || show_leaves_)
          && (show_leaves_ && point.label != PclRichPoint::LABEL_LEAF))
          continue;
        if ((show_stems_ || show_leaves_)
          && (show_stems_ && point.label != PclRichPoint::LABEL_STEM))
          continue;
        colors->push_back(ColorMap::Instance().getColor(ColorMap::JET, transformCurvature(at(i).curvature), getGlobalLeafFlatness(), getGlobalStemFlatness()));
      }
    }
    break;
  case SEGMENT:
    for (size_t i=start, i_end = end;i<i_end;i++)
    {
      const PclRichPoint& point = at(i);
      if ((show_stems_ || show_leaves_)
        && (show_leaves_ && point.label != PclRichPoint::LABEL_LEAF))
        continue;
      if ((show_stems_ || show_leaves_)
        && (show_stems_ && point.label != PclRichPoint::LABEL_STEM))
        continue;

      if (point.segment_id == PclRichPoint::ID_UNINITIALIZED)
        colors->push_back(osg::Vec4(point.r/255.0, point.g/255.0, point.b/255.0, 1.0));
      else
      {
        std::srand(point.segment_id+mod);
        size_t color_idx = std::abs(std::rand())%mod;
        colors->push_back(ColorMap::Instance().getColor(ColorMap::JET, color_idx, 0, mod-1));
      }
    }
    break;
  case LABEL:
    {
      for (size_t i=start, i_end = end;i<i_end;i++)
      {
        const PclRichPoint& point = at(i);
        if ((show_stems_ || show_leaves_)
          && (show_leaves_ && point.label != PclRichPoint::LABEL_LEAF))
          continue;
        if ((show_stems_ || show_leaves_)
          && (show_stems_ && point.label != PclRichPoint::LABEL_STEM))
          continue;

        if (point.label < PclRichPoint::LABEL_LEAF)
          colors->push_back(osg::Vec4(point.r/255.0, point.g/255.0, point.b/255.0, 1.0));
        else
        {
          if (point.label < PclRichPoint::LABEL_STEM)
            colors->push_back(osg::Vec4(0.2, 0.8, 0.2, 1.0));
          else
            colors->push_back(osg::Vec4(0.3, 0.3, 0.3, 1.0));
        }
      }
    }
    break;
  case PROBABILITY:
    {
      for (size_t i=start, i_end = end;i<i_end;i++)
      {
        const PclRichPoint& point = at(i);
        if ((show_stems_ || show_leaves_)
          && (show_leaves_ && point.label != PclRichPoint::LABEL_LEAF))
          continue;
        if ((show_stems_ || show_leaves_)
          && (show_stems_ && point.label != PclRichPoint::LABEL_STEM))
          continue;
        colors->push_back(ColorMap::Instance().getColor(ColorMap::JET, std::pow(point.probability, 5.0), 0.0, 1.01));
      }
    }
    break;
  case ORGAN:
    for (size_t i=start, i_end = end;i<i_end;i++)
    {
      const PclRichPoint& point = at(i);
      if ((show_stems_ || show_leaves_)
        && (show_leaves_ && point.label != PclRichPoint::LABEL_LEAF))
        continue;
      if ((show_stems_ || show_leaves_)
        && (show_stems_ && point.label != PclRichPoint::LABEL_STEM))
        continue;

      if (point.organ_id == PclRichPoint::ID_UNINITIALIZED)
        colors->push_back(osg::Vec4(point.r/255.0, point.g/255.0, point.b/255.0, 1.0));
      else
      {
        std::srand(point.organ_id+mod);
        size_t color_idx = std::abs(std::rand())%mod;
        if (at(i).label == PclRichPoint::LABEL_STEM)
          color_idx += mod;
        colors->push_back(osg::Vec4(ColorMap::Instance().getColor(ColorMap::JET, color_idx, 0, 2*mod+1)));
      }
    }
    break;
  default:
    break;
  }

  return;
}


void PointCloud::povray(SlavePovRayVisitor* povray_visitor) const
{
  // points
  bool render_all = (plant_points_num_ == 0 && noise_points_num_ == 0 && pot_points_num_ == 0
    && (show_plant_ || show_noise_ || show_pot_));

  for (size_t i = 0, i_end = size(); i < i_end; ++ i)
  {
    const PclRichPoint& point = at(i);
    if ((show_stems_ || show_leaves_)
      && (show_leaves_ && point.label != PclRichPoint::LABEL_LEAF))
      continue;
    if ((show_stems_ || show_leaves_)
      && (show_stems_ && point.label != PclRichPoint::LABEL_STEM))
      continue;

    bool render_label = (point.label == PclRichPoint::LABEL_NOISE && show_noise_)
      ||(point.label == PclRichPoint::LABEL_POT && show_pot_)
      || (point.label >= PclRichPoint::LABEL_PLANT && show_plant_);
    if (!(render_all ||render_label))
      continue;

    osg::Vec4 color = getColor(i);
    const QString& point_radius = povray_visitor->getNamedFloatIdentifier("point_size", 0.06);

    PointCloud::ColorMode color_mode = (PointCloud::ColorMode)MainWindow::getInstance()->getColorMode()->currentIndex();
    if(color_mode == PointCloud::ORGAN)
      povray_visitor->drawSphere(point.cast<CgalPoint>(), point_radius, getFrame(), point.organ_id, (point.label==PclRichPoint::LABEL_LEAF));
    else  
      povray_visitor->drawSphere(point.cast<CgalPoint>(), point_radius, color);
  }

  // triangles
  if (show_triangles_)
  {
    triangulate();

    osg::Vec4 color = ColorMap::Instance().getColor(ColorMap::LIGHT_BLUE);
    color.a() = 0.4;

    double triangle_length = ParameterManager::getInstance().getTriangleLength();
    double triangle_length_threshold = triangle_length*triangle_length;
    for (CGAL::Delaunay::Finite_facets_iterator it = triangulation_->finite_facets_begin();
      it != triangulation_->finite_facets_end(); ++ it)
    {
      CGAL::Delaunay::Cell_handle cell_handle  = it->first;
      int                   vertex_index = it->second;

      bool small_polygon = true;
      for (size_t i = 1; i < 4; ++ i)
      {
        int source_index = (vertex_index+i)%4;
        int target_index = (i == 3)?((vertex_index+1)%4):((vertex_index+i+1)%4);

        const CGAL::Delaunay::Point& source = cell_handle->vertex(source_index)->point();
        const CGAL::Delaunay::Point& target = cell_handle->vertex(target_index)->point();
        if (CGAL::squared_distance(source, target) > triangle_length_threshold)
        {
          small_polygon = false;
          break;
        }
      }
      if (!small_polygon)
        continue;

      CgalPoint p[3];
      for (size_t i = 1; i < 4; ++ i)
      {
        const PclRichPoint& point = at(cell_handle->vertex((vertex_index+i)%4)->info());
        p[i-1] = point.cast<CgalPoint>();
      }

      povray_visitor->drawTriangle(p[0], p[1], p[2], color);
    }
  }

  if (show_stem_graph_)
  {
    boost::SkeletonGraph& g_stem_skeleton = *stem_skeleton_graph_;

    int mod = ParameterManager::getInstance().getColorizeMod();
    osg::Vec4 node_color = osg::Vec4(0.55f, 0.40f, 0.03f, 1.0f);
    osg::Vec4 edge_color = osg::Vec4(0.85f, 0.65f, 0.13f, 1.0f);

    const QString& stem_skeleton_node_radius = povray_visitor->getNamedFloatIdentifier("stem_skeleton_node_radius", 0.3);
    for (size_t i = 0, i_end = boost::num_vertices(g_stem_skeleton); i < i_end; ++ i)
    {
      osg::Vec4 color = node_color;
      if (boost::degree(i, g_stem_skeleton) != 0)
      {
        int id = -g_stem_skeleton[*boost::out_edges(i, g_stem_skeleton).first].heat_kernel_distance;
        if (id > 0)
        {
          id --;
          std::srand(id+mod);
          size_t color_idx = std::abs(std::rand())%mod;
          color = ColorMap::Instance().getColor(ColorMap::JET, color_idx, 0, mod-1);
        }
      }
      povray_visitor->drawSphere(Caster<osg::Vec3, CgalPoint>(g_stem_skeleton[i]),
        stem_skeleton_node_radius, color);
    }


    const QString& stem_skeleton_edge_radius = povray_visitor->getNamedFloatIdentifier("stem_skeleton_edge_radius", 0.15);
    typedef boost::SkeletonGraphTraits::edge_iterator edge_iterator_skeleton;
    std::pair<edge_iterator_skeleton, edge_iterator_skeleton> edges_skeleton = boost::edges(g_stem_skeleton);
    for (edge_iterator_skeleton it = edges_skeleton.first; it != edges_skeleton.second; ++ it)
    {
      osg::Vec4 color = edge_color;
      const osg::Vec3& source = g_stem_skeleton[boost::source(*it, g_stem_skeleton)];
      const osg::Vec3& target = g_stem_skeleton[boost::target(*it, g_stem_skeleton)];
      int id = -g_stem_skeleton[*it].heat_kernel_distance;
      if (id > 0)
      {
        id --;
        std::srand(id+mod);
        size_t color_idx = std::abs(std::rand())%mod;
        color = ColorMap::Instance().getColor(ColorMap::JET, color_idx, 0, mod-1);
      }
      povray_visitor->drawCylinder(Caster<osg::Vec3, CgalPoint>(source), Caster<osg::Vec3, CgalPoint>(target),
        stem_skeleton_edge_radius, color);
    }
  }

  return;
}

void PointCloud::povrayOrgan(SlavePovRayVisitor* povray_visitor) const
{
  // points
  bool render_all = (plant_points_num_ == 0 && noise_points_num_ == 0 && pot_points_num_ == 0
    && (show_plant_ || show_noise_ || show_pot_));

  int frame = getFrame();
  for (size_t i = 0, i_end = size(); i < i_end; ++ i)
  {
    const PclRichPoint& point = at(i);
    if ((show_stems_ || show_leaves_)
      && (show_leaves_ && point.label != PclRichPoint::LABEL_LEAF))
      continue;
    if ((show_stems_ || show_leaves_)
      && (show_stems_ && point.label != PclRichPoint::LABEL_STEM))
      continue;

    bool render_label = (point.label == PclRichPoint::LABEL_NOISE && show_noise_)
      ||(point.label == PclRichPoint::LABEL_POT && show_pot_)
      || (point.label >= PclRichPoint::LABEL_PLANT && show_plant_);
    if (!(render_all ||render_label))
      continue;

    const QString& point_radius = povray_visitor->getNamedFloatIdentifier("point_size", 0.06);
    povray_visitor->drawSphere(point.cast<CgalPoint>(), point_radius, frame, point.organ_id, (point.label == PclRichPoint::LABEL_LEAF));
  }

  return;
}

void PointCloud::savePovrayData(void)
{
  QMutexLocker locker(&mutex_);

  ColorMode backup = color_mode_;
  color_mode_ = ORGAN;

  OSGViewerWidget* osg_viewer_widget = MainWindow::getInstance()->getOSGViewerWidget();
  FileSystemModel* model = MainWindow::getInstance()->getFileSystemModel();
  PovRayVisitor* master = MainWindow::getInstance()->getPovrayVisitorMaster();
  std::map<size_t, size_t>& frame_frequency = MainWindow::getInstance()->getFrameFrequency();
  master->setOSGViewerWidget(osg_viewer_widget);
  SlavePovRayVisitor slave(osg_viewer_widget);
  slave.setMaster(master);
  osg::ref_ptr<PointCloud> poind_cloud = this;

  if(frame_frequency.find(getFrame()) != frame_frequency.end())
    frame_frequency[getFrame()]++;
  else
    frame_frequency.insert(std::make_pair(getFrame(),1));

  QString data_filename = QString("data_%1_%2.inc").arg(getFrame(), 5, 10, QChar('0')).arg(frame_frequency[getFrame()], 2, 10, QChar('0'));
  QString data_filepath = model->rootPath()+"/POV-Ray/data/"+data_filename;
  QFile data_file(data_filepath);
  data_file.open(QIODevice::WriteOnly | QIODevice::Text);
  QTextStream data_file_stream(&data_file);
  if (color_mode_ != ORGAN)
    poind_cloud->povray(&slave);
  else
    poind_cloud->povrayOrgan(&slave);
  data_file_stream << slave.getPovrayText();
  data_file.close();

  QString texture_filename = QString("texture_%1_%2.inc").arg(getFrame(), 5, 10, QChar('0')).arg(frame_frequency[getFrame()], 2, 10, QChar('0'));
  QString texture_filepath = model->rootPath()+"/POV-Ray/texture/"+texture_filename;
  if (color_mode_ != ORGAN)
    slave.saveTextureFile(texture_filepath);
  else
    slave.saveTextureFileOrgan(texture_filepath, getFrame(), getLeafNum(), getStemNum());

  QString folder(model->rootPath()+"/POV-Ray"); 

  QString size_filename(folder+"/size.inc");
  slave.saveSizeFile(size_filename);

  QString light_filename(folder+"/light.inc");
  slave.saveLightFile(light_filename);

  QString camera_filename(folder+"/camera.inc");
  slave.saveCameraFile(camera_filename, NULL, 1);


  QFile povray_file(folder+"/scene.pov");
  povray_file.open(QIODevice::WriteOnly | QIODevice::Text | QIODevice::Append);
  QTextStream povray_file_stream(&povray_file); 
  QString include = QString("#if(frame_number = %1)\n").arg(master->getFrameNumber()++);
  QString data_include = QString("#include \"data\\data_%1_%2.inc\"\n").arg(getFrame(), 5, 10, QChar('0')).arg(frame_frequency[getFrame()], 2, 10, QChar('0'));
  QString texture_include = QString("#include \"texture\\texture_%1_%2.inc\"\n").arg(getFrame(), 5, 10, QChar('0')).arg(frame_frequency[getFrame()], 2, 10, QChar('0'));
  QString end = QString("#end\n");
  povray_file_stream << include;
  povray_file_stream << texture_include;
  povray_file_stream << data_include;
  povray_file_stream << end;

  QFile ini_file(folder+"/scene.ini");
  ini_file.open(QIODevice::WriteOnly | QIODevice::Text);
  QTextStream ini_file_stream(&ini_file);
  ini_file_stream << "Input_File_Name = \"scene.pov\"\nOutput_File_Name = scene_\n\n";
  ini_file_stream << "Antialias = On\nAntialias_Threshold = 0.3\nAntialias_Depth = 2\n\n";
  ini_file_stream << "Output_Alpha = On\n\n";
  ini_file_stream << QString("Height = %1\nWidth = %2\n\n").arg(osg_viewer_widget->height()).arg(osg_viewer_widget->width());
  ini_file_stream << QString("Initial_Frame = %1\nFinal_Frame= %2\n\n").arg(0).arg(master->getFrameNumber()-1);

  color_mode_ = backup;

  return;

}

void PointCloud::initPovrayFile(void)
{
  FileSystemModel* model = MainWindow::getInstance()->getFileSystemModel();
  QDir root(model->rootPath());
  root.mkdir("POV-Ray");
  root.mkdir("POV-Ray/data");
  root.mkdir("POV-Ray/texture");

  QString folder(model->rootPath()+"/POV-Ray");
  QFile povray_file(folder+"/scene.pov");
  povray_file.open(QIODevice::WriteOnly | QIODevice::Text);
  QTextStream povray_file_stream(&povray_file);
  povray_file_stream << "#version 3.7;\n\n";
  povray_file_stream << "global_settings { assumed_gamma 1.0 }\n\n";
  povray_file_stream << "#include \"camera.inc\"\n";
  povray_file_stream << "#include \"light.inc\"\n";
  povray_file_stream << "#include \"size.inc\"\n";
  povray_file_stream << "#include \"global_texture.inc\"\n";
}

void PointCloud::clearPovrayConfig(void)
{
  std::map<size_t, size_t> frame_frequency = MainWindow::getInstance()->getFrameFrequency();
  frame_frequency.erase(frame_frequency.begin(),frame_frequency.end());
  PovRayVisitor* master = MainWindow::getInstance()->getPovrayVisitorMaster();
  master->getFrameNumber() = 0;
}

PointCloud* PointCloud::getPrevFrame(void)
{
  FileSystemModel* model = MainWindow::getInstance()->getFileSystemModel();
  return model->getPointCloud(getFrame()-1);
}

PointCloud* PointCloud::getNextFrame(void)
{
  FileSystemModel* model = MainWindow::getInstance()->getFileSystemModel();
  return model->getPointCloud(getFrame()+1);
}



int PointCloud::getFrame(void) const      
{
  QRegExp frame("[\\/]frame_([0-9]{5,5})[\\/]");
  frame.indexIn(filename_.c_str());
  QString index = frame.cap(1);

  return index.toInt();
}

int PointCloud::getView(void) const
{
  QRegExp frame("[\\/]view_([0-9]{2,2})[\\/]");
  frame.indexIn(filename_.c_str());
  QString index = frame.cap(1);
  if (index.isEmpty())
    return 12;

  return index.toInt();
}

void PointCloud::setRotation(void)
{
  ParameterDialog parameter_dialog("Rotation Parameters", MainWindow::getInstance());
  int view = getView();
  DoubleParameter rotation_angle("Rotation Angle", "Rotation Angle", (view<7)?(-view*30):(12-view)*30, -180, 180, 30);
  parameter_dialog.addParameter(&rotation_angle);
  if (!parameter_dialog.exec() == QDialog::Accepted)
    return;

  double angle = rotation_angle*M_PI/180.0;
  setMatrix(MainWindow::getInstance()->getRegistrator()->getRotationMatrix(angle));

  if (angle == 0.0)
    deleteTransformation();

  return;
}

void PointCloud::toggleDraggers(void)
{
  QMutexLocker locker(&mutex_);
  show_draggers_ = !show_draggers_;
  expire();

  return;
}

void PointCloud::toggleRenderPlant(void)
{
  QMutexLocker locker(&mutex_);
  show_plant_ = !show_plant_;
  expire();

  return;
}
void PointCloud::toggleRenderPot(void)
{
  QMutexLocker locker(&mutex_);
  show_pot_ = !show_pot_;
  expire();

  return;
}
void PointCloud::toggleRenderNoise(void)
{
  QMutexLocker locker(&mutex_);
  show_noise_ = !show_noise_;
  expire();

  return;
}
void PointCloud::toggleRenderNormals(void)
{
  QMutexLocker locker(&mutex_);
  show_normals_ = !show_normals_;
  expire();

  return;
}

void PointCloud::toggleRenderOrientations(void)
{
  QMutexLocker locker(&mutex_);
  show_orientations_ = !show_orientations_;
  expire();

  return;
}

void PointCloud::toggleRenderStems(void)
{
  QMutexLocker locker(&mutex_);
  show_stems_ = !show_stems_;
  expire();

  return;
}

void PointCloud::toggleRenderLeaves(void)
{
  QMutexLocker locker(&mutex_);
  show_leaves_ = !show_leaves_;
  expire();

  return;
}

void PointCloud::toggleRenderTriangles(void)
{
  QMutexLocker locker(&mutex_);
  show_triangles_ = !show_triangles_;
  expire();

  return;
}

void PointCloud::toggleRenderOrgans(void)
{
  QMutexLocker locker(&mutex_);
  show_organs_ = !show_organs_;
  expire();

  return;
}

void PointCloud::toggleRenderStemGraph(void)
{
  QMutexLocker locker(&mutex_);
  show_stem_graph_ = !show_stem_graph_;
  expire();

  return;
}

void PointCloud::toggleRegisterState(void)
{
  registered_ = !registered_;
  FileSystemModel* model = MainWindow::getInstance()->getFileSystemModel();
  if (registered_)
  {
    saveTransformation();
    //    osg::ref_ptr<PointCloud> point_cloud = model->getPointCloud(getFrame());
    //    point_cloud->initPovrayFile();
    //    point_cloud->savePovrayData();
    QFile camera = model->rootPath()+"/camera.inc";
    camera.open(QIODevice::WriteOnly | QIODevice::Text);
    QTextStream camera_fout(&camera);
    camera_fout<<"background {rgbft<0.0, 0.0, 0.0, 1.0, 1.0>}"<<"\n\n";
    camera_fout<<"camera"<<"\n";
    camera_fout<<"{"<<"\n";
    camera_fout<<"  location <0,0,-5>"<<"\n";
    camera_fout<<"  look_at <0,0,1>"<<"\n";
    camera_fout<<"  up y"<<"\n";
    camera_fout<<"}"<<"\n";

    QFile light = model->rootPath()+"/light.inc";
    light.open(QIODevice::WriteOnly | QIODevice::Text);
    QTextStream light_fout(&light);
    light_fout<<"light_source { <500,500,-1000> White }"<<"\n";

    int start_frame = model->getStartFrame();
    int end_frame = model->getEndFrame();
    int delta = 4;
    QFile text = model->rootPath()+"/text.inc";
    text.open(QIODevice::WriteOnly | QIODevice::Text);
    QTextStream text_fout(&text);
    int i = start_frame;
    int day = 0;
    QString day_str;
    while(i != (end_frame + 1))
    {
       if((i - start_frame)%delta == 0)
         day ++;
       if(day > 9)
         day_str = QString("  ttf \"timrom.ttf\" \"%1\" 0, 0.0*y").arg(day, 2, 10, QChar('0'));
       else
         day_str = QString("  ttf \"timrom.ttf\" \"%1\" 0, 0.0*y").arg(day, 1, 10, QChar('0'));

       text_fout<<QString("#if(frame_number = %1)\n").arg(i - start_frame);
       text_fout<<"text"<<"\n";
       text_fout<<"{"<<"\n";
       text_fout<<"  ttf \"timrom.ttf\" \"Day\" 0, 0.0*y"<<"\n";
       text_fout<<"  pigment { Black }"<<"\n";
       text_fout<<"  translate -3*x"<<"\n";
       text_fout<<"}"<<"\n";  

       text_fout<<"text"<<"\n";
       text_fout<<"{"<<"\n";
       text_fout<<day_str<<"\n";
       text_fout<<"  pigment { Red }"<<"\n";
       text_fout<<"  translate -1*x"<<"\n";
       text_fout<<"}"<<"\n";
       text_fout<<"#end\n";

       i++;
    }
    

    QFile pov = model->rootPath()+"/text.pov";
    pov.open(QIODevice::WriteOnly | QIODevice::Text);
    QTextStream pov_fout(&pov);

    pov_fout << "#version 3.7;\n\n";
    pov_fout << "global_settings { assumed_gamma 1.0 }\n\n";
    pov_fout << "#include \"colors.inc\"\n";
    pov_fout << "#include \"camera.inc\"\n";
    pov_fout << "#include \"light.inc\"\n";
    pov_fout << "#include \"text.inc\"\n";
  }
  else
    deleteTransformation();

  return;
}

struct CompareByLabel
{
  bool operator()(const PclRichPoint& a, const PclRichPoint& b)
  {
    return a.label > b.label;
  }
};

void PointCloud::extractPlant(int segment_threshold, double triangle_length)
{
  QMutexLocker locker(&mutex_);

  Registrator* registrator = MainWindow::getInstance()->getRegistrator();
  osg::Vec3 pivot_point = registrator->getPivotPoint();
  osg::Vec3 axis_normal = registrator->getAxisNormal();
  osg::Vec4 plane = osg::Plane(axis_normal, pivot_point).asVec4();
  double a = plane.x();
  double b = plane.y();
  double c = plane.z();
  double d = plane.w();

  plant_points_num_ = 0;
  for (size_t i=0, i_end = size();i<i_end;i++) {
    const PclRichPoint& point = at(i);
    double check = a*point.x + b*point.y + c*point.z + d;
    if (check > 0)
    {
      plant_points_num_ ++;
      at(i).label = PclRichPoint::LABEL_PLANT;
    }
    else
      at(i).label = PclRichPoint::LABEL_POT;
  }
  pot_points_num_ = size() - plant_points_num_;

  std::sort(points.begin(), points.end(), CompareByLabel());

  locker.unlock();
  denoise(segment_threshold, triangle_length);

  return;
}

void PointCloud::denoise(int segment_threshold, double triangle_length)
{
  QMutexLocker locker(&mutex_);

  initPointGraph(ParameterManager::getInstance().getTriangleLength());

  boost::PointGraph& g_point = *point_graph_;
  std::vector<boost::PointGraphTraits::vertex_descriptor> component(boost::num_vertices(g_point));
  size_t component_num = boost::connected_components(g_point, &component[0]);

  std::vector<std::vector<boost::PointGraphTraits::vertex_descriptor> > components(component_num);
  for (size_t i = 0; i < plant_points_num_; ++ i)
    components[component[i]].push_back(i);

  for (size_t i = 0, i_end = components.size(); i < i_end; ++ i)
  {
    if (components[i].size() >= segment_threshold)
      continue;

    for (size_t j = 0, j_end = components[i].size(); j < j_end; ++ j)
      at(components[i][j]).label = PclRichPoint::LABEL_NOISE;

    noise_points_num_ += components[i].size();
  }
  plant_points_num_ -= noise_points_num_;

  std::sort(points.begin(), points.end(), CompareByLabel());

  triangulation_->clear();
  g_point.clear();

  expire();

  return;
}

void PointCloud::getPlantPoints(PclPointCloud& plant)
{
  plant.clear();

  for (size_t i = 0, i_end = plant_points_num_; i < i_end; ++ i)
  {
    PclRichPoint& point = at(i);
    plant.push_back(PclPoint(point.x, point.y, point.z));
  }

  return;
}

void PointCloud::getTransformedPoints(PclPointCloud& points)
{
  points.clear();

  const osg::Matrix& matrix = getMatrix();
  for (size_t i = 0, i_end = size(); i < i_end; ++ i)
  {
    osg::Vec3 point = at(i).cast<osg::Vec3>();
    point = matrix.preMult(point);
    points.push_back(PclPoint(point));
  }

  return;
}

void PointCloud::getTransformedPotPoints(PclPointCloud& pot)
{
  pot.clear();

  const osg::Matrix& matrix = getMatrix();
  for (size_t i = plant_points_num_+noise_points_num_, i_end = size(); i < i_end; ++ i)
  {
    osg::Vec3 point = at(i).cast<osg::Vec3>();
    point = matrix.preMult(point);
    pot.push_back(PclPoint(point));
  }

  return;
}

void PointCloud::getTransformedPlantPoints(PclPointCloud& plant)
{
  plant.clear();

  const osg::Matrix& matrix = getMatrix();
  for (size_t i = 0, i_end = plant_points_num_; i < i_end; ++ i)
  {
    osg::Vec3 point = at(i).cast<osg::Vec3>();
    point = matrix.preMult(point);
    plant.push_back(PclPoint(point));
  }

  return;
}

void PointCloud::getPlantPoints(PclRichPointCloud& plant)
{
  plant.clear();
  ;
  for (size_t i = 0, i_end = plant_points_num_; i < i_end; ++ i)
    plant.push_back(at(i));

  return;
}

void PointCloud::getPotPoints(PclPointCloud& pot)
{
  pot.clear();

  for (size_t i = plant_points_num_+noise_points_num_, i_end = size(); i < i_end; ++ i)
  {
    PclRichPoint& point = at(i);
    pot.push_back(PclPoint(point.x, point.y, point.z));
  }

  return;
}

void PointCloud::getPotPoints(PclRichPointCloud& pot)
{
  pot.clear();

  for (size_t i = plant_points_num_+noise_points_num_, i_end = size(); i < i_end; ++ i)
    pot.push_back(at(i));

  return;
}

void PointCloud::setUniformColor(const QColor& color)
{
  osg::Vec4 new_color = osg::Vec4(color.red()/255.0, color.green()/255.0, color.blue()/255.0, color.alpha()/255.0);
  if (color_ == new_color && color_mode_ == UNIFORM)
    return;

  QMutexLocker locker(&mutex_);

  color_ = new_color;
  color_mode_ = UNIFORM;

  expire();

  return;
}

void PointCloud::setUniformColor(void)
{
  QColor initial(color_.r()*255, color_.g()*255, color_.b()*255, color_.a()*255);
  QColor color = QColorDialog::getColor(initial, MainWindow::getInstance(),
    "Select Color for the Point Cloud", QColorDialog::ShowAlphaChannel);

  if (!color.isValid())
    return;

  setUniformColor(color);

  return;
}

void PointCloud::setColorMode(ColorMode color_mode)
{
  if (color_mode_ == color_mode)
    return;

  QMutexLocker locker(&mutex_);
  color_mode_ = color_mode;
  expire();
  return;
}


void PointCloud::setOriginalColor(void)
{
  setColorMode(ORIGINAL);
}

void PointCloud::setThicknessColor(void)
{
  setColorMode(THICKNESS);
}

void PointCloud::setFlatnessColor(void)
{
  setColorMode(FLATNESS);
}

void PointCloud::setSegmentColor(void)
{
  setColorMode(SEGMENT);
}

void PointCloud::setLabelColor(void)
{
  setColorMode(LABEL);
}

void PointCloud::setProbabilityColor(void)
{
  setColorMode(PROBABILITY);
}

void PointCloud::setOrganColor(void)
{
  setColorMode(ORGAN);
}

void PointCloud::loadTransformation(void)
{
  std::string filename = (QFileInfo(filename_.c_str()).path()+"/transformation.txt").toStdString();
  FILE *file = fopen(filename.c_str(),"r");
  if (file == NULL)
    return;

  osg::Matrix matrix;
  for (int i = 0; i < 4; ++ i)
  {
    for (int j = 0; j < 4; ++ j)
    {
      double element;
      fscanf(file, "%lf", &element);
      matrix(j, i) = element;
    }
  }
  setMatrix(matrix);
  fclose(file);

  return;
}

void PointCloud::saveTransformation(void)
{
  std::string filename = (QFileInfo(filename_.c_str()).path()+"/transformation.txt").toStdString();
  FILE *file = fopen(filename.c_str(),"w");
  if (file == NULL)
    return;

  osg::Matrix matrix = getMatrix();
  for (int i = 0; i < 4; ++ i)
  {
    for (int j = 0; j < 4; ++ j)
    {
      fprintf(file, "%lf ", matrix(j, i));
    }
    fprintf(file, "\n");
  }
  fclose(file);

  return;
}

void PointCloud::deleteTransformation(void)
{
  std::string filename = (QFileInfo(filename_.c_str()).path()+"/transformation.txt").toStdString();
  std::remove(filename.c_str());
}

void PointCloud::triangulate(void) const
{
  if (triangulation_->number_of_vertices() != 0)
    return;

  for (size_t i = 0, i_end = plant_points_num_; i < i_end; ++ i)
  {
    const PclRichPoint& point = at(i);
    CGAL::Delaunay::Point delaunay_point(point.x, point.y, point.z);
    CGAL::Delaunay::Vertex_handle vertex_handle = triangulation_->insert(delaunay_point);
    vertex_handle->info() = i;
  }

  return;
}

void PointCloud::initPointGraph(double distance_threshold)
{
  if (distance_threshold == point_graph_threshold_
    && boost::num_edges(*point_graph_) != 0)
    return;

  triangulate();
  point_graph_threshold_ = distance_threshold;

  boost::PointGraph& g_point = *point_graph_;
  g_point = boost::PointGraph(plant_points_num_);

  size_t t = 8;

  for (CGAL::Delaunay::Finite_edges_iterator it = triangulation_->finite_edges_begin();
    it != triangulation_->finite_edges_end(); ++ it)
  {
    const CGAL::Delaunay::Cell_handle& cell_handle  = it->first;
    const CGAL::Delaunay::Vertex_handle& source_handle = cell_handle->vertex(it->second);
    const CGAL::Delaunay::Vertex_handle& target_handle = cell_handle->vertex(it->third);
    double distance_L2 = CGAL::squared_distance(source_handle->point(), target_handle->point());
    double distance_L1 = std::sqrt(distance_L2);
    if (distance_L1 > point_graph_threshold_)
      continue;

    size_t source_id = source_handle->info();
    size_t target_id = target_handle->info();
    assert (source_id < plant_points_num_ && target_id < plant_points_num_);
    WeightedEdge weighted_edge(distance_L1, std::exp(-distance_L2/t));
    boost::add_edge(source_id, target_id, weighted_edge, g_point);
  }

  return;
}

void PointCloud::initMinMaxCurvature(void)
{
  if (size() == 0)
    return;

  //// save curvature
  //std::string filename = (QFileInfo(filename_.c_str()).path()+"/curvature.txt").toStdString();
  //FILE *file = fopen(filename.c_str(),"w");
  //if (file == NULL)
  //  return;
  //for (size_t i = 0, i_end = size(); i < i_end; ++ i)
  //  fprintf(file, "%f\n", at(i).curvature);
  //fclose(file);

  for (size_t i = 0, i_end = size(); i < i_end; ++ i)
    min_curvature_ = std::min(min_curvature_, at(i).curvature);

  return;
}

void PointCloud::initMinMaxThickness(void)
{
  if (size() == 0)
    return;

  //// save thickness
  //std::string filename = (QFileInfo(filename_.c_str()).path()+"/thickness.txt").toStdString();
  //FILE *file = fopen(filename.c_str(),"w");
  //if (file == NULL)
  //  return;
  //for (size_t i = 0, i_end = size(); i < i_end; ++ i)
  //  fprintf(file, "%f\n", at(i).thickness);
  //fclose(file);

  min_thickness_ = max_thickness_ = at(0).thickness;
  for (size_t i = 0, i_end = size(); i < i_end; ++ i)
  {
    min_thickness_ = std::min(min_thickness_, at(i).thickness);
    max_thickness_ = std::max(max_thickness_, at(i).thickness);
  }

  return;
}

void PointCloud::estimateNormal(double normal_radius)
{
  QMutexLocker locker(&mutex_);

  // Create an empty kdtree representation, and pass it to the normal estimation object.
  // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
  pcl::search::KdTree<PclRichPoint>::Ptr tree (new pcl::search::KdTree<PclRichPoint> ());
  tree->setInputCloud(boost::shared_ptr<const PclRichPointCloud>(this, NullDeleter()));

  // Create the normal estimation class, and pass the input dataset to it
  pcl::NormalEstimation<PclRichPoint, PclRichPoint> normal_estimator;
  normal_estimator.setInputCloud(boost::shared_ptr<const PclRichPointCloud>(this, NullDeleter()));
  normal_estimator.setSearchMethod(tree);
  // Use all neighbors in a sphere of the search radius
  normal_estimator.setRadiusSearch(normal_radius);
  normal_estimator.setViewPoint(0, 0, 0);

  normal_estimator.compute(*this);

  initMinMaxCurvature();

  expire();

  return;
}

void PointCloud::estimateOrientation(double orientation_radius)
{
  QMutexLocker locker(&mutex_);

  Registrator* registrator = MainWindow::getInstance()->getRegistrator();
  osg::Vec3 normal = registrator->getAxisNormal();
  for (size_t i = 0; i < plant_points_num_; ++ i)
  {
    std::vector<int> neighbor_indices;
    std::vector<float> neighbor_squared_distances;
    int neighbor_num = kdtree_->radiusSearch(at(i), orientation_radius, neighbor_indices, neighbor_squared_distances);

    if (neighbor_indices.size() < 3)
      continue;

    pcl::PCA<PclRichPoint> pca;
    pca.setInputCloud(boost::shared_ptr<const PclRichPointCloud>(this, NullDeleter()));
    pca.setIndices(boost::shared_ptr<const std::vector<int> >(&neighbor_indices, NullDeleter()));

    const Eigen::Matrix3f& eigen_vectors = pca.getEigenVectors();

    osg::Vec3 orientation(eigen_vectors(0, 0), eigen_vectors(1, 0), eigen_vectors(2, 0));
    orientation.normalize();
    if (orientation*normal < 0)
      orientation = -orientation;
    at(i).orientation_x = orientation.x();
    at(i).orientation_y = orientation.y();
    at(i).orientation_z = orientation.z();
  }

  expire();

  return;
}

void PointCloud::estimateThickness(double thickness_radius)
{
  QMutexLocker locker(&mutex_);

  for (size_t i = 0; i < plant_points_num_; ++ i)
  {
    std::vector<int> neighbor_indices;
    std::vector<float> neighbor_squared_distances;
    int neighbor_num = kdtree_->radiusSearch(at(i), thickness_radius, neighbor_indices, neighbor_squared_distances);

    if (neighbor_indices.size() < 3)
    {
      at(i).thickness = 0.0;
      continue;
    }

    pcl::PCA<PclRichPoint> pca;
    pca.setInputCloud(boost::shared_ptr<const PclRichPointCloud>(this, NullDeleter()));
    pca.setIndices(boost::shared_ptr<const std::vector<int> >(&neighbor_indices, NullDeleter()));

    const Eigen::Vector3f& eigen_values = pca.getEigenValues();
    assert(eigen_values.x() >= eigen_values.y() && eigen_values.y() >= eigen_values.z());
    at(i).thickness = thickness_radius*eigen_values.y()/eigen_values.x();
  }

  expire();

  return;
}

void PointCloud::registration(int segment_threshold, int max_iterations, double max_distance)
{
  if (getView() != 12)
    return;

  Registrator* registrator = MainWindow::getInstance()->getRegistrator();
  registrator->registrationLUM(segment_threshold, max_iterations, max_distance, getFrame());

  expire();

  return;
}

void PointCloud::registration(void)
{
  int segment_threshold, max_iterations;
  double max_distance;
  if (!ParameterManager::getInstance().getRegistrationLUMParameters(segment_threshold, max_iterations, max_distance))
    return;

  QFutureWatcher<void>* watcher = new QFutureWatcher<void>(this);
  connect(watcher, SIGNAL(finished()), watcher, SLOT(deleteLater()));

  int frame = getFrame();
  QString running_message = QString("Computing registration for frame %1!").arg(frame);
  QString finished_message = QString("Registration for frame %1 computed!").arg(frame);
  Messenger* messenger = new Messenger(running_message, finished_message, this);
  connect(watcher, SIGNAL(started()), messenger, SLOT(sendRunningMessage()));
  connect(watcher, SIGNAL(finished()), messenger, SLOT(sendFinishedMessage()));

  watcher->setFuture(QtConcurrent::run(this, &PointCloud::registration, segment_threshold, max_iterations, max_distance));

  return;
}

void PointCloud::setRenderPlant(bool render)
{
  if (show_plant_ == render)
    return;

  QMutexLocker locker(&mutex_);
  show_plant_ = render;
  expire();
  return;
}

void PointCloud::setRenderPot(bool render)
{
  if (show_pot_ == render)
    return;

  QMutexLocker locker(&mutex_);
  show_pot_ = render;
  expire();
  return;
}

void PointCloud::setRenderNoise(bool render)
{
  if (show_noise_ == render)
    return;

  QMutexLocker locker(&mutex_);
  show_noise_ = render;
  expire();
  return;
}

void PointCloud::setRenderNormals(bool render)
{
  if (show_normals_ == render)
    return;

  QMutexLocker locker(&mutex_);
  show_normals_ = render;
  expire();
  return;
}

void PointCloud::setRenderOrientations(bool render)
{
  if (show_orientations_ == render)
    return;

  QMutexLocker locker(&mutex_);
  show_orientations_ = render;
  expire();
  return;
}


void PointCloud::setRenderStems(bool render)
{
  if (show_stems_ == render)
    return;

  QMutexLocker locker(&mutex_);
  show_stems_ = render;
  expire();
  return;
}


void PointCloud::setRenderLeaves(bool render)
{
  if (show_leaves_ == render)
    return;

  QMutexLocker locker(&mutex_);
  show_leaves_ = render;
  expire();
  return;
}

void PointCloud::setRenderTriangles(bool render)
{
  if (show_triangles_ == render)
    return;

  QMutexLocker locker(&mutex_);
  show_triangles_ = render;
  expire();
  return;
}

void PointCloud::setRenderOrgans(bool render)
{
  if (show_organs_ == render)
    return;

  QMutexLocker locker(&mutex_);
  show_organs_ = render;
  expire();
  return;
}

void PointCloud::setRenderStemGraph(bool render)
{
  if (show_stem_graph_ == render)
    return;

  QMutexLocker locker(&mutex_);
  show_stem_graph_ = render;
  expire();
  return;
}

bool PointCloud::isShown(void) const
{
  FileSystemModel* model = MainWindow::getInstance()->getFileSystemModel();

  return model->isShown(filename_);
}

void PointCloud::initRotation(void)
{
  if (!getMatrix().isIdentity())
    return;

  int view = getView();
  if (view == 0)
    return;

  double angle = ((view<7)?(-view):(12-view))*M_PI/6;
  setMatrix(MainWindow::getInstance()->getRegistrator()->getRotationMatrix(angle));

  return;
}

void PointCloud::setPlantPointsNum(size_t plant_points_num)
{
  plant_points_num_ = plant_points_num;

  return;
}

void PointCloud::setPotPointsNum(size_t pot_points_num)
{
  pot_points_num_ = pot_points_num;

  return;
}