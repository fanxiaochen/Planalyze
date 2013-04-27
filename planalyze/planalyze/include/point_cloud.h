#pragma once
#ifndef POINTCLOUD_H
#define POINTCLOUD_H

#include <QObject>
#include <QColor>
#include <osg/Array>
#include <unordered_set>

#include "organ.h"
#include "forward.h"
#include "cgal_types.h"
#include "pcl_wrapper_types.h"
#include "Renderable.h"
#include "povray_visitor.h"

namespace osgManipulator
{
  class TranslateAxisDragger;
  class TrackballDragger;
}

class GCoptimizationGeneralGraph;

class PointCloud : public QObject, public Renderable, public PclRichPointCloud
{
  Q_OBJECT

public:
  PointCloud(void);
  virtual ~PointCloud(void);

  virtual const char* className() const {return "PointCloud";}

  typedef boost::shared_ptr<pcl::KdTreeFLANN<PclRichPoint> > KdTreePtr;

  bool open(const std::string& filename);
  bool save(const std::string& filename);
  void reload(void);

  void updateStatistics(void);

  virtual void povray(SlavePovRayVisitor* povray_visitor) const;
  virtual void povrayOrgan(SlavePovRayVisitor* povray_visitor) const;
  void savePovrayData(void);
  static void initPovrayFile(void);
  static void clearPovrayConfig(void);

  inline const std::string& getFilename(void) const {return filename_;}

  size_t getPlantPointsNum(void) const {return plant_points_num_;}
  void setPlantPointsNum(size_t plant_points_num);
  void setPotPointsNum(size_t pot_points_num);

  void getPlantPoints(PclPointCloud& plant);
  void getPlantPoints(PclRichPointCloud& plant);

  void getPotPoints(PclPointCloud& pot);
  void getPotPoints(PclRichPointCloud& pot);

  void getTransformedPoints(PclPointCloud& points);
  void getTransformedPlantPoints(PclPointCloud& points);
  void getTransformedPotPoints(PclPointCloud& pot);

  std::vector<Organ>& getStems(void) {return stems_;}
  std::vector<Organ>& getLeaves(void) {return leaves_;}

  inline bool isRegistered(void) const {return registered_;}
  void setRegisterState(bool registered);

  void setRenderPlant(bool render);
  void setRenderPot(bool render);
  void setRenderNoise(bool render);
  void setRenderNormals(bool render);
  void setRenderOrientations(bool render);
  void setRenderLeaves(bool render);
  void setRenderStems(bool render);
  void setRenderTriangles(bool render);
  void setRenderOrgans(bool render);

  void registration(int segment_threshold, int max_iterations, double max_distance);
  void extractPlant(int segment_threshold, double triangle_length);
  void estimateNormal(double normal_radius);
  void estimateThickness(double thickness_radius);
  void estimateOrientation(double orientation_radius);

  int getFrame(void) const;
  int getView(void) const;
  bool isShown(void) const;
  void initRotation(void);

  enum ColorMode
  {
    ORIGINAL,
    LABEL,
    ORGAN,
    THICKNESS,
    FLATNESS,
    SEGMENT,
    PROBABILITY,
    UNIFORM,
  };
  void setColorMode(ColorMode color_mode);
  ColorMode getColorMode(void) const {return color_mode_;}
  void setUniformColor(const QColor& color);

  size_t getLeafNum(void) const {return leaves_.size();}
  size_t getStemNum(void) const {return stems_.size();}
  CGAL::Delaunay* getTriangulation(void);

  KdTreePtr getKdTree(void) {return kdtree_;}
  KdTreePtr getLeafKdTree(int id) const;
  KdTreePtr getStemKdTree(int id) const;
  static double computePointKdTreeDistanceL2(const PclRichPoint& point, KdTreePtr kdtree);
  double computeOrganKdTreeDistance(Organ& organ, KdTreePtr kdtree);

  // Organ Graph
  void classifyLeafStem(double smooth_cost, bool forward);
  void smoothLeaves(double smooth_cost, bool forward);
  void smoothStems(double smooth_cost, bool forward);
  void reorderOrgans(bool forward);

  static double transformCurvature(double curvature);

  static double getGlobalLeafFlatness(void);
  static double getGlobalStemFlatness(void);

  static double getGlobalLeafThickness(void);
  static double getGlobalStemThickness(void);

  static double getGlobalLeafFeature(void);
  static double getGlobalStemFeature(void);

  static double computePointLeafDistance(const PclRichPoint& point, const Organ& leaf);
  static double computePointStemDistance(const PclRichPoint& point, const Organ& stem);

  static double computePointLeafDistance(const PclRichPoint& point);
  static double computePointStemDistance(const PclRichPoint& point);

  static double getPointFeature(const PclRichPoint& point);
  static double getOrganFeature(const Organ& organ);

  void trimOrgans(bool is_leaf);

  KdTreePtr getOrganKdTree(int id, bool leaf) const;

public slots:
  void setRotation(void);
  void registration(void);

  void setOriginalColor(void);
  void setUniformColor(void);
  void setThicknessColor(void);
  void setFlatnessColor(void);
  void setSegmentColor(void);
  void setLabelColor(void);
  void setProbabilityColor(void);
  void setOrganColor(void);

  void toggleDraggers(void);
  void toggleRenderPlant(void);
  void toggleRenderPot(void);
  void toggleRenderNoise(void);
  void toggleRenderNormals(void);
  void toggleRenderOrientations(void);
  void toggleRenderStems(void);
  void toggleRenderLeaves(void);
  void toggleRenderTriangles(void);
  void toggleRenderOrgans(void);
  void toggleRegisterState(void);

  void loadStatus(void);
  void saveStatus(void);
  void save(void);

  void absoluteClassify(void);
  void absoluteDetectLeaves(void);

  void sampleSkeletonPoints(void);
  void initializeSkeleton(void);
  void extractStemSkeleton(void);
  void absoluteDetectStems(void);

  void printOrgans(void);

protected:
  virtual void clearData();
  virtual void updateImpl();

  PointCloud* getPrevFrame(void);
  PointCloud* getNextFrame(void);

  void loadTransformation(void);
  void saveTransformation(void);
  void deleteTransformation(void);

  osg::Vec4 getColor(size_t i) const;
  void getColors(osg::Vec4Array* colors, size_t start, size_t end) const;

  void visualizePoints(size_t start, size_t end);
  void visualizeTriangles(void);
  void visualizeOrgans(void);
  void visualizeStemGraph(void);

  void triangulate(void) const;
  void initPointGraph(double distance_threshold);
  void denoise(int segment_threshold, double triangle_length);

  void initGcoGraphEdges(GCoptimizationGeneralGraph* gco, int smooth_cost);
  void initGcoGraphEdgesStems(GCoptimizationGeneralGraph* gco, int smooth_cost, const std::vector<size_t>& reverse_indices);
  void initGcoGraphEdgesLeaves(GCoptimizationGeneralGraph* gco, int smooth_cost, const std::vector<size_t>& reverse_indices);

  void fillOrganPoints(void);

  void addOrgan(const std::vector<size_t>& stem_point_indices, bool leaf);
  std::vector<std::vector<size_t> > computeLeafComponents(void);
  std::vector<std::vector<size_t> > extractStemComponents(void);

  void collectStems(void);
  void collectLeaves(void);

  void extendStems(void);

  void updateOrganFeature(void);

  void initMinMaxCurvature(void);
  void initMinMaxThickness(void);

  void backup(std::vector<int>& labels, std::vector<int>& ids, std::vector<Organ>& stems, std::vector<Organ>& leaves);
  void rollback(std::vector<int>& labels, std::vector<int>& ids, std::vector<Organ>& stems, std::vector<Organ>& leaves);

protected:
  std::string                     filename_;
  size_t                          plant_points_num_;
  size_t                          pot_points_num_;
  size_t                          noise_points_num_;
  mutable CGAL::Delaunay*         triangulation_;
  ColorMode                       color_mode_;
  osg::Vec4                       color_;

  std::vector<Organ>              stems_;
  std::vector<Organ>              leaves_;
  boost::PointGraph*              point_graph_;

  KdTreePtr                       kdtree_;

private:
  osg::ref_ptr<osgManipulator::TranslateAxisDragger>  translate_dragger_;
  osg::ref_ptr<osgManipulator::TrackballDragger>      trackball_dragger_;

  QMutex                          mutex_;

  double                          point_graph_threshold_;
  float                           min_curvature_;
  float                           min_thickness_;
  float                           max_thickness_;

  bool                            show_draggers_;
  bool                            registered_;
  bool                            show_plant_;
  bool                            show_pot_;
  bool                            show_noise_;
  bool                            show_leaves_;
  bool                            show_stems_;
  bool                            show_normals_;
  bool                            show_orientations_;
  bool                            show_triangles_;
  bool                            show_organs_;
};

#endif // POINTCLOUD_H