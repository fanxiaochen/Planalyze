#pragma once
#ifndef FILE_SYSTEM_MODEL_H
#define FILE_SYSTEM_MODEL_H

#include <unordered_map>

#include <QSet>
#include <QHash>
#include <QMutex>
#include <QFileSystemModel>
#include <QPersistentModelIndex>

#include <osg/ref_ptr>
#include <osg/Vec4>

class PointCloud;
class MeshModel;

class FileSystemModel : public QFileSystemModel
{
  Q_OBJECT

public:
  FileSystemModel();
  virtual ~FileSystemModel();

  QModelIndex setRootPath ( const QString & newPath );

  Qt::ItemFlags flags(const QModelIndex &index) const;
  QVariant data(const QModelIndex &index, int role) const;
  bool setData(const QModelIndex &index, const QVariant &value, int role);
  bool isShown(const std::string& filename) const;

  osg::ref_ptr<MeshModel> getMeshModel(const std::string& filename);
  osg::ref_ptr<MeshModel> getMeshModel(int frame);
  void showMeshModel(int frame);
  void showMeshModel(const std::string& filename);
  void hideMeshModel(int frame);
  void hideMeshModel(const std::string& filename);

  osg::ref_ptr<PointCloud> getPointCloud(const std::string& filename);
  osg::ref_ptr<PointCloud> getPointCloud(int frame);
  osg::ref_ptr<PointCloud> getPointCloud(int frame, int view);
  void getFrameRange(int &start, int &end);
  std::string getPointsFolder(int frame);
  std::string getPointsFolder(int frame, int view);
  std::string getImagesFolder(int frame);
  std::string getImagesFolder(int frame, int view);

  std::string getPointsFilename(int frame, int view);
  std::string getPointsFilename(int frame);

  void showPointCloud(const std::string& filename);

  void hidePointCloud(int frame, int view);
  void hidePointCloud(const std::string& filename);

  void updatePointCloud(int frame, int view);
  void updatePointCloud(int frame);
  const osg::Vec4& getColor(void) const {return color_;}

  int getStartFrame(void) const {return start_frame_;}
  int getEndFrame(void) const {return end_frame_;}
  PointCloud* getDisplayFirstFrame(void);

  void removeBadFrame(void);

  enum NavigationType
  {
    SWITCH,
    APPEND,
    ERASE
  };
  void navigateToPreviousFrame(NavigationType type);
  void navigateToNextFrame(NavigationType type);
  void navigateToPreviousView(NavigationType type);
  void navigateToNextView(NavigationType type);

public slots:
  void setRenderPlant(bool render);
  void setRenderPot(bool render);
  void setRenderNoise(bool render);
  void setRenderNormals(bool render);
  void setRenderOrientations(bool render);
  void setRenderStems(bool render);
  void setRenderLeaves(bool render);
  void setRenderTriangles(bool render);
  void setRenderOrgans(bool render);
  void setColorMode(int color_mode);

  void showPointCloud(int frame, int view);
  void hideAndShowPointCloud(int hide_frame, int hide_view, int show_frame, int show_view);

  void forwardClassify(void);
  void backwardClassify(void);
  void fSmoothStems(void);
  void bSmoothStems(void);
  void fSmoothLeaves(void);
  void bSmoothLeaves(void);

  void absoluteClassify(void);
  void absoluteDetectLeaves(void);

signals:
  void progressValueChanged(int value);
  void timeToHideAndShowPointCloud(int hide_frame, int hide_view, int show_frame, int show_view);
  void timeToShowPointCloud(int show_frame, int show_view);

private:
  QSet<QPersistentModelIndex>     checked_indexes_;
  Qt::CheckState computeCheckState(const QModelIndex &index) const;
  bool checkRegisterState(const QModelIndex &index) const;
  typedef QHash<QPersistentModelIndex, osg::ref_ptr<PointCloud> > PointCloudMap;
  PointCloudMap  point_cloud_map_;
  void showPointCloud(const QPersistentModelIndex& index);
  void hidePointCloud(const QPersistentModelIndex& index);
  void showPointCloudSceneInformation(void) const;
  typedef std::unordered_map<std::string, osg::ref_ptr<PointCloud> > PointCloudCacheMap;
  PointCloudCacheMap point_cloud_cache_map_;
  void limitPointCloudCacheSize(void);

  typedef QHash<QPersistentModelIndex, osg::ref_ptr<MeshModel> > MeshModelMap;
  MeshModelMap  mesh_model_map_;
  void showMeshModel(const QPersistentModelIndex& index);
  void hideMeshModel(const QPersistentModelIndex& index);

  void getDisplayFirstFrameFirstView(int& frame, int& view);
  void getDisplayFirstFrameLastView(int& frame, int& view);
  void getDisplayLastFrameLastView(int& frame, int& view);

  std::string getMeshModelFilename(int frame);

  void forwardClassify(double smooth_cost, int start_frame, int end_frame);
  void backwardClassify(double smooth_cost, int start_frame, int end_frame);
  void fSmoothStems(double smooth_cost, int start_frame, int end_frame);
  void bSmoothStems(double smooth_cost, int start_frame, int end_frame);
  void fSmoothLeaves(double smooth_cost, int start_frame, int end_frame);
  void bSmoothLeaves(double smooth_cost, int start_frame, int end_frame);

private:
  bool recursiveCheck(const QModelIndex &index, const QVariant &value);
  void computeFrameRange(void);

  int                           start_frame_;
  int                           end_frame_;
  osg::Vec4                     color_;
  QMutex                        mutex_;
};
#endif // FILE_SYSTEM_MODEL_H
