#include <QDir>
#include <QColor>
#include <QMutexLocker>
#include <QColorDialog>
#include <QFutureWatcher>
#include <QtConcurrentRun>

#include <osg/Group>
#include <osgDB/ReadFile>

#include "color_map.h"
#include "mesh_model.h"
#include "information.h"
#include "main_window.h"
#include "point_cloud.h"
#include "progress_bar.h"
#include "parameter_manager.h"
#include "osg_viewer_widget.h"
#include "file_system_model.h"


FileSystemModel::FileSystemModel()
  :start_frame_(-1),
  end_frame_(-1),
  color_(ColorMap::Instance().getColor(ColorMap::LIGHT_BLUE))
{
  setNameFilterDisables(false);
  QStringList allowed_file_extensions;
  allowed_file_extensions.push_back("*.pcd");
  allowed_file_extensions.push_back("*.obj");
  setNameFilters(allowed_file_extensions);

  connect(this, SIGNAL(timeToHideAndShowPointCloud(int, int, int, int)), this, SLOT(hideAndShowPointCloud(int, int, int, int)));
  connect(this, SIGNAL(timeToShowPointCloud(int, int)), this, SLOT(showPointCloud(int, int)));
}

FileSystemModel::~FileSystemModel()
{
}

Qt::ItemFlags FileSystemModel::flags(const QModelIndex &index) const
{
  return QFileSystemModel::flags(index) | Qt::ItemIsUserCheckable;
}

QVariant FileSystemModel::data(const QModelIndex &index, int role) const
{
  if(role == Qt::CheckStateRole)
    return computeCheckState(index);
  else
  {
    if(role == Qt::ForegroundRole && checkRegisterState(index))
      return QBrush(QColor(255, 0, 0));
    return QFileSystemModel::data(index, role);
  }
}

bool FileSystemModel::checkRegisterState(const QModelIndex &index) const
{
  PointCloudCacheMap::const_iterator it = point_cloud_cache_map_.find(filePath(index).toStdString());
  if (it != point_cloud_cache_map_.end())
    return it->second.get()->isRegistered();
  else
    return false;
}

Qt::CheckState FileSystemModel::computeCheckState(const QModelIndex &index) const
{
  if(!hasChildren(index))
    return (checked_indexes_.contains(index)) ? (Qt::Checked) : (Qt::Unchecked);

  bool all_checked = true;
  bool all_unchecked = true;
  for(int i = 0, i_end = rowCount(index); i < i_end; i ++)
  {
    QModelIndex child = QFileSystemModel::index(i, 0, index);
    Qt::CheckState check_state = computeCheckState(child);
    if (check_state == Qt::PartiallyChecked)
      return check_state;

    if (check_state == Qt::Checked)
      all_unchecked = false;
    if (check_state == Qt::Unchecked)
      all_checked = false;

    if (!all_checked && !all_unchecked)
      return Qt::PartiallyChecked;
  }

  if (all_unchecked)
    return Qt::Unchecked;

  return Qt::Checked;
}

bool FileSystemModel::isShown(const std::string& filename) const
{
  QModelIndex index = this->index(filename.c_str());
  return (checked_indexes_.contains(index)) ? (true) : (false);
}

bool FileSystemModel::setData(const QModelIndex &index, const QVariant &value, int role)
{
  bool is_point_cloud = (filePath(index).right(3) == "pcd");
  if(role == Qt::CheckStateRole)
  {
    if (is_point_cloud)
    {
      if(value == Qt::Checked)
        showPointCloud(index);
      else
        hidePointCloud(index);
    }
    else
    {
      if(value == Qt::Checked)
        showMeshModel(index);
      else
        hideMeshModel(index);
    }


    if(hasChildren(index) == true)
      recursiveCheck(index, value);

    emit dataChanged(index, index);
    return true;
  }

  return QFileSystemModel::setData(index, value, role);
}

void FileSystemModel::showMeshModel(const QPersistentModelIndex& index)
{
  checked_indexes_.insert(index);

  osg::ref_ptr<MeshModel> mesh_model = getMeshModel(filePath(index).toStdString());
  if (!mesh_model.valid())
    return;

  MeshModelMap::iterator mesh_model_map_it = mesh_model_map_.find(index);
  if (mesh_model_map_it != mesh_model_map_.end())
    return;

  MainWindow::getInstance()->getOSGViewerWidget()->addChild(mesh_model);
  mesh_model_map_[index] = mesh_model;

  return;
}

void FileSystemModel::hideMeshModel(const QPersistentModelIndex& index)
{
  checked_indexes_.remove(index);

  MeshModelMap::iterator mesh_model_map_it = mesh_model_map_.find(index);
  if (mesh_model_map_it == mesh_model_map_.end())
    return;

  MainWindow::getInstance()->getOSGViewerWidget()->removeChild(mesh_model_map_it.value().get());
  mesh_model_map_.erase(mesh_model_map_it);

  return;
}

osg::ref_ptr<MeshModel> FileSystemModel::getMeshModel(const std::string& filename)
{
  QMutexLocker locker(&mutex_);

  QFileInfo fileinfo(filename.c_str());
  if (!fileinfo.exists() || !fileinfo.isFile())
    return NULL;

  osg::ref_ptr<MeshModel> mesh_model(new MeshModel());
  if (!mesh_model->open(filename))
    return NULL;

  return mesh_model;
}

osg::ref_ptr<MeshModel> FileSystemModel::getMeshModel(int frame)
{
  return getMeshModel(getMeshModelFilename(frame));
}

void FileSystemModel::showMeshModel(int frame)
{
  showMeshModel(getMeshModelFilename(frame));

  return;
}

void FileSystemModel::showMeshModel(const std::string& filename)
{
  QModelIndex index = this->index(QString(filename.c_str()));
  if (!index.isValid())
    return;

  showMeshModel(index);

  return;
}

void FileSystemModel::hideMeshModel(int frame)
{
  hideMeshModel(getMeshModelFilename(frame));
}

void FileSystemModel::hideMeshModel(const std::string& filename)
{
  QModelIndex index = this->index(QString(filename.c_str()));
  if (!index.isValid())
    return;

  hideMeshModel(index);
}

void FileSystemModel::limitPointCloudCacheSize(void)
{
  size_t threshold = 32;

  if (point_cloud_cache_map_.size() <= threshold)
    return;

  std::set<osg::ref_ptr<PointCloud> > in_use_clouds;
  std::vector<osg::ref_ptr<PointCloud> > freeable_clouds;

  for (PointCloudMap::const_iterator it = point_cloud_map_.begin(); it != point_cloud_map_.end(); ++ it)
    in_use_clouds.insert(*it);

  for (PointCloudCacheMap::const_iterator it = point_cloud_cache_map_.begin(); it != point_cloud_cache_map_.end(); ++ it)
    if (in_use_clouds.find(it->second) == in_use_clouds.end() && it->second->referenceCount() == 1)
      freeable_clouds.push_back(it->second);

  for (size_t i = 0, i_end = freeable_clouds.size(); i < i_end; ++ i)
    point_cloud_cache_map_.erase(freeable_clouds[i]->getFilename());

  return;
}

osg::ref_ptr<PointCloud> FileSystemModel::getPointCloud(const std::string& filename)
{
  QMutexLocker locker(&mutex_);

  limitPointCloudCacheSize();

  QFileInfo fileinfo(filename.c_str());
  if (!fileinfo.exists() || !fileinfo.isFile())
    return NULL;

  PointCloudCacheMap::iterator it = point_cloud_cache_map_.find(filename);
  if (it != point_cloud_cache_map_.end())
    return it->second.get();

  osg::ref_ptr<PointCloud> point_cloud(new PointCloud());
  if (!point_cloud->open(filename))
    return NULL;

  if (point_cloud->thread() != qApp->thread())
    point_cloud->moveToThread(qApp->thread());

  point_cloud_cache_map_[filename] = point_cloud;

  return point_cloud;
}

QModelIndex FileSystemModel::setRootPath ( const QString & newPath )
{
  point_cloud_cache_map_.clear();
  point_cloud_map_.clear();
  checked_indexes_.clear();

  QModelIndex index = QFileSystemModel::setRootPath(newPath);
  computeFrameRange();
  if (start_frame_ != -1)
  {
    if (getPointCloud(start_frame_) != NULL)
      showPointCloud(start_frame_, 12);
    else if (getPointCloud(start_frame_, 0) != NULL)
      showPointCloud(start_frame_, 0);
    else
    {
      osg::ref_ptr<MeshModel> mesh_model = getMeshModel(start_frame_);
      if (mesh_model.valid())
        showMeshModel(start_frame_);
    }

    MainWindow::getInstance()->getOSGViewerWidget()->centerScene();
  }

  return index;
}

static void extractStartEndFrame(const QStringList& entries, int& start_frame, int& end_frame)
{
  start_frame = std::numeric_limits<int>::max();
  end_frame = std::numeric_limits<int>::min();

  for (QStringList::const_iterator entries_it = entries.begin();
    entries_it != entries.end(); ++ entries_it)
  {
    if (!entries_it->contains("frame_"))
      continue;

    int index = entries_it->right(4).toInt();
    if (start_frame > index)
      start_frame = index;
    if (end_frame < index)
      end_frame = index;
  }

  return;
}

void FileSystemModel::computeFrameRange(void)
{
  start_frame_ = end_frame_ = -1;

  QString root_path = rootPath();
  QModelIndex root_index = index(root_path);

  if (root_path.contains("frame_")) {
    start_frame_ = end_frame_ = root_path.right(4).toInt();
    return;
  }

  if (root_path.compare("points") == 0)
  {
    QStringList points_entries = QDir(root_path).entryList();
    extractStartEndFrame(points_entries, start_frame_, end_frame_);
    return;
  }

  QStringList root_entries = QDir(root_path).entryList();
  for (QStringList::const_iterator root_entries_it = root_entries.begin();
    root_entries_it != root_entries.end(); ++ root_entries_it)
  {
    if (root_entries_it->compare("points") != 0)
      continue;

    QStringList points_entries = QDir(root_path+"/"+*root_entries_it).entryList();
    extractStartEndFrame(points_entries, start_frame_, end_frame_);
    return;
  }

  return;
}

void FileSystemModel::getFrameRange(int &start, int &end)
{
  start = start_frame_;
  end = end_frame_;
}

osg::ref_ptr<PointCloud> FileSystemModel::getPointCloud(int frame)
{
  return getPointCloud(getPointsFilename(frame));
}

std::string FileSystemModel::getImagesFolder(int frame)
{
  std::string points_folder = getPointsFolder(frame);
  if (points_folder.empty())
    return points_folder;

  QString image_folder = QString(points_folder.c_str()).replace("/points/", "/images/");
  if (!QDir(image_folder).exists())
    return std::string();

  return image_folder.toStdString();
}

std::string FileSystemModel::getImagesFolder(int frame, int view)
{
  std::string frame_folder = getImagesFolder(frame);
  if (frame_folder.empty())
    return frame_folder;

  QString view_folder = QString("%1/slice_%2").arg(frame_folder.c_str()).arg(view, 2, 10, QChar('0'));
  if (!QDir(view_folder).exists())
    view_folder = QString("%1/view_%2").arg(frame_folder.c_str()).arg(view, 2, 10, QChar('0'));

  return view_folder.toStdString();
}

std::string FileSystemModel::getPointsFolder(int frame)
{
  QModelIndex root_index = index(rootPath());

  int start, end;
  getFrameRange(start, end);
  if (start < 0 || end < 0)
    return std::string();

  std::string folder;

  QString root_path = rootPath();
  if (root_path.contains("frame_"))
  {
    folder =  root_path.toStdString();
  }
  else if (root_path.compare("points") == 0)
  {
    QModelIndex frame_index = index(frame-start, 0, root_index);
    folder = filePath(frame_index).toStdString();
  }
  else
  {
    QStringList root_entries = QDir(root_path).entryList();
    for (QStringList::const_iterator root_entries_it = root_entries.begin();
      root_entries_it != root_entries.end(); ++ root_entries_it)
    {
      if (root_entries_it->compare("points") != 0)
        continue;

      folder = (root_path+QString("/%1/frame_%2").arg(*root_entries_it).arg(frame, 5, 10, QChar('0'))).toStdString();
      break;
    }
  }

  return folder;
}

std::string FileSystemModel::getPointsFolder(int frame, int view)
{
  std::string frame_folder = getPointsFolder(frame);
  if (frame_folder.empty() || view < 0 || view >= 12)
    return frame_folder;

  QString view_folder = QString("%1/view_%2").arg(frame_folder.c_str()).arg(view, 2, 10, QChar('0'));
  if (!QDir(view_folder).exists())
    view_folder = QString("%1/slice_%2").arg(frame_folder.c_str()).arg(view, 2, 10, QChar('0'));

  return view_folder.toStdString();
}

std::string FileSystemModel::getPointsFilename(int frame, int view)
{
  std::string folder = getPointsFolder(frame, view);
  if (folder.empty())
    return folder;

  return folder+"/points.pcd";
}

std::string FileSystemModel::getPointsFilename(int frame)
{
  return getPointsFilename(frame, 12);
}

std::string FileSystemModel::getMeshModelFilename(int frame)
{
  std::string folder = getPointsFolder(frame);
  if (folder.empty())
    return folder;

  QStringList mesh_model_entries = QDir(folder.c_str()).entryList();
  for (QStringList::iterator it = mesh_model_entries.begin();
    it != mesh_model_entries.end(); ++ it)
  {
    if (it->right(3) == "obj")
      return folder+"/"+it->toStdString();
  }
  return std::string();
}

osg::ref_ptr<PointCloud> FileSystemModel::getPointCloud(int frame, int view)
{
  return getPointCloud(getPointsFilename(frame, view));
}

void FileSystemModel::showPointCloud(int frame, int view)
{
  showPointCloud(getPointsFilename(frame, view));
}

void FileSystemModel::showPointCloud(const std::string& filename)
{
  QModelIndex index = this->index(QString(filename.c_str()));
  if (!index.isValid())
    return;

  showPointCloud(index);

  return;
}

void FileSystemModel::showPointCloud(const QPersistentModelIndex& index)
{
  checked_indexes_.insert(index);

  osg::ref_ptr<PointCloud> point_cloud(getPointCloud(filePath(index).toStdString()));
  if (!point_cloud.valid())
    return;

  PointCloudMap::iterator point_cloud_map_it = point_cloud_map_.find(index);
  if (point_cloud_map_it != point_cloud_map_.end())
    return;

  MainWindow::getInstance()->getOSGViewerWidget()->addChild(point_cloud);
  point_cloud_map_[index] = point_cloud;

  showPointCloudSceneInformation();
  return;
}

void FileSystemModel::hidePointCloud(const std::string& filename)
{
  QModelIndex index = this->index(QString(filename.c_str()));
  if (!index.isValid())
    return;

  hidePointCloud(index);
}

void FileSystemModel::hidePointCloud(int frame, int view)
{
  hidePointCloud(getPointsFilename(frame, view));
}

void FileSystemModel::hidePointCloud(const QPersistentModelIndex& index)
{
  checked_indexes_.remove(index);

  PointCloudMap::iterator point_cloud_map_it = point_cloud_map_.find(index);
  if (point_cloud_map_it == point_cloud_map_.end())
    return;

  MainWindow::getInstance()->getOSGViewerWidget()->removeChild(point_cloud_map_it.value().get());
  point_cloud_map_.erase(point_cloud_map_it);

  showPointCloudSceneInformation();

  return;
}

void FileSystemModel::hideAndShowPointCloud(int hide_frame, int hide_view, int show_frame, int show_view)
{
  bool to_hide = true;
  bool to_show = true;

  osg::ref_ptr<PointCloud> show_cloud = getPointCloud(show_frame, show_view);
  osg::ref_ptr<PointCloud> hide_cloud = getPointCloud(hide_frame, hide_view);

  QModelIndex show_index = this->index(QString(getPointsFilename(show_frame, show_view).c_str()));
  checked_indexes_.insert(show_index);
  PointCloudMap::iterator point_cloud_map_it = point_cloud_map_.find(show_index);
  if (point_cloud_map_it == point_cloud_map_.end())
  {
    if (show_cloud != NULL)
      point_cloud_map_[show_index] = show_cloud;
    else
      to_show = false;
  }
  else
    to_show = false;

  QModelIndex hide_index = this->index(QString(getPointsFilename(hide_frame, hide_view).c_str()));
  checked_indexes_.remove(hide_index);
  point_cloud_map_it = point_cloud_map_.find(hide_index);
  if (point_cloud_map_it != point_cloud_map_.end())
    point_cloud_map_.erase(point_cloud_map_it);
  else
    to_hide = false;

  OSGViewerWidget* osg_viewer_widget = MainWindow::getInstance()->getOSGViewerWidget();
  if (to_hide && to_show)
    osg_viewer_widget->replaceChild(hide_cloud, show_cloud, true);
  else if (to_hide)
    osg_viewer_widget->removeChild(getPointCloud(hide_frame, hide_view), true);
  else if (to_show)
    osg_viewer_widget->addChild(getPointCloud(show_frame, show_view), true);

  showPointCloudSceneInformation();

  return;
}

void FileSystemModel::showPointCloudSceneInformation(void)
{
  QString information("Displaying Point Cloud:\n");

  if (point_cloud_map_.empty())
  {
    MainWindow::getInstance()->getInformation()->setText(information.toStdString(), 20, 20);
    return;
  }

  std::vector<std::pair<int, int> > sorted_scene_info;
  for (PointCloudMap::const_iterator it = point_cloud_map_.begin(); it != point_cloud_map_.end(); ++ it)
  {
    int frame = (*it)->getFrame();
    int view = (*it)->getView();
    sorted_scene_info.push_back(std::make_pair(frame, view));
  }

  std::sort(sorted_scene_info.begin(), sorted_scene_info.end());
  for (size_t i = 0, i_end = sorted_scene_info.size(); i < i_end; ++ i)
  {
    int frame = sorted_scene_info[i].first;
    int view = sorted_scene_info[i].second;
    if (view < 12)
      information += QString("Frame %1 View %2\n").arg(frame, 5, 10, QChar('0')).arg(view, 2, 10, QChar('0'));
    else
    {
      osg::ref_ptr<PointCloud> point_cloud = getPointCloud(frame);
      information += QString("Frame %1 StemNum:%2 LeafNum:%3\n").arg(frame, 5, 10, QChar('0'))
        .arg(point_cloud->getStemNum()).arg(point_cloud->getLeafNum());
    }
  }
  MainWindow::getInstance()->getInformation()->setText(information.toStdString(), 20, 20);

  return;
}

PointCloud* FileSystemModel::getDisplayFirstFrame(void)
{
  if (point_cloud_map_.empty())
    return NULL;

  osg::ref_ptr<PointCloud> first_cloud = *point_cloud_map_.begin();
  int frame = first_cloud->getFrame();
  int view = first_cloud->getView();

  for (PointCloudMap::const_iterator it = point_cloud_map_.begin(); it != point_cloud_map_.end(); ++ it)
  {
    osg::ref_ptr<PointCloud> point_cloud = *it;
    int this_frame = point_cloud->getFrame();
    int this_view = point_cloud->getView();
    if (this_frame < frame)
    {
      frame = this_frame;
      view = this_view;
    }
    else if(this_frame == frame && this_view > view)
    {
      frame = this_frame;
      view = this_view;
    }
  }

  if (view != 12)
    return NULL;

  return getPointCloud(frame);
}

void FileSystemModel::getDisplayFirstFrameFirstView(int& frame, int& view)
{
  if (point_cloud_map_.empty())
  {
    frame = -1;
    view = -1;
    return;
  }

  osg::ref_ptr<PointCloud> first_cloud = *point_cloud_map_.begin();
  frame = first_cloud->getFrame();
  view = first_cloud->getView();

  for (PointCloudMap::const_iterator it = point_cloud_map_.begin(); it != point_cloud_map_.end(); ++ it)
  {
    osg::ref_ptr<PointCloud> point_cloud = *it;
    int this_frame = point_cloud->getFrame();
    int this_view = point_cloud->getView();
    if (this_frame < frame)
    {
      frame = this_frame;
      view = this_view;
    }
    else if(this_frame == frame && this_view < view)
    {
      frame = this_frame;
      view = this_view;
    }
  }
  return;
}

void FileSystemModel::getDisplayFirstFrameLastView(int& frame, int& view)
{
  if (point_cloud_map_.empty())
  {
    frame = -1;
    view = -1;
    return;
  }

  std::vector<std::pair<int, int> > display_items;
  for (PointCloudMap::const_iterator it = point_cloud_map_.begin(); it != point_cloud_map_.end(); ++ it)
  {
    osg::ref_ptr<PointCloud> point_cloud = *it;
    display_items.push_back(std::make_pair(point_cloud->getFrame(), point_cloud->getView()));
  }

  frame = display_items[0].first;
  view = display_items[0].second;
  for (size_t i = 1, i_end = display_items.size(); i < i_end; ++ i)
  {
    int this_frame = display_items[i].first;
    if (this_frame < frame)
      frame = this_frame;
  }
  for (size_t i = 1, i_end = display_items.size(); i < i_end; ++ i)
  {
    int this_frame = display_items[i].first;
    int this_view = display_items[i].second;
    if (this_frame == frame && this_view > view)
      view = this_view;
  }
  return;
}

void FileSystemModel::getDisplayLastFrameLastView(int& frame, int& view)
{
  if (point_cloud_map_.empty())
  {
    frame = -1;
    view = -1;
    return;
  }

  osg::ref_ptr<PointCloud> first_cloud = *point_cloud_map_.begin();
  frame = first_cloud->getFrame();
  view = first_cloud->getView();

  for (PointCloudMap::const_iterator it = point_cloud_map_.begin(); it != point_cloud_map_.end(); ++ it)
  {
    osg::ref_ptr<PointCloud> point_cloud = *it;
    int this_frame = point_cloud->getFrame();
    int this_view = point_cloud->getView();
    if (this_frame > frame)
    {
      frame = this_frame;
      view = this_view;
    }
    else if(this_frame == frame && this_view > view)
    {
      frame = this_frame;
      view = this_view;
    }
  }
  return;
}

bool FileSystemModel::recursiveCheck(const QModelIndex &index, const QVariant &value)
{
  if(!hasChildren(index))
    return false;

  for(int i = 0, i_end = rowCount(index); i < i_end; i ++)
  {
    QModelIndex child = QFileSystemModel::index(i, 0, index);
    setData(child, value, Qt::CheckStateRole);
  }

  return true;
}

void FileSystemModel::updatePointCloud(int frame, int view)
{
  std::string filename = getPointsFilename(frame, view);
  if (point_cloud_cache_map_.find(filename) == point_cloud_cache_map_.end())
    return;

  getPointCloud(frame, view)->reload();

  return;
}

void FileSystemModel::updatePointCloud(int frame)
{
  std::string filename = getPointsFilename(frame);
  if (point_cloud_cache_map_.find(filename) == point_cloud_cache_map_.end())
    return;

  getPointCloud(frame)->reload();

  return;
}

void FileSystemModel::setRenderPlant(bool render)
{
  for (PointCloudCacheMap::iterator it = point_cloud_cache_map_.begin(); it != point_cloud_cache_map_.end(); ++ it)
    it->second->setRenderPlant(render);

  return;
}

void FileSystemModel::setRenderPot(bool render)
{
  for (PointCloudCacheMap::iterator it = point_cloud_cache_map_.begin(); it != point_cloud_cache_map_.end(); ++ it)
    it->second->setRenderPot(render);

  return;
}

void FileSystemModel::setRenderNoise(bool render)
{
  for (PointCloudCacheMap::iterator it = point_cloud_cache_map_.begin(); it != point_cloud_cache_map_.end(); ++ it)
    it->second->setRenderNoise(render);

  return;
}

void FileSystemModel::setRenderNormals(bool render)
{
  for (PointCloudCacheMap::iterator it = point_cloud_cache_map_.begin(); it != point_cloud_cache_map_.end(); ++ it)
    it->second->setRenderNormals(render);

  return;
}

void FileSystemModel::setRenderOrientations(bool render)
{
  for (PointCloudCacheMap::iterator it = point_cloud_cache_map_.begin(); it != point_cloud_cache_map_.end(); ++ it)
    it->second->setRenderOrientations(render);

  return;
}

void FileSystemModel::setRenderLeaves(bool render)
{
  for (PointCloudCacheMap::iterator it = point_cloud_cache_map_.begin(); it != point_cloud_cache_map_.end(); ++ it)
    it->second->setRenderLeaves(render);

  return;
}


void FileSystemModel::setRenderStems(bool render)
{
  for (PointCloudCacheMap::iterator it = point_cloud_cache_map_.begin(); it != point_cloud_cache_map_.end(); ++ it)
    it->second->setRenderStems(render);

  return;
}

void FileSystemModel::setRenderTriangles(bool render)
{
  for (PointCloudCacheMap::iterator it = point_cloud_cache_map_.begin(); it != point_cloud_cache_map_.end(); ++ it)
    it->second->setRenderTriangles(render);

  return;
}

void FileSystemModel::setRenderOrgans(bool render)
{
  for (PointCloudCacheMap::iterator it = point_cloud_cache_map_.begin(); it != point_cloud_cache_map_.end(); ++ it)
    it->second->setRenderOrgans(render);

  return;
}

void FileSystemModel::setColorMode(int color_mode)
{
  if (color_mode == PointCloud::UNIFORM)
  {
    QColor initial(color_.r()*255, color_.g()*255, color_.b()*255, color_.a()*255);
    QColor color = QColorDialog::getColor(initial, MainWindow::getInstance(),
      "Select Color for the Point Cloud", QColorDialog::ShowAlphaChannel);

    if (!color.isValid())
      return;

    color_ = osg::Vec4(color.red()/255.0, color.green()/255.0, color.blue()/255.0, color.alpha()/255.0);

    for (PointCloudCacheMap::iterator it = point_cloud_cache_map_.begin(); it != point_cloud_cache_map_.end(); ++ it)
      it->second->setUniformColor(color);
    return;
  }

  for (PointCloudCacheMap::iterator it = point_cloud_cache_map_.begin(); it != point_cloud_cache_map_.end(); ++ it)
    it->second->setColorMode((PointCloud::ColorMode)(color_mode));

  return;
}

void FileSystemModel::navigateToPreviousFrame(NavigationType type)
{
  int first_frame, first_view;
  getDisplayLastFrameLastView(first_frame, first_view);

  if (first_frame == -1 || first_view == -1)
  {
    showPointCloud(getStartFrame(), 12);
    return;
  }

  if (type == ERASE)
  {
    hidePointCloud(first_frame, first_view);
    return;
  }

  int current_frame = first_frame-1;
  if (current_frame < getStartFrame())
    return;

  if (type == APPEND)
    showPointCloud(current_frame, first_view);
  else
    hideAndShowPointCloud(first_frame, first_view, current_frame, first_view);

  return;
}

void FileSystemModel::navigateToNextFrame(NavigationType type)
{
  int last_frame, last_view;
  getDisplayFirstFrameLastView(last_frame, last_view);

  if (last_frame == -1 || last_view == -1)
  {
    showPointCloud(getEndFrame(), 12);
    return;
  }

  if (type == ERASE)
  {
    hidePointCloud(last_frame, last_view);
    return;
  }

  int current_frame = last_frame+1;
  if (current_frame > getEndFrame())
    return;

  if (type == APPEND)
    showPointCloud(current_frame, last_view);
  else
    hideAndShowPointCloud(last_frame, last_view, current_frame, last_view);

  return;
}

void FileSystemModel::navigateToPreviousView(NavigationType type)
{
  int first_frame, first_view;
  getDisplayFirstFrameFirstView(first_frame, first_view);

  if (first_frame == -1 || first_view == -1)
  {
    showPointCloud(getStartFrame(), 0);
    return;
  }

  if (type == ERASE)
  {
    hidePointCloud(first_frame, first_view);
    return;
  }

  int current_view = first_view-1;
  if (current_view < 0)
    return;

  if (type == APPEND)
    showPointCloud(first_frame, current_view);
  else
    hideAndShowPointCloud(first_frame, first_view, first_frame, current_view);

  return;
}

void FileSystemModel::navigateToNextView(NavigationType type)
{
  int first_frame, last_view;
  getDisplayFirstFrameLastView(first_frame, last_view);

  if (first_frame == -1 || last_view == -1)
  {
    showPointCloud(getStartFrame(), 11);
    return;
  }

  if (type == ERASE)
  {
    hidePointCloud(first_frame, last_view);
    return;
  }

  int current_view = last_view+1;
  if (current_view > 12)
    return;

  if (type == APPEND)
    showPointCloud(first_frame, current_view);
  else
    hideAndShowPointCloud(first_frame, last_view, first_frame, current_view);

  return;
}


void FileSystemModel::removeBadFrame(void)
{
  /*FileSystemModel* model = MainWindow::getInstance()->getFileSystemModel();

  const size_t delta = 80;

  for(size_t k = 1; k < 12; k++)
  {
  for(size_t i = 1680; i <= end_frame_; i = i + delta)
  {
  osg::ref_ptr<PointCloud> this_cloud = model->getPointCloud(i, k);
  this_cloud->update();
  double radius = this_cloud->getBound().radius();

  size_t j = i;
  while(radius > 350 && j < end_frame_)
  {
  this_cloud = model->getPointCloud(++j, k);
  if(this_cloud == NULL)
  {
  radius = 10000;
  continue;
  }
  else
  {
  this_cloud->update();
  radius = this_cloud->getBound().radius();
  }

  std::cout<<"ffff"<<std::endl;
  std::cout<<j<<std::endl;
  }

  std::cout<<"Frame:"<<i<<" View:"<<k<<std::endl;

  std::string this_file = model->getPointsFilename(i, k);
  std::string next_file = model->getPointsFilename(j, k);

  if((j != i && j != end_frame_)||(j == end_frame_ && radius <= 350))
  {
  QFile::remove(QString::fromUtf8(this_file.c_str()));
  QFile::copy(QString::fromUtf8(next_file.c_str()), QString::fromUtf8(this_file.c_str()));
  }   

  }
  }

  return;*/
  OSGViewerWidget* viewer = MainWindow::getInstance()->getOSGViewerWidget();

  std::fstream fout_modify, fout_temp;
  std::string file_modify = rootPath().toStdString()+"/ModifiedFrameInfo.txt";
  std::string file_temp = rootPath().toStdString()+"/AdjacentFrameInfo.txt";
  fout_modify.open(file_modify,std::ios::out|std::ios::app);
  fout_temp.open(file_temp,std::ios::out|std::ios::app);

  for(size_t j = 0, j_end = 12; j < j_end; j++)
  {
    std::string this_file, prev_file, next_file;
    osg::ref_ptr<PointCloud> this_cloud, prev_cloud, next_cloud;
    int this_num, prev_num, next_num;

    prev_file = getPointsFilename(start_frame_, j);
    prev_cloud = getPointCloud(start_frame_, j);
    prev_num = prev_cloud->size();

    this_file = getPointsFilename(start_frame_+1, j);
    this_cloud = getPointCloud(start_frame_+1,j);
    this_num = this_cloud->size();

    for(size_t i = start_frame_+1, i_end = end_frame_; i <= i_end; i++)
    {  
      if(i_end == i)
      {
        double this_ratio = (double)(prev_num - this_num)/prev_num;
        if(this_ratio > 0.05)
        {
          /*QFile::remove(QString::fromUtf8(this_file.c_str()));
          QFile::copy(QString::fromUtf8(prev_file.c_str()), QString::fromUtf8(this_file.c_str()));
          this_cloud->reload();*/
          fout_modify<<"Frame:"<<i<<" "<<"View:"<<j<<" "<<"ratio:"<<this_ratio<<std::endl;
        }

        fout_temp<<"Frame:"<<i<<" "<<"View:"<<j<<" "<<prev_num<<" "<<this_num<<" "<<
          this_ratio<<std::endl;
        fout_temp<<std::endl;

        continue;
      }

      next_file = getPointsFilename(i+1, j);
      QFile next_qfile(QString::fromUtf8(next_file.c_str()));
      bool flag = next_qfile.exists();
      if(!flag)
      {
        QFile::copy(QString::fromUtf8(this_file.c_str()), QString::fromUtf8(next_file.c_str())); 
        next_cloud = getPointCloud(i+1,j);
        next_num = 0;
      }
      else
      {
        next_cloud = getPointCloud(i+1,j);
        next_num = next_cloud->size();
      }



      double this_ratio = (double)(prev_num - this_num)/prev_num;
      double next_ratio;
      if(!flag)
        next_ratio = 1;
      else
        next_ratio= (double)(next_num - this_num)/next_num;

      if(this_ratio > 0.05 && next_ratio > 0.02)
      {
        /*QFile::remove(QString::fromUtf8(this_file.c_str()));
        QFile::copy(QString::fromUtf8(prev_file.c_str()), QString::fromUtf8(this_file.c_str()));
        this_cloud->reload();*/
        fout_modify<<"Frame:"<<i<<" "<<"View:"<<j<<" "<<"ratio:"<<this_ratio<<std::endl;
      }

      fout_temp<<"Frame:"<<i<<" "<<"View:"<<j<<" "<<prev_num<<" "<<this_num<<" "<<
        this_ratio<<std::endl;
      fout_temp<<std::endl;

      prev_file = this_file;
      prev_cloud = this_cloud;
      prev_num = this_num;

      this_file = next_file;
      this_cloud = next_cloud;
      this_num = next_num;

    }
  }

  fout_modify.close();
  fout_temp.close();

  return;

}

void FileSystemModel::forwardClassify(double smooth_cost, int start_frame, int end_frame)
{
  for (int forward_frame = start_frame; forward_frame <= end_frame; ++ forward_frame)
  {
    osg::ref_ptr<PointCloud> forward_frame_current = getPointCloud(forward_frame);
    forward_frame_current->classifyLeafStem(smooth_cost, true);
    forward_frame_current->smoothLeaves(smooth_cost, true);
    forward_frame_current->smoothStems(smooth_cost, true);
    forward_frame_current->printOrgans();

    timeToHideAndShowPointCloud(forward_frame-1, 12, forward_frame, 12);
    emit progressValueChanged(forward_frame-start_frame+1);
  }

  return;
}

void FileSystemModel::backwardClassify(double smooth_cost, int start_frame, int end_frame)
{
  for (int backward_frame = start_frame; backward_frame >= end_frame; -- backward_frame)
  {
    osg::ref_ptr<PointCloud> backward_frame_current = getPointCloud(backward_frame);
    backward_frame_current->classifyLeafStem(smooth_cost, false);
    backward_frame_current->smoothLeaves(smooth_cost, false);
    backward_frame_current->smoothStems(smooth_cost, false);
    backward_frame_current->printOrgans();

    timeToHideAndShowPointCloud(backward_frame+1, 12, backward_frame, 12);
    emit progressValueChanged(backward_frame-end_frame+1);
  }

  return;
}

void FileSystemModel::fSmoothStems(double smooth_cost, int start_frame, int end_frame)
{
  for (int forward_frame = start_frame; forward_frame <= end_frame; ++ forward_frame)
  {
    osg::ref_ptr<PointCloud> forward_frame_current = getPointCloud(forward_frame);
    forward_frame_current->smoothStems(smooth_cost, true);
    forward_frame_current->printOrgans();

    timeToHideAndShowPointCloud(forward_frame-1, 12, forward_frame, 12);
    emit progressValueChanged(forward_frame-start_frame+1);
  }

  return;
}

void FileSystemModel::bSmoothStems(double smooth_cost, int start_frame, int end_frame)
{
  for (int backward_frame = start_frame; backward_frame >= end_frame; -- backward_frame)
  {
    osg::ref_ptr<PointCloud> backward_frame_current = getPointCloud(backward_frame);
    backward_frame_current->smoothStems(smooth_cost, false);
    backward_frame_current->printOrgans();

    timeToHideAndShowPointCloud(backward_frame+1, 12, backward_frame, 12);
    emit progressValueChanged(backward_frame-end_frame+1);
  }

  return;
}


void FileSystemModel::fSmoothLeaves(double smooth_cost, int start_frame, int end_frame)
{
  for (int forward_frame = start_frame; forward_frame <= end_frame; ++ forward_frame)
  {
    osg::ref_ptr<PointCloud> forward_frame_current = getPointCloud(forward_frame);
    forward_frame_current->smoothLeaves(smooth_cost, true);
    forward_frame_current->printOrgans();

    timeToHideAndShowPointCloud(forward_frame-1, 12, forward_frame, 12);
    emit progressValueChanged(forward_frame-start_frame+1);
  }

  return;
}

void FileSystemModel::bSmoothLeaves(double smooth_cost, int start_frame, int end_frame)
{
  for (int backward_frame = start_frame; backward_frame >= end_frame; -- backward_frame)
  {
    osg::ref_ptr<PointCloud> backward_frame_current = getPointCloud(backward_frame);
    backward_frame_current->smoothLeaves(smooth_cost, false);
    backward_frame_current->printOrgans();

    timeToHideAndShowPointCloud(backward_frame+1, 12, backward_frame, 12);
    emit progressValueChanged(backward_frame-end_frame+1);
  }

  return;
}

void FileSystemModel::fReorderOrgans(int start_frame, int end_frame)
{
  for (int forward_frame = start_frame; forward_frame <= end_frame; ++ forward_frame)
  {
    osg::ref_ptr<PointCloud> backward_frame_current = getPointCloud(forward_frame);
    backward_frame_current->reorderOrgans(true);

    timeToHideAndShowPointCloud(forward_frame-1, 12, forward_frame, 12);
    emit progressValueChanged(forward_frame-start_frame+1);
  }

  return;
}

void FileSystemModel::bReorderOrgans(int start_frame, int end_frame)
{
  for (int backward_frame = start_frame; backward_frame >= end_frame; -- backward_frame)
  {
    osg::ref_ptr<PointCloud> backward_frame_current = getPointCloud(backward_frame);
    backward_frame_current->reorderOrgans(false);

    timeToHideAndShowPointCloud(backward_frame+1, 12, backward_frame, 12);
    emit progressValueChanged(backward_frame-end_frame+1);
  }

  return;
}

void FileSystemModel::forwardClassify(void)
{
  int start_frame, end_frame;
  double smooth_cost;
  if (!ParameterManager::getInstance().getTrackAndEvolveParameters(smooth_cost, start_frame, end_frame))
    return;

  ProgressBar* progress_bar = new ProgressBar(MainWindow::getInstance());
  progress_bar->setRange(0, end_frame-start_frame+1);
  progress_bar->setValue(0);
  progress_bar->setFormat(QString("Forward Classify: %p% completed"));
  progress_bar->setTextVisible(true);
  MainWindow::getInstance()->statusBar()->addPermanentWidget(progress_bar);

  QFutureWatcher<void>* watcher = new QFutureWatcher<void>(this);
  connect(this, SIGNAL(progressValueChanged(int)), progress_bar, SLOT(setValue(int)));
  connect(watcher, SIGNAL(finished()), progress_bar, SLOT(deleteLater()));

  watcher->setFuture(QtConcurrent::run(this, &FileSystemModel::forwardClassify, smooth_cost, start_frame, end_frame));

  return;
}

void FileSystemModel::backwardClassify(void)
{
  int start_frame, end_frame;
  double smooth_cost;
  if (!ParameterManager::getInstance().getTrackAndEvolveParameters(smooth_cost, start_frame, end_frame))
    return;

  ProgressBar* progress_bar = new ProgressBar(MainWindow::getInstance());
  progress_bar->setRange(0, end_frame-start_frame+1);
  progress_bar->setValue(0);
  progress_bar->setFormat(QString("Backward Classify: %p% completed"));
  progress_bar->setTextVisible(true);
  MainWindow::getInstance()->statusBar()->addPermanentWidget(progress_bar);

  QFutureWatcher<void>* watcher = new QFutureWatcher<void>(this);
  connect(this, SIGNAL(progressValueChanged(int)), progress_bar, SLOT(setValue(int)));
  connect(watcher, SIGNAL(finished()), progress_bar, SLOT(deleteLater()));

  watcher->setFuture(QtConcurrent::run(this, &FileSystemModel::backwardClassify, smooth_cost, end_frame, start_frame));

  return;
}


void FileSystemModel::fSmoothStems(void)
{
  int start_frame, end_frame;
  double smooth_cost;
  if (!ParameterManager::getInstance().getTrackAndEvolveParameters(smooth_cost, start_frame, end_frame))
    return;

  ProgressBar* progress_bar = new ProgressBar(MainWindow::getInstance());
  progress_bar->setRange(0, end_frame-start_frame+1);
  progress_bar->setValue(0);
  progress_bar->setFormat(QString("Forward Smooth Stems: %p% completed"));
  progress_bar->setTextVisible(true);
  MainWindow::getInstance()->statusBar()->addPermanentWidget(progress_bar);

  QFutureWatcher<void>* watcher = new QFutureWatcher<void>(this);
  connect(this, SIGNAL(progressValueChanged(int)), progress_bar, SLOT(setValue(int)));
  connect(watcher, SIGNAL(finished()), progress_bar, SLOT(deleteLater()));

  watcher->setFuture(QtConcurrent::run(this, &FileSystemModel::fSmoothStems, smooth_cost, start_frame, end_frame));

  return;
}


void FileSystemModel::bSmoothStems(void)
{
  int start_frame, end_frame;
  double smooth_cost;
  if (!ParameterManager::getInstance().getTrackAndEvolveParameters(smooth_cost, start_frame, end_frame))
    return;

  ProgressBar* progress_bar = new ProgressBar(MainWindow::getInstance());
  progress_bar->setRange(0, end_frame-start_frame+1);
  progress_bar->setValue(0);
  progress_bar->setFormat(QString("Backward Smooth Stems: %p% completed"));
  progress_bar->setTextVisible(true);
  MainWindow::getInstance()->statusBar()->addPermanentWidget(progress_bar);

  QFutureWatcher<void>* watcher = new QFutureWatcher<void>(this);
  connect(this, SIGNAL(progressValueChanged(int)), progress_bar, SLOT(setValue(int)));
  connect(watcher, SIGNAL(finished()), progress_bar, SLOT(deleteLater()));

  watcher->setFuture(QtConcurrent::run(this, &FileSystemModel::bSmoothStems, smooth_cost, end_frame, start_frame));

  return;
}


void FileSystemModel::bSmoothLeaves(void)
{
  int start_frame, end_frame;
  double smooth_cost;
  if (!ParameterManager::getInstance().getTrackAndEvolveParameters(smooth_cost, start_frame, end_frame))
    return;

  ProgressBar* progress_bar = new ProgressBar(MainWindow::getInstance());
  progress_bar->setRange(0, end_frame-start_frame+1);
  progress_bar->setValue(0);
  progress_bar->setFormat(QString("Backward Smooth Leaves: %p% completed"));
  progress_bar->setTextVisible(true);
  MainWindow::getInstance()->statusBar()->addPermanentWidget(progress_bar);

  QFutureWatcher<void>* watcher = new QFutureWatcher<void>(this);
  connect(this, SIGNAL(progressValueChanged(int)), progress_bar, SLOT(setValue(int)));
  connect(watcher, SIGNAL(finished()), progress_bar, SLOT(deleteLater()));

  watcher->setFuture(QtConcurrent::run(this, &FileSystemModel::bSmoothLeaves, smooth_cost, end_frame, start_frame));

  return;
}

void FileSystemModel::fSmoothLeaves(void)
{
  int start_frame, end_frame;
  double smooth_cost;
  if (!ParameterManager::getInstance().getTrackAndEvolveParameters(smooth_cost, start_frame, end_frame))
    return;

  ProgressBar* progress_bar = new ProgressBar(MainWindow::getInstance());
  progress_bar->setRange(0, end_frame-start_frame+1);
  progress_bar->setValue(0);
  progress_bar->setFormat(QString("Forward Smooth Leaves: %p% completed"));
  progress_bar->setTextVisible(true);
  MainWindow::getInstance()->statusBar()->addPermanentWidget(progress_bar);

  QFutureWatcher<void>* watcher = new QFutureWatcher<void>(this);
  connect(this, SIGNAL(progressValueChanged(int)), progress_bar, SLOT(setValue(int)));
  connect(watcher, SIGNAL(finished()), progress_bar, SLOT(deleteLater()));

  watcher->setFuture(QtConcurrent::run(this, &FileSystemModel::fSmoothLeaves, smooth_cost, start_frame, end_frame));

  return;
}

void FileSystemModel::fReorderOrgans(void)
{
  int start_frame, end_frame;
  if (!ParameterManager::getInstance().getFrameParameters(start_frame, end_frame))
    return;

  ProgressBar* progress_bar = new ProgressBar(MainWindow::getInstance());
  progress_bar->setRange(0, end_frame-start_frame+1);
  progress_bar->setValue(0);
  progress_bar->setFormat(QString("Forward Reorder Organs: %p% completed"));
  progress_bar->setTextVisible(true);
  MainWindow::getInstance()->statusBar()->addPermanentWidget(progress_bar);

  QFutureWatcher<void>* watcher = new QFutureWatcher<void>(this);
  connect(this, SIGNAL(progressValueChanged(int)), progress_bar, SLOT(setValue(int)));
  connect(watcher, SIGNAL(finished()), progress_bar, SLOT(deleteLater()));

  watcher->setFuture(QtConcurrent::run(this, &FileSystemModel::fReorderOrgans, start_frame, end_frame));

  return;
}

void FileSystemModel::bReorderOrgans(void)
{
  int start_frame, end_frame;
  if (!ParameterManager::getInstance().getFrameParameters(start_frame, end_frame))
    return;

  ProgressBar* progress_bar = new ProgressBar(MainWindow::getInstance());
  progress_bar->setRange(0, end_frame-start_frame+1);
  progress_bar->setValue(0);
  progress_bar->setFormat(QString("Backward Reorder Organs: %p% completed"));
  progress_bar->setTextVisible(true);
  MainWindow::getInstance()->statusBar()->addPermanentWidget(progress_bar);

  QFutureWatcher<void>* watcher = new QFutureWatcher<void>(this);
  connect(this, SIGNAL(progressValueChanged(int)), progress_bar, SLOT(setValue(int)));
  connect(watcher, SIGNAL(finished()), progress_bar, SLOT(deleteLater()));

  watcher->setFuture(QtConcurrent::run(this, &FileSystemModel::bReorderOrgans, end_frame, start_frame));

  return;
}