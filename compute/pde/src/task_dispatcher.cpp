#include <fstream>
#include <QProcess>
#include <QMessageBox>
#include <QMutexLocker>
#include <QProgressBar>
#include <QFutureWatcher>
#include <QtConcurrentFilter>
#include <QFileDialog>
#include <QComboBox>

#include "mesh_model.h"
#include "main_window.h"
#include "point_cloud.h"
#include "registrator.h"
#include "parameter_manager.h"
#include "file_system_model.h"
#include "povray_visitor.h"
#include "osg_viewer_widget.h"

#include "task_dispatcher.h"

TaskImpl::TaskImpl(int frame, int view)
  :frame_(frame),view_(view)
{}

TaskImpl::~TaskImpl(void)
{}

Task::Task(void)
{
}

Task::Task(TaskImpl* task_impl)
  :task_impl_(task_impl)
{
}

Task::Task(const Task &other)
{
  task_impl_ = other.task_impl_;
}

Task& Task::operator=(const Task &other)
{
  task_impl_ = other.task_impl_;
  return (*this);
}

Task::~Task()
{
}

bool Task::run(void) const
{
  task_impl_->run();

  emit finished(task_impl_->frame_, task_impl_->view_);

  return false;
}

TaskVirtualScan::TaskVirtualScan(int frame, double noise, double distance, double resolution)
  :TaskImpl(frame, -1),noise_(noise), distance_(distance), resolution_(resolution)
{}

TaskVirtualScan::~TaskVirtualScan(void)
{}

void TaskVirtualScan::run(void) const
{
  FileSystemModel* model = MainWindow::getInstance()->getFileSystemModel();

  osg::ref_ptr<MeshModel> mesh_model = model->getMeshModel(frame_);
  mesh_model->virtualScan(noise_, distance_, resolution_);
  model->updatePointCloud(frame_);
 
  return;
}

TaskExtractPoints::TaskExtractPoints(int frame, const QString& root_folder, int downsampling)
  :TaskImpl(frame, -1), root_folder_(root_folder), downsampling_(downsampling)
{}

TaskExtractPoints::~TaskExtractPoints(void)
{}

void TaskExtractPoints::run(void) const
{
  FileSystemModel* model = MainWindow::getInstance()->getFileSystemModel();

  QString original_folder(model->getPointsFolder(frame_).c_str());
  QString frame_string = QString("frame_%1").arg(frame_, 5, 10, QChar('0'));
  QString extract_folder = root_folder_+"/points/"+frame_string;
  QDir dir(root_folder_+"/points");
  dir.mkdir(frame_string);

  osg::ref_ptr<PointCloud> point_cloud = model->getPointCloud(frame_);
  if(point_cloud == NULL || point_cloud->size() == 0)
  {
    std::cout<<"Frame:"<<frame_<<" where is the point cloud?"<<std::endl;
    return;
  }

  std::vector<size_t> indices(point_cloud->getPlantPointsNum());
  for (size_t i = 0, i_end = indices.size(); i < i_end; ++ i)
    indices[i] = i;
  std::random_shuffle(indices.begin(), indices.end());

  PointCloud extract_cloud;  
  size_t extract_point_size = indices.size()/downsampling_;
  assert(extract_point_size != 0);
  while(extract_point_size > 0)
  {
    extract_cloud.push_back(point_cloud->at(indices[extract_point_size - 1]));
    extract_point_size --;
  }

  extract_cloud.save(extract_folder.toStdString()+"/points.pcd");
 
  return;
}


TaskExtractPlant::TaskExtractPlant(int frame, int delta)
  :TaskImpl(frame, -1), delta_(delta) 
{}

TaskExtractPlant::~TaskExtractPlant(void)
{}

struct CompareByLabel
{
  bool operator()(const PclRichPoint& a, const PclRichPoint& b)
  {
    return a.label > b.label;
  }
};

void TaskExtractPlant::run(void) const
{
  FileSystemModel* model = MainWindow::getInstance()->getFileSystemModel();
  osg::ref_ptr<PointCloud> point_cloud = model->getPointCloud(frame_);
  if(point_cloud == NULL || point_cloud->size() == 0)
  {
    QFile info(model->rootPath()+"/info.txt");
    info.open(QIODevice::WriteOnly | QIODevice::Text);
    QTextStream fout(&info);
    fout<<"Frame:"<<frame_<<" where is the point cloud?\n";
    return;
  }
  Registrator* registrator = MainWindow::getInstance()->getRegistrator();
  osg::Vec3 pivot_point = registrator->getPivotPoint();
  pivot_point.y() = pivot_point.y() + delta_;
  osg::Vec3 axis_normal = registrator->getAxisNormal();
  osg::Vec4 plane = osg::Plane(axis_normal, pivot_point).asVec4();
  double a = plane.x();
  double b = plane.y();
  double c = plane.z();
  double d = plane.w();

  size_t plant_points_num = 0, pot_points_num = 0;
  for (size_t i=0, i_end = point_cloud->size();i<i_end;i++) {
    const PclRichPoint& point = point_cloud->at(i);
    double check = a*point.x + b*point.y + c*point.z + d;
    if (check > 0)
    {
      plant_points_num ++;
      point_cloud->at(i).label = PclRichPoint::LABEL_PLANT;
    }
    else
      point_cloud->at(i).label = PclRichPoint::LABEL_POT;
  }

  pot_points_num = point_cloud->size() - plant_points_num;
  point_cloud->setPlantPointsNum(plant_points_num);
  point_cloud->setPotPointsNum(pot_points_num);
  std::sort(point_cloud->points.begin(), point_cloud->points.end(), CompareByLabel());
  point_cloud->save(point_cloud->getFilename());
  model->updatePointCloud(frame_);
  
  return;
}

TaskRotateCloud::TaskRotateCloud(int frame, int angle)
  :TaskImpl(frame, -1), angle_(angle)
{}

TaskRotateCloud::~TaskRotateCloud(void)
{}

void TaskRotateCloud::run(void) const
{
  FileSystemModel* model = MainWindow::getInstance()->getFileSystemModel();
  osg::ref_ptr<PointCloud> point_cloud = model->getPointCloud(frame_);
  double angle = angle_*30*M_PI/180.0;
  point_cloud->setMatrix(MainWindow::getInstance()->getRegistrator()->getRotationMatrix(angle));

  const osg::Matrix& matrix = point_cloud->getMatrix();
  for (size_t j = 0, j_end = point_cloud->size(); j < j_end; ++ j)
  {
    PclRichPoint& plant_point = point_cloud->at(j);

    osg::Vec3 point(plant_point.x, plant_point.y, plant_point.z);
    point = matrix.preMult(point);
    plant_point.x = point.x();
    plant_point.y = point.y();
    plant_point.z = point.z();

    osg::Vec3 normal(plant_point.normal_x, plant_point.normal_y, plant_point.normal_z);
    normal = matrix.preMult(normal);
    plant_point.normal_x = normal.x();
    plant_point.normal_y = normal.y();
    plant_point.normal_z = normal.z();
  }

  point_cloud->save(model->getPointsFilename(frame_));

  return;
}

TaskExtractKeyFrames::TaskExtractKeyFrames(int frame, int offset, int start_frame, bool is_point, QString root_folder)
  :TaskImpl(frame, -1), offset_(offset), start_frame_(start_frame), is_point_(is_point), root_folder_(root_folder) 
{}

TaskExtractKeyFrames::~TaskExtractKeyFrames(void)
{}

void TaskExtractKeyFrames::run(void) const
{
  if((frame_ - start_frame_)%offset_ == 0)
  {
    FileSystemModel* model = MainWindow::getInstance()->getFileSystemModel();
    size_t index = (frame_ - start_frame_)/offset_;

    QDir path = root_folder_+"/points/";
    QString frame_folder = QString("frame_%1").arg(index, 5, 10, QChar('0'));
    path.mkdir(frame_folder);
    QDir extract_folder = QDir(path.absolutePath()+"/"+frame_folder);   
    QDir original_folder(model->getPointsFolder(frame_).c_str());
    for(size_t i = 0; i < 12; i++)
    {
      QString view = QString("view_%1").arg(i, 2, 10, QChar('0'));
      extract_folder.mkdir(view);
      QString original_view = original_folder.absolutePath()+"/"+view+"/points.pcd";
      QString extract_view = extract_folder.absolutePath()+"/"+view+"/points.pcd";

      QString original_snapshot = original_folder.absolutePath()+"/"+view+"/snapshot.jpg";
      QString extract_snapshot = extract_folder.absolutePath()+"/"+view+"/snapshot.jpg";

      QFile::copy(original_view, extract_view);
      QFile::copy(original_snapshot, extract_snapshot);
    }
    
    QString original_file = original_folder.absolutePath()+"/points.pcd";
    QString extract_file = extract_folder.absolutePath()+"/points.pcd";
    QFile file(original_file);
    if(file.exists())
      QFile::copy(original_file, extract_file); 

    if(!is_point_)
    {
      QDir original_image_path = model->rootPath()+"/images/";
      QString orginal_frame_folder = QString("frame_%1").arg(frame_, 5, 10, QChar('0'));
      QString extracted_frame_folder = QString("frame_%1").arg(index, 5, 10, QChar('0'));
      
      QDir extract_image_path = root_folder_+"/images/";  
      extract_image_path.mkdir(frame_folder);
      QDir image_original = original_image_path.absolutePath() + "/" + orginal_frame_folder;
      QDir image_extracted = extract_image_path.absolutePath() + "/" + extracted_frame_folder;
      
      for(size_t i = 0; i < 12; i++)
      {
        QString view = QString("view_%1").arg(i, 2, 10, QChar('0'));
        image_extracted.mkdir(view);

        for(size_t j = 0 ; j < 30; j++)
        {
          QString image = QString("image_%1.jpg").arg(j, 2, 10, QChar('0'));
          QString original_view = image_original.absolutePath()+"/"+view+"/"+image;
          QString extract_view = image_extracted.absolutePath()+"/"+view+"/"+image;
          QFile::copy(original_view, extract_view);
        }
        
      }
    }
  
  }
  
  return;
}

TaskRenameFrames::TaskRenameFrames(int frame, int rename_offset)
:TaskImpl(frame, -1),rename_offset_(rename_offset)
{}

TaskRenameFrames::~TaskRenameFrames(void)
{}

void TaskRenameFrames::run(void) const
{
  FileSystemModel* model = MainWindow::getInstance()->getFileSystemModel();

  QDir images_dir(model->rootPath()+"/images");
  QDir points_dir(model->rootPath()+"/points");

  QString frame_folder = QString("frame_%1_tmp").arg(frame_, 5, 10, QChar('0'));


  int new_frame_number = frame_ + rename_offset_;
  QString new_frame_folder = QString("frame_%1").arg(rename_offset_ - frame_, 5, 10, QChar('0'));

  images_dir.rename(frame_folder, new_frame_folder);
  points_dir.rename(frame_folder, new_frame_folder);

  return;

  /*FileSystemModel* model = MainWindow::getInstance()->getFileSystemModel();
  QDir points_dir(model->rootPath()+"/points");
  QDir frame_folder = QString("frame_%1").arg(frame_, 5, 10, QChar('0'));
  std::cout<<frame_folder.absolutePath().toStdString()<<std::endl;
  for(size_t i = 0; i < 12; i ++)
  {
  QString view = QString("view_%1").arg(i, 2, 10, QChar('0'));
  QString view_renamed = QString("frame_%1").arg(i, 5, 10, QChar('0'));
  frame_folder.rename(view, view_renamed);
  }

  return;*/

}

TaskRenameViews::TaskRenameViews(int frame, int rename_offset)
  :TaskImpl(frame, -1),rename_offset_(rename_offset)
{}

TaskRenameViews::~TaskRenameViews(void)
{}

void TaskRenameViews::run(void) const
{
  /*FileSystemModel* model = MainWindow::getInstance()->getFileSystemModel();

  QDir points_dir(model->getPointsFolder(frame_).c_str());
  QDir images_dir(model->getImagesFolder(frame_).c_str());

  for (size_t view = 0; view < 12; ++ view)
  {
  QString view_folder = QString("slice_%1").arg(view, 2, 10, QChar('0'));
  if (!points_dir.exists(view_folder))
  view_folder = QString("view_%1").arg(view, 2, 10, QChar('0'));

  points_dir.rename(view_folder, view_folder+"_tmp");
  images_dir.rename(view_folder, view_folder+"_tmp");
  }

  for (int view = 0; view < 12; ++ view)
  {
  QString view_folder = QString("slice_%1_tmp").arg(view, 2, 10, QChar('0'));
  if (!points_dir.exists(view_folder))
  view_folder = QString("view_%1_tmp").arg(view, 2, 10, QChar('0'));

  int new_view_number = view + rename_offset_;
  if (new_view_number < 0)
  new_view_number = 12+new_view_number;
  if (new_view_number > 11)
  new_view_number = new_view_number-12;
  QString new_view_folder = QString("view_%1").arg(new_view_number, 2, 10, QChar('0'));

  points_dir.rename(view_folder, new_view_folder);
  images_dir.rename(view_folder, new_view_folder);
  }*/

  FileSystemModel* model = MainWindow::getInstance()->getFileSystemModel();
  osg::ref_ptr<PointCloud> point_cloud = model->getPointCloud(0);
  point_cloud->update();
  osg::Vec3 center = point_cloud->getBound().center();
  double radius = point_cloud->getBound().radius();

  point_cloud = model->getPointCloud(frame_);
  point_cloud->update();

  PointCloud new_cloud;
  for(size_t i = 0, i_end = point_cloud->size(); i < i_end; i++)
  {
    PclRichPoint point = point_cloud->at(i);
    double distance = std::sqrt((point.x-center.x())*(point.x-center.x())+(point.y-center.y())*(point.y-center.y())+
      (point.z-center.z())*(point.z-center.z()));
    if(distance <= radius)
      new_cloud.push_back(point);
  }
  new_cloud.save(model->getPointsFilename(frame_));

  return;

  /*FileSystemModel* model = MainWindow::getInstance()->getFileSystemModel();
  QString frame_dir(model->getPointsFolder(frame_).c_str());
  QString view = QString("view_%1").arg(11, 2, 10, QChar('0'));
  QString original_image = frame_dir+"/"+view+"/snapshot.jpg";

  QString extracted_dir(model->rootPath()+"/snapshots");
  QString extracted_image = QString(extracted_dir+"/%1.jpg").arg(frame_, 2, 10, QChar('0'));

  QFile::copy(original_image, extracted_image);*/
  
}


TaskRemoveBadFrames::TaskRemoveBadFrames(int view, int start_frame, int end_frame)
  :TaskImpl(-1, view), start_frame_(start_frame), end_frame_(end_frame)
{

}

TaskRemoveBadFrames::~TaskRemoveBadFrames(void)
{

}
                                              
void TaskRemoveBadFrames::run() const
{
  /*FileSystemModel* model = MainWindow::getInstance()->getFileSystemModel();

  QFile file = model->rootPath()+QString("/Info_%1.txt").arg(view_);
  file.open(QIODevice::WriteOnly | QIODevice::Text);
  QTextStream fout(&file);

  const size_t delta = 80;
  for(size_t i = start_frame_; i <= end_frame_; i = i + delta)
  {
  osg::ref_ptr<PointCloud> this_cloud = model->getPointCloud(i, view_);
  this_cloud->update();
  double radius = this_cloud->getBound().radius();

  size_t j = i;
  while(radius > 500 && j < end_frame_ && j < delta/2)
  {
  this_cloud = model->getPointCloud(++j, view_);
  if(this_cloud == NULL)
  {
  radius = 10000;
  continue;
  }
  this_cloud->update();
  radius = this_cloud->getBound().radius();

  }


  fout<<"Frame:"<<i<<" "<<"View:"<<view_<<"\n";

  if(j == end_frame_ && radius > 500)
  break;

  std::string this_file = model->getPointsFilename(i, view_);
  std::string next_file = model->getPointsFilename(j, view_);

  if((j != i && j != end_frame_)||(j == end_frame_ && radius <= 500))
  {
  QFile::remove(QString::fromUtf8(this_file.c_str()));
  QFile::copy(QString::fromUtf8(next_file.c_str()), QString::fromUtf8(this_file.c_str()));
  }   

  }
  return;*/


  FileSystemModel* model = MainWindow::getInstance()->getFileSystemModel();
  QFile file_modify = model->rootPath()+QString("/ModifiedFrameInfo_%1.txt").arg(view_);
  QFile file_temp = model->rootPath()+QString("/AdjacentFrameInfo_%1.txt").arg(view_);
  file_modify.open(QIODevice::WriteOnly | QIODevice::Text);
  file_temp.open(QIODevice::WriteOnly | QIODevice::Text);
  QTextStream fout_modify(&file_modify);
  QTextStream fout_temp(&file_temp);

  std::string this_file, prev_file, next_file;
  osg::ref_ptr<PointCloud> this_cloud, prev_cloud, next_cloud;
  int this_num, prev_num, next_num;

  prev_file = model->getPointsFilename(start_frame_, view_);
  prev_cloud = model->getPointCloud(start_frame_, view_);
  prev_num = prev_cloud->size();

  this_file = model->getPointsFilename(start_frame_+1, view_);
  this_cloud = model->getPointCloud(start_frame_+1,view_);
  this_num = this_cloud->size();

  for(size_t i = start_frame_+1, i_end = end_frame_; i <= i_end; i++)
  {  
    if(i_end == i)
    {
      double this_ratio = (double)(prev_num - this_num)/prev_num;
      if(this_ratio > 0.05)
      {
        QFile::remove(QString::fromUtf8(this_file.c_str()));
        QFile::copy(QString::fromUtf8(prev_file.c_str()), QString::fromUtf8(this_file.c_str()));
        this_cloud->reload();
        this_cloud = model->getPointCloud(i,view_);
        fout_modify<<"Frame:"<<i<<" "<<"View:"<<view_<<" "<<"ratio:"<<this_ratio<<"\n";
      }

      fout_temp<<"Frame:"<<i<<" "<<"View:"<<view_<<" "<<prev_num<<" "<<this_num<<" "<<
        this_ratio<<"\n";
      fout_temp<<"\n";

      continue;
    }

    next_file = model->getPointsFilename(i+1, view_);
    QFile next_qfile(QString::fromUtf8(next_file.c_str()));
    bool flag = next_qfile.exists();
    if(!flag)
    {
      QFile::copy(QString::fromUtf8(this_file.c_str()), QString::fromUtf8(next_file.c_str())); 
      next_cloud = model->getPointCloud(i+1,view_);
      next_num = 0;
    }
    else
    {
      next_cloud = model->getPointCloud(i+1,view_);
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
      QFile::remove(QString::fromUtf8(this_file.c_str()));
      QFile::copy(QString::fromUtf8(prev_file.c_str()), QString::fromUtf8(this_file.c_str()));
      this_cloud->reload();
      this_cloud = model->getPointCloud(i,view_);
      fout_modify<<"Frame:"<<i<<" "<<"View:"<<view_<<" "<<"ratio:"<<this_ratio<<"\n";
    }

    fout_temp<<"Frame:"<<i<<" "<<"View:"<<view_<<" "<<prev_num<<" "<<this_num<<" "<<
      this_ratio<<"\n";
    fout_temp<<"\n";

    prev_file = this_file;
    prev_cloud = this_cloud;
    prev_num = this_num;

    this_file = next_file;
    this_cloud = next_cloud;
    this_num = next_num;
  }

  file_modify.close();
  file_temp.close();

  return;
}

TaskGeneratePovrayData::TaskGeneratePovrayData(int frame, PovRayVisitor* master, QString root_folder)
  :TaskImpl(frame, -1), master_(master), root_folder_(root_folder)
{
  
}

TaskGeneratePovrayData::~TaskGeneratePovrayData(void)
{}


void TaskGeneratePovrayData::run() const
{
  OSGViewerWidget* osg_viewer_widget = MainWindow::getInstance()->getOSGViewerWidget();
  FileSystemModel* model = MainWindow::getInstance()->getFileSystemModel();
  osg::ref_ptr<PointCloud> poind_cloud = model->getPointCloud(frame_);
  QString data_filename = QString("data_%1.inc").arg(frame_, 5, 10, QChar('0'));
  QString data_filepath = root_folder_+"/data/"+data_filename;
  QFile data_file(data_filepath);
  data_file.open(QIODevice::WriteOnly | QIODevice::Text);
  QTextStream data_file_stream(&data_file);
  SlavePovRayVisitor slave;
  slave.setMaster(master_);
  poind_cloud->povray(&slave);

  data_file_stream << slave.getPovrayText();
  data_file.close();

  QString texture_filename = QString("texture_%1.inc").arg(frame_, 5, 10, QChar('0'));
  QString texture_filepath = root_folder_+"/texture/"+texture_filename;

  PointCloud::ColorMode color_mode = (PointCloud::ColorMode)MainWindow::getInstance()->getColorMode()->currentIndex();
  if(color_mode == PointCloud::ORGAN)
    slave.saveTextureFileOrgan(texture_filepath,frame_, poind_cloud->getLeafNum(), poind_cloud->getStemNum());
  else
    slave.saveTextureFile(texture_filepath);

  return;
}

TaskPointsGeneration::TaskPointsGeneration(int frame, int view, int ctr_threshold, int sat_threshold)
  :TaskImpl(frame, view),ctr_threshold_(ctr_threshold), sat_threshold_(sat_threshold)
{}

TaskPointsGeneration::~TaskPointsGeneration(void)
{}

void TaskPointsGeneration::run(void) const
{
  convertImages();

  FileSystemModel* model = MainWindow::getInstance()->getFileSystemModel();

  QStringList arguments;
  arguments << model->getPointsFolder(frame_, view_).c_str()
    << QString::number(ctr_threshold_) << QString::number(sat_threshold_);
  QProcess process;
  process.start(getExeFilename(), arguments);
  process.waitForFinished(-1);

  deleteImages();
  colorizePoints();

  model->updatePointCloud(frame_, view_);

  return;
}

void TaskPointsGeneration::convertImages(void) const
{
  FileSystemModel* model = MainWindow::getInstance()->getFileSystemModel();

  for (size_t i = 0; i < 30; ++ i)
  {
    QString load_filename = QString("%1/image_%2.jpg")
      .arg(model->getImagesFolder(frame_, view_).c_str()).arg(i, 2, 10, QChar('0'));

    if (!QFile::exists(load_filename))
      continue;

    QImage image;
    bool f = image.load(load_filename);

    QString save_filename = QString("%1/%2.bmp").arg(model->getPointsFolder(frame_, view_).c_str()).arg(i);
    bool flag = image.save(save_filename);
  }

  return;
}

void TaskPointsGeneration::deleteImages(void) const
{
  FileSystemModel* model = MainWindow::getInstance()->getFileSystemModel();

  for (size_t i = 0; i < 30; ++ i)
  {
    QString delete_filename = QString("%1/%2.bmp").arg(model->getPointsFolder(frame_, view_).c_str()).arg(i);
    QFile::remove(delete_filename);
  }

  return;
}

void TaskPointsGeneration::colorizePoints(void) const
{
  FileSystemModel* model = MainWindow::getInstance()->getFileSystemModel();
  std::string points_folder = model->getPointsFolder(frame_, view_);

  std::string filename = points_folder+"/points.bxyzuv";
  if (!QFile::exists(filename.c_str()))
    return;

  QImage snapshot((points_folder+"/snapshot.jpg").c_str());

  std::ifstream fin(filename, std::ios::binary);
  if (!fin.good())
    return;

  long begin, end;
  fin.seekg(0, std::ios::end);
  end = fin.tellg();
  fin.seekg(0, std::ios::beg);
  begin = fin.tellg();

  PointCloud point_cloud;
  long num_points = (end-begin)/(5*sizeof(double));
  PclRichPoint point;
  while (num_points)
  {
    double x, y, z;
    fin.read((char*)&x, sizeof(double));
    fin.read((char*)&y, sizeof(double));
    fin.read((char*)&z, sizeof(double));
    point.x = x;
    point.y = y;
    point.z = z;

    osg::Vec3 normal(-x, -y, -z);
    normal.normalize();
    point.normal_x = normal.x();
    point.normal_y = normal.y();
    point.normal_z = normal.z();

    double u, v;
    fin.read((char*)&u, sizeof(double));
    fin.read((char*)&v, sizeof(double));

    int px = std::min(std::max((int)(u), 0), snapshot.width()-1);
    int py = std::min(std::max((int)(v), 0), snapshot.height()-1);
    QColor rgb(snapshot.pixel(px, py));
    point.r = rgb.red();
    point.g = rgb.green();
    point.b = rgb.blue();

    point_cloud.push_back(point);
    num_points --;
  }

  fin.close();
  if (!filename.empty())
    QFile::remove(filename.c_str());

  point_cloud.save(points_folder+"/points.pcd");

  return;
}


QString TaskPointsGeneration::getExeFilename(void)
{
  FileSystemModel* model = MainWindow::getInstance()->getFileSystemModel();

  QString exe_filename;
  QStringList root_entries = QDir(model->rootPath()).entryList();
  for (QStringList::const_iterator root_entries_it = root_entries.begin();
    root_entries_it != root_entries.end(); ++ root_entries_it)
  {
    if (root_entries_it->startsWith("EvoGeoConvert") && root_entries_it->endsWith(".exe"))
    {
      exe_filename = model->rootPath()+"/"+*root_entries_it;
      break;
    }
  }

  return exe_filename;
}

TaskDispatcher::TaskDispatcher(QObject* parent)
  :QObject(parent)
{
  
}

TaskDispatcher::~TaskDispatcher(void)
{
  cancelRunningTasks(true);

  return;
}

bool TaskDispatcher::isRunning(void) const
{
  QMutexLocker locker(&mutex_);

  return (!active_watchers_.empty());
}

void TaskDispatcher::cancelRunningTasks(bool wait)
{
  QMutexLocker locker(&mutex_);

  for (size_t i = 0, i_end = active_watchers_.size(); i < i_end; ++ i)
  {
    QFutureWatcher<void>* watcher = dynamic_cast<QFutureWatcher<void>*>(active_watchers_[i]);
    watcher->cancel();
    if (wait)
      watcher->waitForFinished();
  }

  return;
}

void TaskDispatcher::removeFinishedWatchers(void)
{
  QMutexLocker locker(&mutex_);

  std::vector<QObject*> temp;
  for (size_t i = 0, i_end = active_watchers_.size(); i < i_end; ++ i)
  {
    QFutureWatcher<void>* watcher = dynamic_cast<QFutureWatcher<void>*>(active_watchers_[i]);
    if (!watcher->isFinished())
      temp.push_back(active_watchers_[i]);
    else
      watcher->deleteLater();
  }

  active_watchers_ = temp;

  return;
}

void TaskDispatcher::clearDisplayQueue(void)
{
  QMutexLocker locker(&mutex_);

  FileSystemModel* model = MainWindow::getInstance()->getFileSystemModel();
  for (DisplayQueue::iterator it = display_queue_.begin(); it != display_queue_.end(); ++ it)
  {
    int hide_frame = it->first;
    int hide_view = it->second;
    model->hidePointCloud(hide_frame, hide_view);
  }
  display_queue_.clear();

  return;
}

void TaskDispatcher::updateDisplayQueue(int frame, int view)
{
  QMutexLocker locker(&mutex_);

  FileSystemModel* model = MainWindow::getInstance()->getFileSystemModel();
  int queue_size_threshold = QThread::idealThreadCount();

  if (display_queue_.size() < queue_size_threshold)
    model->showPointCloud(frame, view);
  else
  {
    int hide_frame = display_queue_.front().first;
    int hide_view = display_queue_.front().second;
    model->hideAndShowPointCloud(hide_frame, hide_view, frame, view);
    display_queue_.pop_front();
  }

  display_queue_.push_back(std::make_pair(frame, view));

  return;
}

void TaskDispatcher::runTasks(QList<Task>& tasks, const QString& task_name, bool display)
{
  QProgressBar* progress_bar = new QProgressBar(MainWindow::getInstance());
  progress_bar->setRange(0, tasks.size());
  progress_bar->setValue(0);
  progress_bar->setFormat(QString("%1: %p% completed").arg(task_name));
  progress_bar->setTextVisible(true);
  MainWindow::getInstance()->statusBar()->addPermanentWidget(progress_bar);

  QFutureWatcher<void>* watcher = new QFutureWatcher<void>(this);
  active_watchers_.push_back(watcher);
  
  connect(watcher, SIGNAL(progressValueChanged(int)), progress_bar, SLOT(setValue(int)));
  connect(watcher, SIGNAL(finished()), progress_bar, SLOT(deleteLater()));
  connect(watcher, SIGNAL(finished()), this, SLOT(removeFinishedWatchers()));

  if (display)
  {
    connect(watcher, SIGNAL(finished()), this, SLOT(clearDisplayQueue()));
    for (QList<Task>::const_iterator it = tasks.begin(); it != tasks.end(); ++ it)
      connect(&(*it), SIGNAL(finished(int, int)), this, SLOT(updateDisplayQueue(int, int)));
  }

  watcher->setFuture(QtConcurrent::filter(tasks, &Task::run));

  return;
}

void TaskDispatcher::dispatchTaskVirtualScan(void)
{
  if (!virtual_scan_tasks_.isEmpty())
  {
    QMessageBox::warning(MainWindow::getInstance(), "Virtual Scan Task Warning",
      "Run virtual scan task after the previous one has finished");
    return;
  }

  int start_frame, end_frame;
  double noise, distance, resolution;
  if (!ParameterManager::getInstance()
    .getVirtualScanParameters(noise, distance, resolution, start_frame, end_frame))
    return;

  for (int frame = start_frame; frame <= end_frame; frame ++)
    virtual_scan_tasks_.push_back(Task(new TaskVirtualScan(frame, noise, distance, resolution)));

  runTasks(virtual_scan_tasks_, "Virtual Scan");

  return;
}

void TaskDispatcher::dispatchTaskRenameViews(void)
{
  if (!rename_views_tasks_.isEmpty())
  {
    QMessageBox::warning(MainWindow::getInstance(), "Rename Task Warning",
      "Run rename task after the previous one has finished");
    return;
  }

  int rename_offset, start_frame, end_frame;
  if (!ParameterManager::getInstance()
    .getRenameViewsParameters(rename_offset, start_frame, end_frame))
    return;

  FileSystemModel* model = MainWindow::getInstance()->getFileSystemModel();

  QDir images_dir(model->rootPath());
  images_dir.mkdir("snapshots");


  for (int frame = start_frame; frame <= end_frame; frame ++)
    rename_views_tasks_.push_back(Task(new TaskRenameViews(frame, rename_offset)));

  runTasks(rename_views_tasks_, "Rename Views", false);

  return;
}

void TaskDispatcher::dispatchTaskRenameFrames(void)
{
  if (!rename_frames_tasks_.isEmpty())
  {
    QMessageBox::warning(MainWindow::getInstance(), "Rename Task Warning",
      "Run rename task after the previous one has finished");
    return;
  }

  int rename_offset, start_frame, end_frame;
  if (!ParameterManager::getInstance()
    .getRenameFramesParameters(rename_offset, start_frame, end_frame))
    return;

  FileSystemModel* model = MainWindow::getInstance()->getFileSystemModel();

  QDir images_dir(model->rootPath()+"/images");
  QDir points_dir(model->rootPath()+"/points");

  for (int frame = end_frame; frame >= start_frame; frame --)
  {
    QString frame_folder = QString("frame_%1").arg(frame, 5, 10, QChar('0'));
    images_dir.rename(frame_folder, frame_folder+"_tmp");
    points_dir.rename(frame_folder, frame_folder+"_tmp");
  }
  

  for (int frame = end_frame; frame >= start_frame; frame --)
    rename_frames_tasks_.push_back(Task(new TaskRenameFrames(frame, rename_offset)));

  runTasks(rename_frames_tasks_, "Rename Frames", false);

  return;
}

void TaskDispatcher::dispathcTaskRemoveBadFrames(void)
{
  if (!remove_bad_frames_tasks_.isEmpty())
  {
    QMessageBox::warning(MainWindow::getInstance(), "Remove Task Warning",
      "Run remove task after the previous one has finished");
    return;
  }

  int start_frame, end_frame;
  if (!ParameterManager::getInstance()
    .getBadFramesParameters(start_frame, end_frame))
    return;

  for (int view = 0; view < 12; view ++)
    remove_bad_frames_tasks_.push_back(Task(new TaskRemoveBadFrames(view, start_frame, end_frame)));

  runTasks(remove_bad_frames_tasks_, "Remove Bad Frames", false);

  return;
}

void TaskDispatcher::dispathcTaskExtractPlant(void)
{
  if (!extract_plant_tasks_.isEmpty())
  {
    QMessageBox::warning(MainWindow::getInstance(), "Extract Task Warning",
      "Run Extract task after the previous one has finished");
    return;
  }

  int start_frame, end_frame;
  int delta;
  if (!ParameterManager::getInstance()
    .getExtractPlantParameters(start_frame, end_frame, delta))
    return;

  for (int frame = start_frame; frame <= end_frame; frame ++)
    extract_plant_tasks_.push_back(Task(new TaskExtractPlant(frame, delta)));

  runTasks(extract_plant_tasks_, "Extract Plant", false);

  return;
}

void TaskDispatcher::dispathcTaskExtractKeyFrames(void)
{
  if (!extract_key_frames_.isEmpty())
  {
    QMessageBox::warning(MainWindow::getInstance(), "Extract Task Warning",
      "Run Extract task after the previous one has finished");
    return;
  }

  int start_frame, end_frame, frame_offset;
  bool is_point;
  if (!ParameterManager::getInstance()
    .getExtractKeyFramesParameters(frame_offset ,start_frame, end_frame , is_point))
    return;

  /*FileSystemModel* model = MainWindow::getInstance()->getFileSystemModel();
  QDir root(model->rootPath());
  root.mkdir("KeyFrames");
  root.mkdir("KeyFrames/points");
  if(!is_point)
    root.mkdir("KeyFrames/images");*/

  MainWindow* main_window = MainWindow::getInstance();
  QDir workspaceUp(main_window->getWorkspace());
  workspaceUp.cdUp();
  QString root_folder = QFileDialog::getExistingDirectory(main_window, "Extract Points Folder", workspaceUp.path());
  if (root_folder.isEmpty())
    return;

  QDir folder(root_folder);
  if(!is_point)
    folder.mkdir("images");
  folder.mkdir("points");

  for (int frame = start_frame; frame <= end_frame; frame ++)
    extract_key_frames_.push_back(Task(new TaskExtractKeyFrames(frame, frame_offset, start_frame, is_point, root_folder)));

  runTasks(extract_key_frames_, "Extract Frames", false);

  return;
}

void TaskDispatcher::dispathcTaskRotateCloud(void)
{
  if (!rotate_cloud_tasks_.isEmpty())
  {
    QMessageBox::warning(MainWindow::getInstance(), "Rotate Task Warning",
      "Run Rotate task after the previous one has finished");
    return;
  }

  int start_frame, end_frame, angle;
  if (!ParameterManager::getInstance()
    .getRotateCloudParameters(angle ,start_frame, end_frame))
    return;


  for (int frame = start_frame; frame <= end_frame; frame ++)
    rotate_cloud_tasks_.push_back(Task(new TaskRotateCloud(frame, angle)));

  runTasks(rotate_cloud_tasks_, "Rotate Cloud", false);

  return;
}

void TaskDispatcher::dispathcTaskGeneratePovrayData(void)
{
  if (!generate_povray_data_tasks_.isEmpty())
  {
    QMessageBox::warning(MainWindow::getInstance(), "Generate Povray Data Task Warning",
      "Run generate povray data task after the previous one has finished");
    return;
  }

  if (!ParameterManager::getInstance()
    .getPovrayDataParameters(start_frame_, end_frame_, frame_delta_, frame_multiple_, camera_number_, stop_delta_, camera_delta_))
    return;

  /*FileSystemModel* model = MainWindow::getInstance()->getFileSystemModel();
  QDir root(model->rootPath());
  root.mkdir("POV-Ray");
  root.mkdir("POV-Ray/data");
  root.mkdir("POV-Ray/texture");*/

  MainWindow* main_window = MainWindow::getInstance();
  QDir workspaceUp(main_window->getWorkspace());
  workspaceUp.cdUp();
  QString root_folder = QFileDialog::getExistingDirectory(main_window, "Extract Points Folder", workspaceUp.path());
  if (root_folder.isEmpty())
    return;

  root_folder_ = root_folder;
  QDir dir(root_folder);
  dir.mkdir("data");
  dir.mkdir("texture");

   
  for (int frame = start_frame_; frame <= end_frame_; frame ++)
    generate_povray_data_tasks_.push_back(Task(new TaskGeneratePovrayData(frame, &master_ ,root_folder)));

  //////////////////////////////////////////////////////////////////////////
  QProgressBar* progress_bar = new QProgressBar(MainWindow::getInstance());
  progress_bar->setRange(0, generate_povray_data_tasks_.size());
  progress_bar->setValue(0);
  progress_bar->setFormat(QString("%1: %p% completed").arg("Generate Povray Data"));
  progress_bar->setTextVisible(true);
  MainWindow::getInstance()->statusBar()->addPermanentWidget(progress_bar);

  QFutureWatcher<void>* watcher = new QFutureWatcher<void>(this);
  active_watchers_.push_back(watcher);

  connect(watcher, SIGNAL(progressValueChanged(int)), progress_bar, SLOT(setValue(int)));
  connect(watcher, SIGNAL(finished()), progress_bar, SLOT(deleteLater()));
  connect(watcher, SIGNAL(finished()), this, SLOT(removeFinishedWatchers()));
  connect(watcher, SIGNAL(finished()), this, SLOT(clearDisplayQueue()));
  connect(watcher, SIGNAL(finished()), this, SLOT(finishPovRayMaster()));
  for (QList<Task>::const_iterator it = generate_povray_data_tasks_.begin(); it != generate_povray_data_tasks_.end(); ++ it)
    connect(&(*it), SIGNAL(finished(int, int)), this, SLOT(updateDisplayQueue(int, int)));

  watcher->setFuture(QtConcurrent::filter(generate_povray_data_tasks_, &Task::run));
  //////////////////////////////////////////////////////////////////////////


  return;
}

void TaskDispatcher::finishPovRayMaster(void)
{
  OSGViewerWidget* osg_viewer_widget = MainWindow::getInstance()->getOSGViewerWidget();
  FileSystemModel* model = MainWindow::getInstance()->getFileSystemModel();
  master_.setOSGViewerWidget(osg_viewer_widget);
  QString folder(root_folder_);
  osg::ref_ptr<PointCloud> poind_cloud = model->getPointCloud(start_frame_);
  SlavePovRayVisitor slave(osg_viewer_widget);
  slave.setMaster(&master_);
  poind_cloud->povray(&slave);

  QString size_filepath(folder+"/size.inc");
  slave.saveSizeFile(size_filepath);

  QString light_filepath(folder+"/light.inc");
  slave.saveLightFile(light_filepath);

  QFile povray_file(folder+"/scene.pov");
  povray_file.open(QIODevice::WriteOnly | QIODevice::Text);
  QTextStream povray_file_stream(&povray_file);
  povray_file_stream << "#version 3.7;\n\n";
  povray_file_stream << "global_settings { assumed_gamma 1.0 }\n\n";
  povray_file_stream << "#include \"camera.inc\"\n";
  povray_file_stream << "#include \"light.inc\"\n";
  povray_file_stream << "#include \"size.inc\"\n";

  size_t frame_number = (end_frame_-start_frame_+1)/frame_delta_; 
  size_t multiple = frame_multiple_;
  for(size_t i = 0; i < frame_number; i ++)
  {
    for(size_t j = 0; j < multiple; j ++)
    {
      QString include = QString("#if(frame_number = %1)\n").arg(i*multiple+j);
      QString data_filename = QString("#include \"data\\data_%1.inc\"\n").arg(start_frame_+i*frame_delta_, 5, 10, QChar('0'));
      QString texture_filename = QString("#include \"texture\\texture_%1.inc\"\n").arg(start_frame_+i*frame_delta_, 5, 10, QChar('0'));
      QString end = QString("#end\n");
      povray_file_stream << include;
      povray_file_stream << texture_filename;
      povray_file_stream << data_filename;
      povray_file_stream << end;
    }
    
  }

  QString camera_filename(folder+"/camera.inc");
  master_.saveCameraFile(camera_filename, frame_number, camera_number_, stop_delta_, camera_delta_, frame_delta_, frame_multiple_, end_frame_);


  QFile ini_file(folder+"/scene.ini");
  ini_file.open(QIODevice::WriteOnly | QIODevice::Text);
  QTextStream ini_file_stream(&ini_file);
  ini_file_stream << "Input_File_Name = \"scene.pov\"\nOutput_File_Name = scene_\n\n";
  ini_file_stream << "Antialias = On\nAntialias_Threshold = 0.3\nAntialias_Depth = 2\n\n";
  ini_file_stream << "Output_Alpha = On\n\n";
  ini_file_stream << QString("Height = %1\nWidth = %2\n\n").arg(osg_viewer_widget->height()).arg(osg_viewer_widget->width());
  ini_file_stream << QString("Initial_Frame = %1\nFinal_Frame= %2\n\n").arg(0).arg(frame_number-1);
}

void TaskDispatcher::dispatchTaskExtractPoints(void)
{
  if (!extract_points_tasks_.isEmpty())
  {
    QMessageBox::warning(MainWindow::getInstance(), "Extract Points Warning",
      "Run extract points task after the previous one has finished");
    return;
  }

  int start_frame, end_frame;
  int downsampling;
  if (!ParameterManager::getInstance().getFrameParameters(start_frame, end_frame, downsampling))
    return;

  MainWindow* main_window = MainWindow::getInstance();
  QDir workspaceUp(main_window->getWorkspace());
  workspaceUp.cdUp();
  QString root_folder = QFileDialog::getExistingDirectory(main_window, "Extract Points Folder", workspaceUp.path());
  if (root_folder.isEmpty())
    return;

  QDir dir(root_folder);
  dir.mkdir("points");

  for (int frame = start_frame; frame <= end_frame; frame ++)
    extract_points_tasks_.push_back(Task(new TaskExtractPoints(frame, root_folder, downsampling)));

  runTasks(extract_points_tasks_, "Extract Points", false);

  return;
}

void TaskDispatcher::dispatchTaskPointsGeneration(void)
{
  if (!points_generation_tasks_.isEmpty())
  {
    QMessageBox::warning(MainWindow::getInstance(), "Points Generation Task Warning",
      "Run points generation task after the previous one has finished");
    return;
  }

  int ctr_threshold, sat_threshold, start_frame, end_frame;
  if (!ParameterManager::getInstance()
    .getGenerationParameters(ctr_threshold, sat_threshold, start_frame, end_frame))
    return;

  QString exe_filename = TaskPointsGeneration::getExeFilename();
  if (!QFile::exists(exe_filename))
  {
    QMessageBox::warning(MainWindow::getInstance(), "Point Cloud Generator Warning",
      "There's no EvoGeoConvert.exe in the root folder");
    return;
  }

  for (int frame = start_frame; frame <= end_frame; frame ++)
    for (int view = 0; view < 12; ++ view)
      points_generation_tasks_.push_back(Task(new TaskPointsGeneration(frame, view, ctr_threshold, sat_threshold)));

  runTasks(points_generation_tasks_, "Points Generation");

  return;
}

TaskRegistration::TaskRegistration(int frame, int segment_threshold, int max_iterations, double max_distance)
  :TaskImpl(frame, -1), segment_threshold_(segment_threshold), max_iterations_(max_iterations), max_distance_(max_distance)
{}

TaskRegistration::~TaskRegistration(void)
{}

void TaskRegistration::run(void) const
{
  Registrator* registrator = MainWindow::getInstance()->getRegistrator();
  registrator->registrationLUM(segment_threshold_, max_iterations_, max_distance_, frame_);

  return;
}

void TaskDispatcher::dispatchTaskRegistration(void)
{
  if (!registration_tasks_.isEmpty())
  {
    QMessageBox::warning(MainWindow::getInstance(), "Registration Task Warning",
      "Run registration task after the previous one has finished");
    return;
  }

  int segment_threshold, max_iteration, start_frame, end_frame;
  double max_distance;
  if (!ParameterManager::getInstance().getRegistrationLUMParameters(segment_threshold, max_iteration, max_distance, start_frame, end_frame))
    return;

  for (int frame = start_frame; frame <= end_frame; frame ++)
    registration_tasks_.push_back(Task(new TaskRegistration(frame, segment_threshold, max_iteration, max_distance)));

  runTasks(registration_tasks_, "Register Frames");

  return;
}

TaskEstimateNormal::TaskEstimateNormal(int frame, int view, double normal_radius)
  :TaskImpl(frame, view),normal_radius_(normal_radius)
{}

TaskEstimateNormal::~TaskEstimateNormal(void)
{}

void TaskEstimateNormal::run(void) const
{
  FileSystemModel* model = MainWindow::getInstance()->getFileSystemModel();

  osg::ref_ptr<PointCloud> point_cloud = model->getPointCloud(frame_, view_);
  if (point_cloud.valid())
  {
    point_cloud->estimateNormal(normal_radius_);
    point_cloud->save(point_cloud->getFilename());
    model->updatePointCloud(frame_, view_);
  }

  return;
}

void TaskDispatcher::dispatchTaskEstimateNormal(void)
{
  if (!estimate_normal_tasks_.isEmpty())
  {
    QMessageBox::warning(MainWindow::getInstance(), "Estimate Normal Task Warning",
      "Run estimate normal task after the previous one has finished");
    return;
  }

  int start_frame, end_frame;
  double normal_radius;
  if (!ParameterManager::getInstance().getEstimateNormalParameters(normal_radius, start_frame, end_frame))
    return;

  for (int frame = start_frame; frame <= end_frame; frame ++)
      estimate_normal_tasks_.push_back(Task(new TaskEstimateNormal(frame, 12, normal_radius)));

  runTasks(estimate_normal_tasks_, "Estimate Normal");
}

TaskEstimateThickness::TaskEstimateThickness(int frame, int view, double thickness_radius)
  :TaskImpl(frame, view),thickness_radius_(thickness_radius)
{}

TaskEstimateThickness::~TaskEstimateThickness(void)
{}

void TaskEstimateThickness::run(void) const
{
  FileSystemModel* model = MainWindow::getInstance()->getFileSystemModel();

  osg::ref_ptr<PointCloud> point_cloud = model->getPointCloud(frame_, view_);
  if (point_cloud.valid())
  {
    point_cloud->estimateThickness(thickness_radius_);
    point_cloud->save(point_cloud->getFilename());
    model->updatePointCloud(frame_, view_);
  }

  return;
}

void TaskDispatcher::dispatchTaskEstimateThickness(void)
{
  if (!estimate_thickness_tasks_.isEmpty())
  {
    QMessageBox::warning(MainWindow::getInstance(), "Estimate Thickness Task Warning",
      "Run estimate thickness task after the previous one has finished");
    return;
  }

  int start_frame, end_frame;
  double thickness_radius;
  if (!ParameterManager::getInstance().getEstimateThicknessParameters(thickness_radius, start_frame, end_frame))
    return;

  for (int frame = start_frame; frame <= end_frame; frame ++)
      estimate_thickness_tasks_.push_back(Task(new TaskEstimateThickness(frame, 12, thickness_radius)));

  runTasks(estimate_thickness_tasks_, "Estimate Thickness");
}

TaskEstimateOrientation::TaskEstimateOrientation(int frame, int view, double orientation_radius)
  :TaskImpl(frame, view),orientation_radius_(orientation_radius)
{}

TaskEstimateOrientation::~TaskEstimateOrientation(void)
{}

void TaskEstimateOrientation::run(void) const
{
  FileSystemModel* model = MainWindow::getInstance()->getFileSystemModel();

  osg::ref_ptr<PointCloud> point_cloud = model->getPointCloud(frame_, view_);
  if (point_cloud.valid())
  {
    point_cloud->estimateOrientation(orientation_radius_);
    point_cloud->save(point_cloud->getFilename());
    model->updatePointCloud(frame_, view_);
  }

  return;
}

void TaskDispatcher::dispatchTaskEstimateOrientation(void)
{
  if (!estimate_orientation_tasks_.isEmpty())
  {
    QMessageBox::warning(MainWindow::getInstance(), "Estimate Orientation Task Warning",
      "Run estimate orientation task after the previous one has finished");
    return;
  }

  int start_frame, end_frame;
  double orientation_radius;
  if (!ParameterManager::getInstance().getEstimateOrientationParameters(orientation_radius, start_frame, end_frame))
    return;

  for (int frame = start_frame; frame <= end_frame; frame ++)
    estimate_orientation_tasks_.push_back(Task(new TaskEstimateOrientation(frame, 12, orientation_radius)));

  runTasks(estimate_orientation_tasks_, "Estimate Orientation");
}