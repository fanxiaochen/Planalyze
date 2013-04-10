#pragma once
#ifndef TASK_DISPATCHER_H_
#define TASK_DISPATCHER_H_

#include <vector>
#include <QMutex>
#include <QObject>
#include <boost/shared_ptr.hpp>

#include "povray_visitor.h"
#include "point_cloud.h"

class TaskImpl
{
public:
  TaskImpl(int frame, int view);
  virtual ~TaskImpl(void);

  virtual void run(void) const = 0;

protected:
  friend class Task;
  int frame_;
  int view_;
};

class Task : public QObject
{
  Q_OBJECT

public:
  Task(void);
  Task(TaskImpl* task_impl);
  Task(const Task &other);
  Task& operator=(const Task &other);
  virtual ~Task(void);

  bool run(void) const;

signals:
  void finished(int frame, int view) const;

private:
  boost::shared_ptr<TaskImpl> task_impl_;
};

class TaskVirtualScan : public TaskImpl
{
public:
  TaskVirtualScan(int frame, double noise, double distance, double resolution);
  virtual ~TaskVirtualScan();

  virtual void run(void) const;

private:
  double noise_;
  double distance_;
  double resolution_;
};

class TaskExtractPoints : public TaskImpl
{
public:
  TaskExtractPoints(int frame, const QString& root_folder, int downsampling);
  virtual ~TaskExtractPoints();

  virtual void run(void) const;

private:
  QString       root_folder_;
  int           downsampling_;
};

class TaskExtractPlant : public TaskImpl
{
public:
  TaskExtractPlant(int frame, int delta);
  virtual ~TaskExtractPlant();

  virtual void run(void) const;

private:
  int             delta_;
};

class TaskExtractKeyFrames : public TaskImpl
{
public:
  TaskExtractKeyFrames(int frame, int offset, int start_frame, bool is_point, QString root_folder_);
  virtual ~TaskExtractKeyFrames();

  virtual void run(void) const;

private:
  int             offset_;
  int             start_frame_;
  bool            is_point_;
  QString         root_folder_;
};

class TaskRotateCloud : public TaskImpl
{
public:
  TaskRotateCloud(int frame, int angle);
  virtual ~TaskRotateCloud();

  virtual void run(void) const;

private:
  int             angle_;
};

class TaskConvertPcd : public TaskImpl
{
public:
  TaskConvertPcd(int frame);
  virtual ~TaskConvertPcd();

  virtual void run(void) const;

};

class TaskRemoveErrorPoints : public TaskImpl
{
public:
  TaskRemoveErrorPoints(int frame, int radius);
  virtual ~TaskRemoveErrorPoints();

  virtual void run(void) const;

private:
  int radius_;

};

class TaskRenameFrames : public TaskImpl
{
public:
  TaskRenameFrames(int frame, int rename_offset);
  virtual ~TaskRenameFrames();
 
  virtual void run(void) const;
 
private:
  int rename_offset_;
};


class TaskRenameViews : public TaskImpl
{
public:
  TaskRenameViews(int frame, int rename_offset);
  virtual ~TaskRenameViews();

  virtual void run(void) const;

private:
  int rename_offset_;
};

class TaskRemoveBadFrames : public TaskImpl
{
public:
  TaskRemoveBadFrames(int view, int start_frame, int end_frame);
  virtual ~TaskRemoveBadFrames();

  virtual void run(void) const;

private:
  int start_frame_;
  int end_frame_;
};

class TaskGeneratePovrayData : public TaskImpl
{
public:
  TaskGeneratePovrayData(int frame, PovRayVisitor* master, QString root_folder);
  virtual ~TaskGeneratePovrayData();

  virtual void run(void) const;

private:
  PovRayVisitor* master_;
  mutable QMutex   mutex_;
  QString root_folder_;
};



class TaskPointsGeneration : public TaskImpl
{
public:
  TaskPointsGeneration(int frame, int view, int ctr_threshold, int sat_threshold);
  virtual ~TaskPointsGeneration();

  static QString getExeFilename(void);
  virtual void run(void) const;

private:
  int ctr_threshold_;
  int sat_threshold_;

  void convertImages(void) const;
  void deleteImages(void) const;
  void colorizePoints(void) const;
};

class TaskRegistration : public TaskImpl
{
public:
  TaskRegistration(int frame, int segment_threshold, int max_iterations, double max_distance);
  virtual ~TaskRegistration();

  virtual void run(void) const;

private:
  int segment_threshold_;
  int max_iterations_;
  double max_distance_;
};

class TaskEstimateNormal : public TaskImpl
{
public:
  TaskEstimateNormal(int frame, int view, double normal_radius);
  virtual ~TaskEstimateNormal();

  virtual void run(void) const;

private:
  double normal_radius_;
};

class TaskEstimateThickness : public TaskImpl
{
public:
  TaskEstimateThickness(int frame, int view, double thickness_radius);
  virtual ~TaskEstimateThickness();

  virtual void run(void) const;

private:
  double thickness_radius_;
};

class TaskEstimateOrientation : public TaskImpl
{
public:
  TaskEstimateOrientation(int frame, int view, double orientation_radius);
  virtual ~TaskEstimateOrientation();

  virtual void run(void) const;

private:
  double orientation_radius_;
};

class TaskDispatcher : public QObject
{
  Q_OBJECT

public:
  TaskDispatcher(QObject* parent=0);
  virtual ~TaskDispatcher(void);

  bool isRunning(void) const;

public slots:
  void cancelRunningTasks(bool wait=false);
  void dispatchTaskVirtualScan(void);
  void dispatchTaskRenameViews(void);
  void dispatchTaskRenameFrames(void);
  void dispatchTaskExtractPoints(void);
  void dispatchTaskPointsGeneration(void);
  void dispatchTaskRegistration(void);
  void dispatchTaskEstimateNormal(void);
  void dispatchTaskEstimateThickness(void);
  void dispatchTaskEstimateOrientation(void);
  void dispathcTaskGeneratePovrayData(void);
  void dispathcTaskRemoveBadFrames(void);
  void dispathcTaskExtractPlant(void);
  void dispathcTaskExtractKeyFrames(void);
  void dispathcTaskRotateCloud(void);
  void dispathcTaskConvertPcd(void);
  void dispathcTaskRemoveErrorPoints(void);
  void updateDisplayQueue(int frame, int view);
  void clearDisplayQueue(void);
  void removeFinishedWatchers(void);
  void finishPovRayMaster(void);

protected:
  void runTasks(QList<Task>& tasks, const QString& task_name, bool display = true);

protected slots:

private:
  QList<Task>                         virtual_scan_tasks_;
  QList<Task>                         rename_views_tasks_;
  QList<Task>                         rename_frames_tasks_;
  QList<Task>                         extract_points_tasks_;
  QList<Task>                         points_generation_tasks_;
  QList<Task>                         registration_tasks_;
  QList<Task>                         estimate_normal_tasks_;
  QList<Task>                         estimate_thickness_tasks_;
  QList<Task>                         estimate_orientation_tasks_;
  QList<Task>                         generate_povray_data_tasks_;
  QList<Task>                         remove_bad_frames_tasks_;
  QList<Task>                         extract_plant_tasks_;
  QList<Task>                         extract_key_frames_;
  QList<Task>                         rotate_cloud_tasks_;
  QList<Task>                         convert_pcd_tasks_;
  QList<Task>                         remove_error_points_tasks_;

  std::vector<QObject*>               active_watchers_;
  typedef std::list<std::pair<int, int> > DisplayQueue;
  DisplayQueue                        display_queue_;

  int                              start_frame_;
  int                              end_frame_;
  int                              frame_delta_;
  int                              frame_multiple_;
  int                              camera_number_;
  int                              stop_delta_;
  int                              camera_delta_;

  QString                          root_folder_;

  PovRayVisitor                    master_;

  mutable QMutex                      mutex_;
};

#endif /*TASK_DISPATCHER_H_*/