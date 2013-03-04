#pragma once
#ifndef MainWindow_H
#define MainWindow_H

#include <cassert>
#include <QStringList>
#include <QMainWindow>

#include "ui_main_window.h"

class QComboBox;
class Registrator;
class Information;
class AxisIndicator;
class TaskDispatcher;
class LogViewerWidget;
class FileSystemModel;
class OSGViewerWidget;
class FileViewerWidget;
class PovRayVisitor;
class SkeletonSketcher;

class MainWindow : public QMainWindow
{
  Q_OBJECT

public:
  MainWindow();
  void init(void);
  virtual ~MainWindow();
  static MainWindow* getInstance();

  const Ui::MainWindowClass& getUI(void) const {return ui_;}
  const QComboBox* getColorMode(void) const {return color_mode_;}
  std::string getWorkspaceSafe(void) {return workspace_.toStdString();}
  const QString& getWorkspace(void) const {return workspace_;}
  void setWorkspace(const QString& workspace);

  OSGViewerWidget* getOSGViewerWidget(void) {return osg_viewer_widget_;}
  FileViewerWidget* getFileViewerWidget(void) {return file_viewer_widget_;}
  FileSystemModel* getFileSystemModel(void);
  Registrator* getRegistrator(void) {return registrator_;}
  Information* getInformation(void) {return information_;}
  PovRayVisitor* getPovrayVisitorMaster(void){return master_;}
  std::map<size_t, size_t>& getFrameFrequency(void){return frame_frequency_;}

  void updateStatusMessage(const QString& message);

signals:
  void timeToUpdateStatusMessage(const QString& message);

public slots:
  void updateCurrentFile(const QString& filename);
  bool setWorkspace(void);
  void openRecentWorkspace();
  void parameterMaster(void);
  void loadMeshModel(void);

  void loadParameters(void);
  void saveParameters(void);

protected:
  virtual void closeEvent(QCloseEvent *event);
  virtual void keyPressEvent(QKeyEvent * event);

private:
  void createActionRecentWorkspaces();
  void updateRecentWorkspaces();
  void loadSettings();
  void saveSettings();

private:
  Ui::MainWindowClass             ui_;
  QComboBox*                      color_mode_;
  Registrator*                    registrator_;
  Information*                    information_;
  AxisIndicator*                  axis_indicator_;
  OSGViewerWidget*                osg_viewer_widget_;
  FileViewerWidget*               file_viewer_widget_;
  LogViewerWidget*                log_viewer_widget_;
  TaskDispatcher*                 task_dispatcher_;
  SkeletonSketcher*               skeleton_sketcher_;

  QString                         workspace_;

  PovRayVisitor*                  master_;
  std::map<size_t, size_t>        frame_frequency_;

  static const int                MaxRecentWorkspaces = 10;
  QStringList                     recent_workspaces_;
  QAction*                        action_recent_workspaces_[MaxRecentWorkspaces];
};

class MainWindowInstancer
{
public:
  static MainWindowInstancer& getInstance() {
    static MainWindowInstancer theSingleton;
    return theSingleton;
  }

private:
  MainWindowInstancer():main_window_(NULL){}
  MainWindowInstancer(const MainWindowInstancer &) {}            // copy ctor hidden
  MainWindowInstancer& operator=(const MainWindowInstancer &) {return (*this);}   // assign op. hidden
  virtual ~MainWindowInstancer(){}

  friend class MainWindow;
  MainWindow*   main_window_;
};

class Messenger : public QObject
{
  Q_OBJECT

public:
  Messenger(const QString& running_message, const QString& finished_message, QObject* parent = 0);
  virtual ~Messenger(void);

public slots:
  void sendRunningMessage(void);
  void sendFinishedMessage(void);

private:
  QString running_message_;
  QString finished_message_;
};

#endif // MainWindow_H
