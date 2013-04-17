#include <QDir>
#include <QSlider>
#include <QComboBox>
#include <QSettings>
#include <QDateTime>
#include <QDockWidget>
#include <QFileDialog>
#include <QApplication>
#include <QMessageBox>

#include <FlyCapture2.h>

#include "turn_table.h"
#include "image_viewer.h"
#include "image_grabber.h"
#include "parameter_dialog.h"
#include "plain_text_viewer.h"
#include "pattern_projector.h"

#include "point_cloud.h"
#include "registrator.h"
#include "information.h"
#include "axis_indicator.h"
#include "toggle_handler.h"
#include "resize_handler.h"
#include "task_dispatcher.h"
#include "sketch_handler.h"
#include "parameter_manager.h"
#include "osg_viewer_widget.h"
#include "file_system_model.h"
#include "file_viewer_widget.h"
#include "statistics_viewer_widget.h"

#include "main_window.h"

MainWindow::MainWindow(void)
  :registrator_(new Registrator),
  axis_indicator_(new AxisIndicator),
  osg_viewer_widget_(NULL),
  file_viewer_widget_(NULL),
  statistics_viewer_widget_(NULL),
    skeleton_sketcher_(NULL),
  information_(NULL),
  task_dispatcher_(new TaskDispatcher(this)),
  color_mode_(new QComboBox(this)),
  master_(new PovRayVisitor()),
  workspace_("."),
  turn_table_(new TurnTable),
  image_viewer_(new ImageViewer),
  image_grabber_(new ImageGrabber(this)),
  plain_text_viewer_(new PlainTextViewer),
  pattern_projector_(new PatternProjector),
  current_frame_(0),
  current_view_(0),
  frame_timer_id_(0),
  view_timer_id_(0),
  frame_time_("Frame Period", "The time gap between frames", 300, 120, 1200, 60),
  start_frame_("Start Frame", "Which frame index to start with", 0, 0, 10000, 1),
  view_time_("View Time", "How long is allocated for each view", 15, 10, 20, 0.5)
{
  ui_.setupUi(this);
  color_mode_->addItem("Original", PointCloud::ORIGINAL);
  color_mode_->addItem("Leaf/Stem", PointCloud::LABEL);
  color_mode_->addItem("Organ", PointCloud::ORGAN);
  color_mode_->addItem("Thickness", PointCloud::THICKNESS);
  color_mode_->addItem("Flatness", PointCloud::FLATNESS);
  color_mode_->addItem("Segment", PointCloud::SEGMENT);
  color_mode_->addItem("Probability", PointCloud::PROBABILITY);
  color_mode_->addItem("Uniform", PointCloud::UNIFORM);
  ui_.mainToolBar->addWidget(color_mode_);

  MainWindowInstancer::getInstance().main_window_ = this;
  ParameterManager::getInstance();

  init();

  connect(ui_.actionStartCapture, SIGNAL(triggered()), this, SLOT(startCapture()));
  connect(ui_.actionStopCapture, SIGNAL(triggered()), this, SLOT(stopCapture()));
  connect(ui_.actionWaterPlant, SIGNAL(triggered()), this, SLOT(recordWaterEvent()));

  connect(this, SIGNAL(timeToGrab(int, double)), pattern_projector_, SLOT(projectSnapshot(int, double)));
  connect(this, SIGNAL(timeToSave(QString, QString)), image_grabber_, SLOT(save(QString, QString)));

  connect(pattern_projector_, SIGNAL(timeToGrabSnapshot(int)), image_grabber_, SLOT(grabSnapshot(int)));
  connect(image_grabber_, SIGNAL(snapshotGrabbed()), pattern_projector_, SLOT(projectFirstStripe()));

  connect(pattern_projector_, SIGNAL(timeToGrabStripe(int, int)), image_grabber_, SLOT(grabStripe(int, int)));
  connect(image_grabber_, SIGNAL(stripeGrabbed(int)), pattern_projector_, SLOT(projectNextStripe(int)));

  connect(pattern_projector_, SIGNAL(timeToShowImageMessage(QString)), this, SLOT(showImageMessage(QString)));
  connect(pattern_projector_, SIGNAL(timeToRotate(double)), turn_table_, SLOT(rotate(double)));
  connect(pattern_projector_, SIGNAL(imagesGrabbed()), this, SLOT(onImagesGrabbed()));

  connect(image_grabber_, SIGNAL(timeToView(int, int)), this, SLOT(viewImage(int, int)));

  image_grabber_->moveToThread(&grabber_thread_);
  grabber_thread_.start();
}

MainWindow::~MainWindow()
{
  saveSettings();

  image_grabber_->stop();
  grabber_thread_.wait();

  turn_table_->deleteLater();
  image_viewer_->deleteLater();
  image_grabber_->deleteLater();
  plain_text_viewer_->deleteLater();
  pattern_projector_->deleteLater();

  return;
}

void MainWindow::closeEvent(QCloseEvent *event)
{
  task_dispatcher_->cancelRunningTasks(true);

  if(frame_timer_id_ != 0) {
    stopCapture();
    event->ignore();
  }
  else
  {
    pattern_projector_->close();
    saveStatusLog();
    saveSettings();
    event->accept();
  }

  QMainWindow::closeEvent(event);

  return;
}

void MainWindow::keyPressEvent(QKeyEvent * event)
{
  FileSystemModel* model = getFileSystemModel();
  FileSystemModel::NavigationType type = FileSystemModel::SWITCH;
  type = (event->modifiers() == Qt::ControlModifier)?(FileSystemModel::APPEND):(type);
  type = (event->modifiers() == Qt::ShiftModifier)?(FileSystemModel::ERASE):(type);
  switch (event->key())
  {
  case (Qt::Key_Up):
    model->navigateToPreviousFrame(type);
    break;
  case (Qt::Key_Down):
    model->navigateToNextFrame(type);
    break;
  case (Qt::Key_Left):
    model->navigateToPreviousView(type);
    break;
  case (Qt::Key_Right):
    model->navigateToNextView(type);
    break;
  default:
    QMainWindow::keyPressEvent(event);
    break;
  }

  return;
}

MainWindow* MainWindow::getInstance()
{
  if (MainWindowInstancer::getInstance().main_window_ == NULL)
    std::cout << "shit happens!" << std::endl;
  return MainWindowInstancer::getInstance().main_window_;
}

FileSystemModel* MainWindow::getFileSystemModel(void)
{
  return file_viewer_widget_->getFileSystemModel();
}

void MainWindow::init(void)
{
  osg_viewer_widget_ = new OSGViewerWidget(this);
  file_viewer_widget_ = new FileViewerWidget(this);
  statistics_viewer_widget_ = new StatisticsViewerWidget(this);
  information_ = new Information();

  osg_viewer_widget_->addEventHandler(new ToggleHandler(registrator_, 'r', "Toggle Registrator."));
  osg_viewer_widget_->addChild(registrator_, false);

  osg_viewer_widget_->addEventHandler(new ResizeHandler(information_));
  osg_viewer_widget_->addEventHandler(new ToggleHandler(information_, 'i', "Toggle Information."));
  osg_viewer_widget_->addChild(information_, false);

  osg_viewer_widget_->addEventHandler(new ToggleHandler(axis_indicator_, 'x', "Toggle Axis Indicator."));
  osg_viewer_widget_->addChild(axis_indicator_, false);

  skeleton_sketcher_ = new SkeletonSketcher;
  osg_viewer_widget_->addEventHandler(new SketchHandler(skeleton_sketcher_));
  osg_viewer_widget_->addChild(skeleton_sketcher_, false);

  setCentralWidget(osg_viewer_widget_);
  osg_viewer_widget_->startRendering();

  QDockWidget* dock_widget_status_file = new QDockWidget("File Viewer", this);
  addDockWidget(Qt::LeftDockWidgetArea, dock_widget_status_file);
  dock_widget_status_file->setAllowedAreas(Qt::LeftDockWidgetArea|Qt::RightDockWidgetArea);
  ui_.menuView->addAction(dock_widget_status_file->toggleViewAction());
  file_viewer_widget_->setParent(dock_widget_status_file);
  dock_widget_status_file->setWidget(file_viewer_widget_);

  QDockWidget* dock_widget_statistics = new QDockWidget("Statistics", this);
  addDockWidget(Qt::BottomDockWidgetArea, dock_widget_statistics);
  dock_widget_statistics->setAllowedAreas(Qt::BottomDockWidgetArea|Qt::TopDockWidgetArea);
  ui_.menuView->addAction(dock_widget_statistics->toggleViewAction());
  statistics_viewer_widget_->setParent(dock_widget_statistics);
  dock_widget_statistics->setWidget(statistics_viewer_widget_);

  connect(this, SIGNAL(timeToUpdateStatusMessage(QString)), statusBar(), SLOT(showMessage(QString)));

  FileSystemModel* file_system_model = file_viewer_widget_->getFileSystemModel();
  //file menu
  connect(ui_.actionSetWorkspace, SIGNAL(triggered()), this, SLOT(setWorkspace()));
  connect(ui_.actionParameterMaster, SIGNAL(triggered()), this, SLOT(parameterMaster()));
  connect(ui_.actionLoadParameters, SIGNAL(triggered()), this, SLOT(loadParameters()));
  connect(ui_.actionSaveParameters, SIGNAL(triggered()), this, SLOT(saveParameters()));

  createActionRecentWorkspaces();
  connect(ui_.actionSaveRegistrator, SIGNAL(triggered()), registrator_, SLOT(save()));
  connect(ui_.actionPOVRaySnapshot, SIGNAL(triggered()), osg_viewer_widget_, SLOT(savePOVRaySnapshot()));
  connect(ui_.actionPOVRayAnimation, SIGNAL(triggered()), osg_viewer_widget_, SLOT(savePOVRayAnimation()));
  connect(ui_.actionLoadMeshModel, SIGNAL(triggered()), this, SLOT(loadMeshModel()));
  connect(ui_.actionSaveSceneAsObj, SIGNAL(triggered()), osg_viewer_widget_, SLOT(saveSceneAsObj()));
  connect(ui_.actionLoadCamera, SIGNAL(triggered()), osg_viewer_widget_, SLOT(loadCameraParameters()));
  connect(ui_.actionSaveCamera, SIGNAL(triggered()), osg_viewer_widget_, SLOT(saveCameraParameters()));

  //point_cloud menu
  connect(ui_.actionRegistrationICP, SIGNAL(triggered()), registrator_, SLOT(registrationICP()));
  connect(ui_.actionRegistrationLUM, SIGNAL(triggered()), registrator_, SLOT(registrationLUM()));
  connect(ui_.actionRefineAxis, SIGNAL(triggered()), registrator_, SLOT(refineAxis()));
  connect(ui_.actionSaveRegisteredPoints, SIGNAL(triggered()), registrator_, SLOT(saveRegisteredPoints()));
  connect(ui_.actionForwardClassify, SIGNAL(triggered()), file_system_model, SLOT(forwardClassify()));
  connect(ui_.actionBackwardClassify, SIGNAL(triggered()), file_system_model, SLOT(backwardClassify()));
  connect(ui_.actionFSmoothStems, SIGNAL(triggered()), file_system_model, SLOT(fSmoothStems()));
  connect(ui_.actionBSmoothStems, SIGNAL(triggered()), file_system_model, SLOT(bSmoothStems()));
  connect(ui_.actionFSmoothLeaves, SIGNAL(triggered()), file_system_model, SLOT(fSmoothLeaves()));
  connect(ui_.actionBSmoothLeaves, SIGNAL(triggered()), file_system_model, SLOT(bSmoothLeaves()));
  connect(ui_.actionAbsoluteClassify, SIGNAL(triggered()), file_system_model, SLOT(absoluteClassify()));
  connect(ui_.actionDecomposeLeaves, SIGNAL(triggered()), file_system_model, SLOT(absoluteDetectLeaves()));
  connect(ui_.actionDecomposeStems, SIGNAL(triggered()), file_system_model, SLOT(absoluteDetectStems()));

  //rendering menu
  connect(ui_.actionIncreasePointSize, SIGNAL(triggered()), osg_viewer_widget_, SLOT(increasePointSize()));
  connect(ui_.actionDecreasePointSize, SIGNAL(triggered()), osg_viewer_widget_, SLOT(decreasePointSize()));
  connect(ui_.actionIncreaseLineWidth, SIGNAL(triggered()), osg_viewer_widget_, SLOT(increaseLineWidth()));
  connect(ui_.actionDecreaseLineWidth, SIGNAL(triggered()), osg_viewer_widget_, SLOT(decreaseLineWidth()));
  connect(ui_.actionForceCenterScene, SIGNAL(triggered()), osg_viewer_widget_, SLOT(centerScene()));
  connect(ui_.actionRenderPlant, SIGNAL(toggled(bool)), file_system_model, SLOT(setRenderPlant(bool)));
  connect(ui_.actionRenderPot, SIGNAL(toggled(bool)), file_system_model, SLOT(setRenderPot(bool)));
  connect(ui_.actionRenderNoise, SIGNAL(toggled(bool)), file_system_model, SLOT(setRenderNoise(bool)));
  connect(ui_.actionRenderNormals, SIGNAL(toggled(bool)), file_system_model, SLOT(setRenderNormals(bool)));
  connect(ui_.actionRenderOrientations, SIGNAL(toggled(bool)), file_system_model, SLOT(setRenderOrientations(bool)));
  connect(ui_.actionRenderLeaves, SIGNAL(toggled(bool)), file_system_model, SLOT(setRenderLeaves(bool)));
  connect(ui_.actionRenderStems, SIGNAL(toggled(bool)), file_system_model, SLOT(setRenderStems(bool)));
  connect(ui_.actionRenderTriangles, SIGNAL(toggled(bool)), file_system_model, SLOT(setRenderTriangles(bool)));
  connect(ui_.actionRenderOrgans, SIGNAL(toggled(bool)), file_system_model, SLOT(setRenderOrgans(bool)));
  connect(ui_.actionRenderStemGraph, SIGNAL(toggled(bool)), file_system_model, SLOT(setRenderStemGraph(bool)));
  connect(color_mode_, SIGNAL(currentIndexChanged(int)), file_system_model, SLOT(setColorMode(int)));

  //batch menu
  connect(ui_.actionCancelTasks, SIGNAL(triggered()), task_dispatcher_, SLOT(cancelRunningTasks()));
  connect(ui_.actionBatchVirtualScan, SIGNAL(triggered()), task_dispatcher_, SLOT(dispatchTaskVirtualScan()));
  connect(ui_.actionBatchRenameViews, SIGNAL(triggered()), task_dispatcher_, SLOT(dispatchTaskRenameViews()));
  connect(ui_.actionBatchRenameFrames, SIGNAL(triggered()), task_dispatcher_, SLOT(dispatchTaskRenameFrames()));
  connect(ui_.actionBatchExtractPoints, SIGNAL(triggered()), task_dispatcher_, SLOT(dispatchTaskExtractPoints()));
  connect(ui_.actionGeneratePoints, SIGNAL(triggered()), task_dispatcher_, SLOT(dispatchTaskPointsGeneration()));
  connect(ui_.actionBatchRegistration, SIGNAL(triggered()), task_dispatcher_, SLOT(dispatchTaskRegistration()));
  connect(ui_.actionBatchEstimateNormal, SIGNAL(triggered()), task_dispatcher_, SLOT(dispatchTaskEstimateNormal()));
  connect(ui_.actionBatchEstimateThickness, SIGNAL(triggered()), task_dispatcher_, SLOT(dispatchTaskEstimateThickness()));
  connect(ui_.actionBatchEstimateOrientation, SIGNAL(triggered()), task_dispatcher_, SLOT(dispatchTaskEstimateOrientation()));
  connect(ui_.actionPovRayFrames, SIGNAL(triggered()), task_dispatcher_, SLOT(dispathcTaskGeneratePovrayData()));
  connect(ui_.actionRemoveBadFrames, SIGNAL(triggered()), task_dispatcher_, SLOT(dispathcTaskRemoveBadFrames()));
  connect(ui_.actionExtractPlant, SIGNAL(triggered()), task_dispatcher_, SLOT(dispathcTaskExtractPlant()));
  connect(ui_.actionExtractKeyFrames, SIGNAL(triggered()), task_dispatcher_, SLOT(dispathcTaskExtractKeyFrames()));
  connect(ui_.actionRotateCloud, SIGNAL(triggered()), task_dispatcher_, SLOT(dispathcTaskRotateCloud()));
  connect(ui_.actionConvertPcdToPlainText, SIGNAL(triggered()), task_dispatcher_, SLOT(dispathcTaskConvertPcd()));
  connect(ui_.actionRemoveErrorPoints, SIGNAL(triggered()), task_dispatcher_, SLOT(dispathcTaskRemoveErrorPoints()));

  connect(ui_.actionSkeletonSketcher, SIGNAL(toggled(bool)), skeleton_sketcher_, SLOT(toggle(bool)));


  loadSettings();

  return;
}

void MainWindow::openRecentWorkspace()
{
  QAction *action = qobject_cast<QAction *>(sender());
  if (action) {
    setWorkspace(action->data().toString());
  }

  return;
}

void MainWindow::createActionRecentWorkspaces()
{
  for (int i = 0; i < MaxRecentWorkspaces; ++i) {
    action_recent_workspaces_[i] = new QAction(this);
    action_recent_workspaces_[i]->setVisible(false);
    connect(action_recent_workspaces_[i], SIGNAL(triggered()), this, SLOT(openRecentWorkspace()));
  }

  for (size_t i = 0; i < MaxRecentWorkspaces; ++i) {
    ui_.menuRecentWorkspaces->addAction(action_recent_workspaces_[i]);
  }

  return;
}

void MainWindow::updateCurrentFile(const QString& filename)
{
  QFileInfo fileInfo(filename);
  setWindowTitle(tr("%1[*] - %2").arg(fileInfo.fileName()).arg(tr("Kingfisher")));

  return;
}

void MainWindow::updateRecentWorkspaces()
{
  QMutableStringListIterator it(recent_workspaces_);
  while (it.hasNext()) {
    if (!QFile::exists(it.next()))
      it.remove();
  }

  recent_workspaces_.removeDuplicates();
  int num = (std::min)(MaxRecentWorkspaces, recent_workspaces_.size());
  for (int i = 0; i < num; ++i) {
    QString text = tr("&%1 %2").arg(i + 1).arg(recent_workspaces_[i]);
    action_recent_workspaces_[i]->setText(text);
    action_recent_workspaces_[i]->setData(recent_workspaces_[i]);
    action_recent_workspaces_[i]->setVisible(true);
  }

  for (int i = num; i < MaxRecentWorkspaces; ++ i) {
    action_recent_workspaces_[i]->setVisible(false);
  }

  while (recent_workspaces_.size() > num) {
    recent_workspaces_.removeAt(num);
  }

  return;
}

bool MainWindow::setWorkspace(void)
{
  QString directory;

  bool valid_workspace = false;
  do
  {
    directory = QFileDialog::getExistingDirectory(this,
      tr("Set Workspace"), workspace_,
      QFileDialog::ShowDirsOnly);

    if (directory.isEmpty())
      return false;


    if (directory.contains("frame_"))
      valid_workspace = true;
    else if (directory.contains("points"))
      valid_workspace = true;
    else
    {
      QStringList entries = QDir(directory).entryList();
      bool has_images = false;
      bool has_points = false;
      for (QStringList::const_iterator it = entries.begin(); it != entries.end(); ++ it)
      {
        if (it->contains("images"))
          has_images = true;
        else if (it->contains("points"))
          has_points = true;
        if (has_points && has_points)
        {
          valid_workspace = true;
          break;
        }
      }
    }

  } while (!valid_workspace);


  setWorkspace(directory);

  return true;
}

void MainWindow::setWorkspace(const QString& workspace)
{
  workspace_ = workspace;
  recent_workspaces_.removeAll(workspace);
  recent_workspaces_.prepend(workspace);

  registrator_->reset();
  osg_viewer_widget_->removeChildren(true);
  file_viewer_widget_->setWorkspace(workspace_.toStdString());
  FileSystemModel* file_system_model = getFileSystemModel();
  statistics_viewer_widget_->setAxisScale(QwtPlot::xBottom, file_system_model->getStartFrame(), file_system_model->getEndFrame());

  ParameterManager::getInstance().loadParameters(workspace+"/parameters.xml");

  return;
}

void MainWindow::loadSettings()
{
  QSettings settings("Kingfisher", "Kingfisher");

  recent_workspaces_ = settings.value("recentWorkspaces").toStringList();
  updateRecentWorkspaces();

  workspace_ = settings.value("workspace").toString();

  setWorkspace(workspace_);

  return;
}

void MainWindow::saveSettings()
{
  QSettings settings("Kingfisher", "Kingfisher");
  settings.setValue("recentWorkspaces", recent_workspaces_);
  QString workspace(workspace_);
  settings.setValue("workspace", workspace);

  return;
}

void MainWindow::updateStatusMessage(const QString& message)
{
  emit timeToUpdateStatusMessage(message);

  return;
}

void MainWindow::loadMeshModel(void)
{
  QString folder = QFileDialog::getExistingDirectory(this, tr("Load Mesh Model Files"), this->getWorkspace());
  if (folder.isEmpty())
    return;

  QDir current_folder(folder);
  QStringList entries_list = current_folder.entryList(QStringList("*.obj"));
  if (entries_list.empty())
    return;

  current_folder.mkdir("points");
  QDir points_folder(folder+"/points");
  for (int i = 0, i_end = entries_list.size(); i < i_end; ++ i)
  {
    points_folder.mkdir(QString("frame_%1").arg(i, 5, 10, QChar('0')));
    QString frame_folder(folder+"/points/"+QString("frame_%1").arg(i, 5, 10, QChar('0')));
    QFile::rename(folder+"/"+entries_list[i], frame_folder+"/"+entries_list[i]);
  }

  setWorkspace(folder);

  return;
}

Messenger::Messenger(const QString& running_message, const QString& finished_message, QObject* parent)
  :QObject(parent), running_message_(running_message), finished_message_(finished_message)
{}

Messenger::~Messenger(void)
{}

void Messenger::sendRunningMessage(void)
{
  MainWindow::getInstance()->updateStatusMessage(running_message_);

  return;
}

void Messenger::sendFinishedMessage(void)
{
  MainWindow::getInstance()->updateStatusMessage(finished_message_);
  deleteLater();

  return;
}

void MainWindow::parameterMaster(void)
{
  ParameterManager::getInstance().getAllParameters();
  return;
}

void MainWindow::loadParameters(void)
{
  MainWindow* main_window = MainWindow::getInstance();
  QString filename = QFileDialog::getOpenFileName(main_window, "Load Parameters", main_window->getWorkspace(), "Parameters (*.xml)");
  if (filename.isEmpty())
    return;

  ParameterManager::getInstance().loadParameters(filename);
}

void MainWindow::saveParameters(void)
{
  MainWindow* main_window = MainWindow::getInstance();
  QString filename = QFileDialog::getSaveFileName(main_window, "Save Parameters", main_window->getWorkspace(), "Parameters (*.xml)");
  if (filename.isEmpty())
    return;

  ParameterManager::getInstance().saveParameters(filename);

  std::ofstream fout(workspace_.toStdString() + "/parameters.txt");
  fout << "[Current Time] : " << QDateTime::currentDateTime().toString().toStdString() << std::endl
    << "[Frame Period] : " << frame_time_ << std::endl
    << "[Stripe Set]   : " << std::string(pattern_projector_->stripe_set_) << std::endl
    << "[View Number] : " << turn_table_->view_number_ << std::endl
    << "[View Time]: " << view_time_ << std::endl;
  fout.close();

  return;
}

void MainWindow::timerEvent(QTimerEvent *event)
{
  if (event->timerId() == frame_timer_id_)
  {
    current_frame_ ++;
    current_view_ = 0;

    view_timer_id_ = startTimer(1000*view_time_);
  }
  else if (event->timerId() == view_timer_id_)
  {
    double view_time = 1000*view_time_;
    emit timeToGrab(current_view_, view_time);
  }

  return;
}

void MainWindow::onImagesGrabbed(void)
{
  current_view_ ++;
  if (current_view_ >= turn_table_->view_number_)
  {
    killTimer(view_timer_id_);
    view_timer_id_ = 0;

    emit timeToSave(getImagesFolder(), getPointsFolder());
    saveStatusLog();
  }

  return;
}

QString MainWindow::getImagesFolder()
{
  QString folder = QString("%1/images/frame_%2")
    .arg(workspace_).arg(current_frame_, 5, 10, QChar('0'));

  QDir dir(workspace_);
  dir.mkpath(folder);

  return folder;
}

QString MainWindow::getPointsFolder()
{
  QString folder = QString("%1/points/frame_%2")
    .arg(workspace_).arg(current_frame_, 5, 10, QChar('0'));

  QDir dir(workspace_);
  dir.mkpath(folder);

  return folder;
}

void MainWindow::startCapture()
{
  ParameterDialog parameter_dialog("Capture Parameters", this);
  parameter_dialog.addParameter(&start_frame_);
  parameter_dialog.addParameter(&frame_time_);
  parameter_dialog.addParameter(&(turn_table_->com_port_));
  parameter_dialog.addParameter(&(pattern_projector_->stripe_set_));
  parameter_dialog.addParameter(&(turn_table_->view_number_));
  parameter_dialog.addParameter(&view_time_);
  parameter_dialog.addParameter(&(image_grabber_->stripe_shutter_));
  parameter_dialog.addParameter(&(image_grabber_->snapshot_shutter_));
  parameter_dialog.addParameter(&(image_grabber_->stripe_gain_));
  parameter_dialog.addParameter(&(image_grabber_->snapshot_gain_));

  if(parameter_dialog.exec() != QDialog::Accepted)
    return;

  QString dir = QFileDialog::getExistingDirectory(this, tr("Open Directory"),
    workspace_.isEmpty()?("./"):(workspace_), QFileDialog::ShowDirsOnly|QFileDialog::DontResolveSymlinks);
  if (dir.isEmpty())
    return;
  workspace_ = dir;

  if (!turn_table_->init())
  {
    QMessageBox::warning(this, "Start Capture Warning", "Unable to init turn table!");
    return;
  }

  if (!pattern_projector_->init())
  {
    QMessageBox::warning(this, "Start Capture Warning", "Unable to init pattern projector!");
    return;
  }

  if (!image_grabber_->init(pattern_projector_->getStripeNum(), turn_table_->view_number_))
  {
    QMessageBox::warning(this, "Start Capture Warning", "Unable to init image grabber!");
    return;
  }

  saveParameters();
  ui_.actionStopCapture->setEnabled(true);

  frame_timer_id_ = startTimer(10008*frame_time_);
  view_timer_id_ = startTimer(1000*view_time_);
}

void MainWindow::saveStatusLog()
{
  if (workspace_.isEmpty())
    return;

  QString folder = QString("%1/images/frame_%2").arg(workspace_).arg(current_frame_, 5, 10, QChar('0'));

  std::string filename = folder.toStdString() + "/status_log.txt";
  std::ofstream fout(filename);
  fout << plain_text_viewer_->toPlainText().toStdString();
  fout.close();

  plain_text_viewer_->clear();
}

void MainWindow::stopCapture()
{
  if (frame_timer_id_ != 0)
  {
    killTimer(frame_timer_id_);
    frame_timer_id_ = 0;

    if (current_view_ == 0) {
      ui_.statusBar->showMessage("Stopped...");
      ui_.actionStartCapture->setEnabled(true);
    }
    else
      ui_.statusBar->showMessage("Try to close after the scan of the current frame finishes...");
  }

  ui_.actionStopCapture->setDisabled(true);

  return;
}

void MainWindow::showStatusMessage(const QString& message)
{
  plain_text_viewer_->appendPlainText(message);
  QTextCursor text_cursor = plain_text_viewer_->textCursor();
  text_cursor.movePosition(QTextCursor::End);
  plain_text_viewer_->setTextCursor(text_cursor);

  return;
}

void MainWindow::showImageMessage(const QString& image)
{
  QString message = QString("[%1] : frame_%2-view_%3-%4")
    .arg(QDateTime::currentDateTime().toString())
    .arg(current_frame_, 5, 10, QChar('0'))
    .arg(current_view_, 2, 10, QChar('0'))
    .arg(image);

  showStatusMessage(message);

  if (frame_timer_id_ != 0)
    ui_.statusBar->showMessage("Scanning...");
}

void MainWindow::recordWaterEvent()
{
  QString message = QString("[%1] : Water the poor plants!!!")
    .arg(QDateTime::currentDateTime().toString());

  showStatusMessage(message);
}

void MainWindow::viewImage(int view, int stripe)
{
  FlyCapture2::Image& f_image = image_grabber_->frame_images_[view][stripe];
  bool is_mono = (f_image.GetPixelFormat() == FlyCapture2::PIXEL_FORMAT_MONO8);
  QImage q_image(f_image.GetCols(), f_image.GetRows(), is_mono?QImage::Format_Indexed8:QImage::Format_RGB32);

  if (is_mono)
  {
    QVector<QRgb> color_table;
    for (int i = 0; i < 256; ++ i)
      color_table.push_back(qRgb(i, i, i));
    q_image.setColorTable(color_table);
  }

  for (unsigned int i = 0, i_end = f_image.GetCols(); i < i_end; ++ i)
  {
    for (unsigned int j = 0, j_end = f_image.GetRows(); j < j_end; ++ j)
    {
      if (is_mono)
      {
        uchar gray = *(f_image(j, i));
        q_image.setPixel(i, j, gray);
      }
      else
      {
        uchar r = *(f_image(j, i)+0);
        uchar g = *(f_image(j, i)+1);
        uchar b = *(f_image(j, i)+2);
        q_image.setPixel(i, j, qRgb(r, g, b));
      }
    }
  }

  image_viewer_->setPixmap(QPixmap::fromImage(q_image));

  return;
}