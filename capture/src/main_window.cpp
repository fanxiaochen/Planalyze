#include <fstream>
#include <iostream>

#include <QDir>
#include <QTimer>
#include <QSettings>
#include <QDateTime>
#include <QFileDialog>
#include <QDockWidget>
#include <QMessageBox>
#include <QElapsedTimer>

#include <FlyCapture2.h>

#include "turn_table.h"
#include "image_viewer.h"
#include "image_grabber.h"
#include "parameter_dialog.h"
#include "plain_text_viewer.h"
#include "pattern_projector.h"

#include "main_window.h"

MainWindow::MainWindow(QWidget *parent, Qt::WFlags flags)
  : QMainWindow(parent, flags),
  turn_table_(new TurnTable),
  image_viewer_(new ImageViewer),
  image_grabber_(new ImageGrabber),
  plain_text_viewer_(new PlainTextViewer),
  pattern_projector_(new PatternProjector),
  current_frame_(0),
  current_view_(0),
  frame_timer_id_(0),
  view_timer_id_(0),
  frame_time_("Frame Period", "The time gap between frames", 300, 120, 1200, 60),
  start_frame_("Start Frame", "Which frame index to start with", 0, 0, 10000, 1),
  view_time_("View Time", "How long is allocated for each view", 7.5, 7.5, 15, 0.5)
{
  ui_.setupUi(this);
  ui_.actionStop->setDisabled(true);
  setCentralWidget(image_viewer_);

  QDockWidget* dock_widget_status_log = new QDockWidget("Status Log", this);
  addDockWidget(Qt::RightDockWidgetArea, dock_widget_status_log);
  dock_widget_status_log->setAllowedAreas(Qt::LeftDockWidgetArea|Qt::RightDockWidgetArea);
  ui_.mainToolBar->addAction(dock_widget_status_log->toggleViewAction());
  plain_text_viewer_->setParent(dock_widget_status_log);
  dock_widget_status_log->setWidget(plain_text_viewer_);

  connect(ui_.actionStart, SIGNAL(triggered()), this, SLOT(startCapture()));
  connect(ui_.actionStop, SIGNAL(triggered()), this, SLOT(stopCapture()));
  connect(ui_.actionWater, SIGNAL(triggered()), this, SLOT(recordWaterEvent()));

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

  loadGlobalSettings();
}

MainWindow::~MainWindow()
{
  saveGlobalSettings();

  image_grabber_->stop();
  grabber_thread_.wait();

  turn_table_->deleteLater();
  image_viewer_->deleteLater();
  image_grabber_->deleteLater();
  plain_text_viewer_->deleteLater();
  pattern_projector_->deleteLater();
}

void MainWindow::closeEvent(QCloseEvent* event)
{
  if(frame_timer_id_ != 0) {
    stopCapture();
    event->ignore();
  }
  else
  {
    pattern_projector_->close();
    saveStatusLog();
    saveGlobalSettings();
    event->accept();
  }
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
    base_folder_.isEmpty()?("./"):(base_folder_), QFileDialog::ShowDirsOnly|QFileDialog::DontResolveSymlinks);
  if (dir.isEmpty())
    return;
  base_folder_ = dir;

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
  ui_.actionStop->setEnabled(true);

  frame_timer_id_ = startTimer(5*60*1000);
  view_timer_id_ = startTimer(1000*view_time_);
}

void MainWindow::saveParameters()
{
  std::string filename = base_folder_.toStdString() + "/parameters.txt";
  std::ofstream fout(filename);
  fout << "[Current Time] : " << QDateTime::currentDateTime().toString().toStdString() << std::endl
       << "[Frame Period] : " << frame_time_ << std::endl
       << "[Stripe Set]   : " << std::string(pattern_projector_->stripe_set_) << std::endl
       << "[View Number] : " << turn_table_->view_number_ << std::endl
       << "[View Time]: " << view_time_ << std::endl;
  fout.close();
}

void MainWindow::saveStatusLog()
{
  if (base_folder_.isEmpty())
    return;

  QString folder = QString("%1/images/frame_%2").arg(base_folder_).arg(current_frame_, 5, 10, QChar('0'));

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
      ui_.actionStart->setEnabled(true);
    }
    else
      ui_.statusBar->showMessage("Try to close after the scan of the current frame finishes...");
  }

  ui_.actionStop->setDisabled(true);

  return;
}

QString MainWindow::getImagesFolder()
{
  QString folder = QString("%1/images/frame_%2")
    .arg(base_folder_).arg(current_frame_, 5, 10, QChar('0'));

  QDir dir(base_folder_);
  dir.mkpath(folder);

  return folder;
}

QString MainWindow::getPointsFolder()
{
  QString folder = QString("%1/points/frame_%2")
    .arg(base_folder_).arg(current_frame_, 5, 10, QChar('0'));

  QDir dir(base_folder_);
  dir.mkpath(folder);

  return folder;
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

void MainWindow::loadGlobalSettings()
{
  QSettings global_settings("EvoGeo", "Capture");

  base_folder_ = global_settings.value("base_folder").toString();
}

void MainWindow::saveGlobalSettings()
{
  QSettings global_settings("EvoGeo", "Capture");

  global_settings.setValue("base_folder", base_folder_);
}

void MainWindow::viewImage(int view, int stripe)
{
  FlyCapture2::Image& f_image = image_grabber_->frame_images_[view][stripe];
  bool is_mono = (f_image.GetPixelFormat() == FlyCapture2::PIXEL_FORMAT_MONO8);
  QImage q_image(f_image.GetCols(), f_image.GetRows(), is_mono?QImage::Format_Indexed8:QImage::Format_RGB32);

  if (is_mono)
  {
    QVector<QRgb> color_table;
    for (size_t i = 0; i < 256; ++ i)
      color_table.push_back(qRgb(i, i, i));
    q_image.setColorTable(color_table);
  }

  for (size_t i = 0, i_end = f_image.GetCols(); i < i_end; ++ i)
  {
    for (size_t j = 0, j_end = f_image.GetRows(); j < j_end; ++ j)
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