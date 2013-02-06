#pragma once
#ifndef MAIN_WINDOW_H
#define MAIN_WINDOW_H

#include <QThread>
#include <QMainWindow>

#include "ui_main_window.h"

#include "parameter.h"

class TurnTable;
class ImageViewer;
class ImageGrabber;
class PlainTextViewer;
class PatternProjector;

class MainWindow : public QMainWindow
{
  Q_OBJECT

public:
  MainWindow(QWidget *parent = 0, Qt::WFlags flags = 0);
  ~MainWindow();

public slots:
  void onImagesGrabbed(void);
  void showImageMessage(const QString& image);
  void viewImage(int view, int stripe);

signals:
  void timeToGrab(int current_view, double time);
  void timeToSave(const QString& image_folder, const QString& points_folder);

protected:
  virtual void timerEvent(QTimerEvent *event);
  virtual void closeEvent(QCloseEvent* event);

private:
  QString getImagesFolder();
  QString getPointsFolder();
  void saveParameters();
  void saveStatusLog();
  void showStatusMessage(const QString& message);
  void loadGlobalSettings();
  void saveGlobalSettings();

private slots:
  void recordWaterEvent(void);
  void startCapture(void);
  void stopCapture(void);

private:
  Ui::EvoGeoCaptureClass  ui_;
  TurnTable               *turn_table_;
  ImageViewer             *image_viewer_;
  ImageGrabber            *image_grabber_;
  PlainTextViewer         *plain_text_viewer_;
  PatternProjector        *pattern_projector_;

  int                     current_frame_;
  int                     current_view_;

  int                     frame_timer_id_;
  int                     view_timer_id_;

  IntParameter            frame_time_;
  IntParameter            start_frame_;
  DoubleParameter            view_time_;

  QString                 base_folder_;

  QThread                 grabber_thread_;
};

#endif // MAIN_WINDOW_H
