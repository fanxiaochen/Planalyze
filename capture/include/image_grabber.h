#pragma once
#ifndef IMAGE_GRABBER_H
#define IMAGE_GRABBER_H

#include <vector>
#include <QMutex>
#include <QString>
#include <QObject>
#include <FlyCapture2.h>
#include "parameter.h"

class MainWindow;

class ImageGrabber : public QObject
{
  Q_OBJECT

public:
  ImageGrabber(void);
  ~ImageGrabber(void);

  bool init(size_t stripe_num, size_t view_num);
  void stop(void);

public slots:
  void grabSnapshot(int view);
  void grabStripe(int view, int stripe);
  void save(const QString& image_folder, const QString& points_folder);

signals:
  void snapshotGrabbed(void);
  void stripeGrabbed(int stripe);
  void timeToView(int view, int stripe);

public:
  DoubleParameter               stripe_shutter_;
  DoubleParameter               snapshot_shutter_;
  DoubleParameter               stripe_gain_;
  DoubleParameter               snapshot_gain_;

  std::vector<std::vector<FlyCapture2::Image> > frame_images_;
  QImage                          current_image_;

private:
  FlyCapture2::Camera     *camera_;
  FlyCapture2::PGRGuid    *guid_;
  QMutex                  mutex_;

  bool connect();
  bool disconnect();
  bool setCamera(bool is_snapshot);
  void grabImpl(size_t view, size_t stripe);
};

#endif /*IMAGE_GRABBER_H*/