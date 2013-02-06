#include <iostream>

#include <QDir>
#include <QMutexLocker>
#include <QApplication>
#include <QElapsedTimer>

#include <boost/thread/thread.hpp>

#include "main_window.h"
#include "pattern_projector.h"

#include "image_grabber.h"

using namespace FlyCapture2;

void PrintError( Error error, const std::string& stage)
{
  std::cout << std::endl;
  std::cout << "Camera error in stage: [" << stage << "]" << std::endl;
  error.PrintErrorTrace();
  std::cout << std::endl;
}

ImageGrabber::ImageGrabber(void)
  :camera_(NULL),
  guid_(NULL),
  stripe_shutter_("Stripe Shutter", "Camera shutter value for the stripes", 30, 0.1, 60, 0.1),
  snapshot_shutter_("Snapshot Shutter", "Camera shutter value for the snapshot", 60, 0.1, 60, 0.1),
  stripe_gain_("Stripe Gain", "Camera gain value for the stripes", 1.0, 0.1, 18.1, 0.1),
  snapshot_gain_("Snapshot Gain", "Camera gain value for the snapshot", 2.0, 0.1, 18.1, 0.1)
{
}

ImageGrabber::~ImageGrabber(void)
{
  if (camera_->IsConnected())
  {
    camera_->StopCapture();
    camera_->Disconnect();
  }

  delete camera_;
  delete guid_;
}

void ImageGrabber::stop(void)
{
  QMutexLocker locker(&mutex_);
  QThread::currentThread()->quit();
}

bool ImageGrabber::init(size_t stripe_num, size_t view_num)
{
  QMutexLocker locker(&mutex_);

  Error error;

  BusManager bus_manager;
  unsigned int num_cameras;
  error = bus_manager.GetNumOfCameras(&num_cameras);
  if (error != PGRERROR_OK)
  {
    PrintError(error, "Init");
    return false;
  }

  if (num_cameras != 1)
    return false;

  guid_ = new PGRGuid();
  error = bus_manager.GetCameraFromIndex(0, guid_);
  if (error != PGRERROR_OK)
  {
    PrintError(error, "Init");
    return false;
  }

  frame_images_.assign(view_num, std::vector<Image>(stripe_num+1, Image()));

  camera_ = new Camera();
  connect();
  FC2Config config;
  camera_->GetConfiguration(&config);
  config.grabMode = DROP_FRAMES;
  config.numBuffers = 64;
  camera_->SetConfiguration(&config);
  disconnect();

  return true;
}

bool ImageGrabber::setCamera(bool is_snapshot)
{
  Error error;

  VideoMode video_mode = is_snapshot?VIDEOMODE_1280x960RGB:VIDEOMODE_1280x960Y8;
  FrameRate frame_rate = is_snapshot?FRAMERATE_15:FRAMERATE_30;
  camera_->SetVideoModeAndFrameRate (video_mode, frame_rate);

  Property gain_property(GAIN);
  gain_property.absControl = true;
  gain_property.onePush = false;
  gain_property.onOff = true;
  gain_property.autoManualMode = false;
  gain_property.absValue = is_snapshot?snapshot_gain_:stripe_gain_;
  error = camera_->SetProperty(&gain_property);
  if (error != PGRERROR_OK)
  {
    PrintError(error, "SetGain");
    return false;
  }

  Property shutter_property(SHUTTER);
  shutter_property.absControl = true;
  shutter_property.onePush = false;
  shutter_property.onOff = true;
  shutter_property.autoManualMode = false;
  shutter_property.absValue = is_snapshot?snapshot_shutter_:stripe_shutter_;
  error = camera_->SetProperty(&shutter_property);
  if (error != PGRERROR_OK)
  {
    PrintError(error, "SetShutter");
    return false;
  }

  return true;
}

void ImageGrabber::grabImpl(size_t view, size_t stripe)
{
  bool is_snapshot = (stripe == frame_images_[view].size()-1);

  QElapsedTimer timer;
  timer.start();

  if (is_snapshot)
    setCamera(true);
  else if (stripe == 0)
    setCamera(false);
  int set_camera_time = timer.restart();

  boost::this_thread::sleep(boost::posix_time::milliseconds(200));
  int sleep_time = timer.restart();

  Error error = camera_->StartCapture();
  if (error != PGRERROR_OK)
    PrintError(error, "StartCapture");
  int start_capture_time = timer.restart();

  Image image;
  error = camera_->RetrieveBuffer(&image);
  if (error != PGRERROR_OK)
    PrintError(error, "RetrieveBuffer");
  int retrieve_buffer_time = timer.restart();

  PixelFormat format = (is_snapshot)?(PIXEL_FORMAT_RGB):(PIXEL_FORMAT_MONO8);
  error = image.Convert(format, &frame_images_[view][stripe]);
  if (error != PGRERROR_OK)
    PrintError(error, "Convert");
  int conver_image_time = timer.restart();

  error = camera_->StopCapture();
  if (error != PGRERROR_OK)
    PrintError(error, "StopCapture");
  int stop_capture_time = timer.restart();

  QString message = QString("Total[%0]  SetCamera[%1]  Sleep[%2]  Start[%3]  Retrieve[%4]  Convert[%5]  Stop[%6]")
    .arg(set_camera_time+sleep_time+start_capture_time+retrieve_buffer_time+conver_image_time+stop_capture_time, 3, 10)
    .arg(set_camera_time, 2, 10).arg(sleep_time, 2, 10).arg(start_capture_time, 2, 10)
    .arg(retrieve_buffer_time, 2, 10).arg(conver_image_time, 2, 10).arg(stop_capture_time, 2, 10);

  //std::cout << (is_snapshot?"\n":"") << message.toStdString() << std::endl;

  return;
}

void ImageGrabber::grabSnapshot(int view)
{
  connect();

  size_t stripe_num = frame_images_[view].size()-1;
  grabImpl(view, stripe_num);
  emit snapshotGrabbed();

  return;
}

void ImageGrabber::grabStripe(int view, int stripe)
{
  grabImpl(view, stripe);
  emit stripeGrabbed(stripe);

  bool is_last_stripe = (stripe == frame_images_[view].size()-2);
  if (is_last_stripe)
    disconnect();

  return;
}

void ImageGrabber::save(const QString& images_folder, const QString& points_folder)
{
  QMutexLocker locker(&mutex_);

  QDir images_dir(images_folder);
  QDir points_dir(points_folder);

  Error error;
  for (size_t i = 0, i_end = frame_images_.size(); i < i_end; ++ i)
  {
    QString images_folder_view = QString("%1/view_%2").arg(images_folder).arg(i, 2, 10, QChar('0'));
    QString points_folder_view = QString("%1/view_%2").arg(points_folder).arg(i, 2, 10, QChar('0'));
    images_dir.mkpath(images_folder_view);
    points_dir.mkpath(points_folder_view);

    for (size_t j = 0, j_end = frame_images_[i].size(); j < j_end; ++ j)
    {
      std::string filename = QString("%1/image_%2.jpg").arg(images_folder_view).arg(j, 2, 10, QChar('0')).toStdString();
      if (j == j_end-1)
        filename = points_folder_view.toStdString()+"/snapshot.jpg";

      JPEGOption jpeg_option;
      jpeg_option.progressive = false;
      jpeg_option.quality = 100;

      error = frame_images_[i][j].Save(filename.c_str(), &jpeg_option);
      if (error != PGRERROR_OK)
        PrintError(error, "Save");

      emit timeToView(i, j);
      boost::this_thread::sleep(boost::posix_time::milliseconds(50));
    }
    boost::this_thread::sleep(boost::posix_time::milliseconds(100));
  }

  return;
}

bool ImageGrabber::connect()
{
  Error error;

  // Connect to a camera
  error = camera_->Connect(guid_);
  if (error != PGRERROR_OK)
  {
    PrintError(error, "Connect");
    return false;
  }

  return true;
}

bool ImageGrabber::disconnect()
{
  Error error;

  // Disconnect the camera
  error = camera_->Disconnect();
  if (error != PGRERROR_OK)
  {
    PrintError(error, "Disconnect");
    return false;
  }

  return true;
}