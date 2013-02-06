#include <sstream>
#include <iomanip>
#include <iostream>
#include <QDateTime>
#include <QMutexLocker>
#include <QApplication>
#include <QDesktopWidget>

#include "pattern_projector.h"

PatternProjector::PatternProjector()
  :QGLWidget(),
  stripe_set_("Stripe Set", "Stripe set to use", "1024x768x30", std::map<std::string, std::string>()),
  view_(0),
  time_(7.5*1000),
  current_stripe_(-1),
  rendering_stripe_(-1)
{
  std::map<std::string, std::string> strip_map;
  strip_map["800x600x30"] = "800x600x30";
  strip_map["1024x768x24"] = "1024x768x24";
  strip_map["1024x768x30"] = "1024x768x30";
  stripe_set_.setCadidates(strip_map);

  setAutoBufferSwap(false);

  qglClearColor(QColor(0, 0, 0, 255));

  //about the painful screen tearing...
  //http://en.wikipedia.org/wiki/Screen_tearing
  //http://answers.yahoo.com/question/index?qid=20090711205848AAa2h7s
  //http://blog.qt.digia.com/2010/12/02/velvet-and-the-qml-scene-graph/
}

PatternProjector::~PatternProjector(void)
{
}

bool PatternProjector::init(void)
{
  int screen_count = QApplication::desktop()->screenCount();
  if (screen_count != 2)
    return false;

  std::stringstream sin(stripe_set_);
  int width = 0;
  int height = 0;
  int stripe_number = 0;
  char x;
  sin >> width >> x >> height >> x >> stripe_number;

  QRect screen_res = QApplication::desktop()->screenGeometry(1);
  if (screen_res.width() != width || screen_res.height() != height)
    return false;

  stripes_.clear();
  for (int i = 0; i < stripe_number; i ++)
  {
    QString url = QString(":/stripes/%1/%2.bmp").arg(std::string(stripe_set_).c_str()).arg(i, 2, 10, QChar('0'));
    stripes_.push_back(convertToGLFormat(QImage(url)));
  }

  setGeometry(0, 0, width, height);
  move(QPoint(screen_res.x(), screen_res.y()));
  showFullScreen();

  return true;
}

void PatternProjector::paintEvent(QPaintEvent *event)
{
  // bypass the paint from paint event
}

void PatternProjector::paintGL(void)
{
  //std::cout << QDateTime::currentDateTime().toString("mm:ss:zzz").toStdString() << std::endl;

  if (current_stripe_ >= 0)
  {
    QImage& stripe = stripes_[current_stripe_];
    glDrawPixels(stripe.width(), stripe.height(), GL_RGBA, GL_UNSIGNED_BYTE, stripe.bits());
    swapBuffers();
    glDrawPixels(stripe.width(), stripe.height(), GL_RGBA, GL_UNSIGNED_BYTE, stripe.bits());
    swapBuffers();
  }

  return;
}

void PatternProjector::projectSnapshot(int view, double time)
{
  view_ = view;
  time_ = time;
  timer_.start();
  current_stripe_ = 0;

  updateGL();

  emit timeToGrabSnapshot(view);
  emit timeToShowImageMessage("snapshot");

  return;
}

void PatternProjector::projectFirstStripe()
{
  current_stripe_ = 0;

  updateGL();

  emit timeToGrabStripe(view_, current_stripe_);
  emit timeToShowImageMessage("stripe_00");

  return;
}

void PatternProjector::projectNextStripe(int stripe)
{
  if (stripe != current_stripe_)
    std::cout << "Warning:\tThe camera and projector is not synchronized!" << std::endl;

  current_stripe_ ++;
  if (current_stripe_ >= stripes_.size())
  {
    current_stripe_ = 1;

    updateGL();

    double capture_time = timer_.elapsed();
    double rotation_time = time_*1000-capture_time;
    emit timeToRotate(rotation_time);
    emit imagesGrabbed();

    return;
  }

  updateGL();

  emit timeToGrabStripe(view_, current_stripe_);
  QString message = QString("stripe_%1%2")
    .arg(current_stripe_, 2, 10, QChar('0')).arg((current_stripe_==stripes_.size()-1)?"\n":"");
  emit timeToShowImageMessage(message);

  return;
}