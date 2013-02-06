#pragma once
#ifndef PATTERN_PROJECTOR_H_
#define PATTERN_PROJECTOR_H_

#include <QMutex>
#include <QGLWidget>
#include <QElapsedTimer>
#include "parameter.h"

class PatternProjector : public QGLWidget
{
  Q_OBJECT

public:
  PatternProjector(void);
  ~PatternProjector(void);

  bool init(void);
  inline size_t getStripeNum(void) {return stripes_.size();}

public:
  EnumParameter<std::string>    stripe_set_;
  std::vector<QImage>           stripes_;

public slots:
  void projectSnapshot(int view, double time);
  void projectFirstStripe();
  void projectNextStripe(int stripe);

signals:
  void timeToGrabSnapshot(int view);
  void timeToGrabStripe(int view, int stripe);
  void timeToShowImageMessage(QString stripe);
  void timeToRotate(double time);
  void imagesGrabbed(void);

protected:
  virtual void paintEvent(QPaintEvent *event);
  virtual void paintGL(void);
  QSize sizeHint() const {return QSize(1024, 768);}

  int           view_;
  double        time_;
  int           current_stripe_;
  int           rendering_stripe_;
  QElapsedTimer timer_;
};

#endif /*PATTERN_PROJECTOR_H_*/