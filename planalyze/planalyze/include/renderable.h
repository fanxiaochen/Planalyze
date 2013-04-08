#pragma once
#ifndef Renderable_H
#define Renderable_H

#include <QMutex>
#include <osg/MatrixTransform>

class PovRayVisitor;

class Renderable : public osg::MatrixTransform
{
public:
  Renderable(void);
  virtual ~Renderable(void);

  virtual const char* className() const {return "Renderable";}
  inline bool isHidden(void) const {return hidden_;}

  virtual void toggleRendering(void);
  void expire();
  void update();

  virtual void povray(PovRayVisitor* povray_visitor) const {}

protected:
  virtual void clear();
  virtual void updateImpl() = 0;

protected:
  QMutex  mutex_;
  bool    hidden_;

private:
  bool    expired_;
};

#endif // Renderable_H