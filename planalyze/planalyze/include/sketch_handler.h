#pragma once
#ifndef sketch_handler_H
#define sketch_handler_H

#include <QObject>
#include <osgViewer/ViewerEventHandlers>

#include "forward.h"
#include "renderable.h"

class Sketcher : public Renderable
{
public:
  Sketcher(void) {}
  virtual ~Sketcher(void) {}

  virtual bool handle(const osgGA::GUIEventAdapter& ea,osgGA::GUIActionAdapter& aa) = 0; 
};

class SkeletonSketcher : public QObject, public Sketcher
{
  Q_OBJECT

public:
  SkeletonSketcher(void);
  virtual ~SkeletonSketcher(void);

  bool handle(const osgGA::GUIEventAdapter& ea,osgGA::GUIActionAdapter& aa);

public slots:
  void toggle(bool toggled);

protected:
  virtual void updateImpl(void);

  void addPoint(osgViewer::View* view, const osgGA::GUIEventAdapter& ea);
  void removeEdge(osgViewer::View* view, const osgGA::GUIEventAdapter& ea);

private:
  osg::ref_ptr<osg::Vec3Array>  current_path_;
};

class SketchHandler : public osgGA::GUIEventHandler
{
public:
  SketchHandler(Sketcher* sketcher);
  virtual ~SketchHandler(void);

  virtual bool handle(const osgGA::GUIEventAdapter& ea,osgGA::GUIActionAdapter& aa);

private:
  osg::ref_ptr<Sketcher>  sketcher_;
};



#endif // sketch_handler_H