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
  boost::SkeletonGraph* getSkeletonGraph(void) {return skeleton_graph_;}

public slots:
  void toggle(bool toggled);

protected:
  virtual void updateImpl(void);

  void jointSkeleton(osgViewer::View* view, const osgGA::GUIEventAdapter& ea);
  void breakSkeleton(osgViewer::View* view, const osgGA::GUIEventAdapter& ea);

private:
  osg::ref_ptr<osg::Vec3Array>  current_path_;
  boost::SkeletonGraph*         skeleton_graph_;
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