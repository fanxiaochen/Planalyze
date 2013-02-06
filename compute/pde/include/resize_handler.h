#pragma once
#ifndef RESIZE_HANDLER_H
#define RESIZE_HANDLER_H

#include <osgViewer/ViewerEventHandlers>

class Information;

class ResizeHandler : public osgGA::GUIEventHandler
{
public:
  ResizeHandler(Information* information);
  ~ResizeHandler(void);

  bool handle(const osgGA::GUIEventAdapter& ea,osgGA::GUIActionAdapter& aa);
protected:
  osg::ref_ptr<Information>    information_;
};

#endif // ShowAxesHandlers_H