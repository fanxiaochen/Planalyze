#pragma once
#ifndef SNAPSHOT_HANDLER_H
#define SNAPSHOT_HANDLER_H

#include <osgViewer/View>
#include <osgViewer/ViewerEventHandlers>

class WriteToFile : public osgViewer::ScreenCaptureHandler::WriteToFile
{
public:

  WriteToFile(osgViewer::ScreenCaptureHandler::WriteToFile::SavePolicy savePolicy = SEQUENTIAL_NUMBER);
  virtual ~WriteToFile(void);

  virtual void operator() (const osg::Image& image, const unsigned int context_id);
};

class RecordCameraPathHandler : public osgViewer::RecordCameraPathHandler
{
public:
  RecordCameraPathHandler(void);
  virtual ~RecordCameraPathHandler(void);

  bool handle(const osgGA::GUIEventAdapter &ea, osgGA::GUIActionAdapter &aa);
};

#endif // SNAPSHOT_HANDLER_H