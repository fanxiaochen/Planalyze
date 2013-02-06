#include <sstream>
#include <osg/io_utils>
#include <osgDB/WriteFile>
#include <boost/filesystem.hpp>
#include <boost/system/error_code.hpp>

#include "snapshot_handler.h"
#include "main_window.h"
#include "point_cloud.h"

WriteToFile::WriteToFile(osgViewer::ScreenCaptureHandler::WriteToFile::SavePolicy savePolicy )
  :osgViewer::ScreenCaptureHandler::WriteToFile("screen_shot", "png", savePolicy)
{
}

WriteToFile::~WriteToFile(void)
{

}

void WriteToFile::operator() (const osg::Image& image, const unsigned int context_id)
{
  if (_savePolicy == SEQUENTIAL_NUMBER)
  {
    if (_contextSaveCounter.size() <= context_id)
    {
      unsigned int oldSize = _contextSaveCounter.size();
      _contextSaveCounter.resize(context_id + 1);
      // Initialize all new values to 0 since context ids may not be consecutive.
      for (unsigned int i = oldSize; i <= context_id; i++)
        _contextSaveCounter[i] = 0;
    }
  }

  std::stringstream filename;
  filename << MainWindow::getInstance()->getWorkspace().toStdString() << "/" << _filename << "_" << context_id;

  if (_savePolicy == SEQUENTIAL_NUMBER)
    filename << "_" << _contextSaveCounter[context_id];

  filename << "." << _extension;

  osgDB::writeImageFile(image, filename.str());

  OSG_INFO<<"ScreenCaptureHandler: Taking a screen shot, saved as '"<<filename.str()<<"'"<<std::endl;

  if (_savePolicy == SEQUENTIAL_NUMBER)
  {
    _contextSaveCounter[context_id]++;
  }
}


RecordCameraPathHandler::RecordCameraPathHandler(void)
  :osgViewer::RecordCameraPathHandler()
{
}

RecordCameraPathHandler::~RecordCameraPathHandler(void)
{
}

bool RecordCameraPathHandler::handle(const osgGA::GUIEventAdapter &ea, osgGA::GUIActionAdapter &aa)
{
  _filename = MainWindow::getInstance()->getWorkspaceSafe()+"/camera.path";
  return osgViewer::RecordCameraPathHandler::handle(ea, aa);
}