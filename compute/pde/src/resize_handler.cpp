#include "information.h"
#include "resize_handler.h"

ResizeHandler::ResizeHandler(Information* information)
  :information_(information)
{
}

ResizeHandler::~ResizeHandler(void)
{
}

bool ResizeHandler::handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
{
  osgGA::GUIEventAdapter::EventType ev = ea.getEventType();
  if(ev != osgGA::GUIEventAdapter::RESIZE) return false;

  information_->resize(ea.getWindowWidth(), ea.getWindowHeight());

  return true;
}