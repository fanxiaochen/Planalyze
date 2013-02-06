#include <osg/Depth>
#include "stateset_manipulator.h"


StateSetManipulator::StateSetManipulator(osg::StateSet* stateset)
  :osgGA::StateSetManipulator(stateset),
  _transparency(false),
  _keyEventToggleTransparency('o')
{
}

StateSetManipulator::~StateSetManipulator(void)
{
}

void StateSetManipulator::setTransparencyEnabled(bool transparency)
{
  if (_transparency == transparency) return;

  clone();

  _transparency = transparency;
  if( _transparency )
  {
    // enable transparency
    // Enable blending, select transparent bin.
    _stateset->setMode(GL_BLEND, osg::StateAttribute::ON);
    _stateset->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
    // Enable depth test so that an opaque polygon will occlude a transparent one behind it.
    _stateset->setMode(GL_DEPTH_TEST, osg::StateAttribute::ON);
    // Conversely, disable writing to depth buffer so that
    // a transparent polygon will allow polygons behind it to shine through.
    // OSG renders transparent polygons after opaque ones.
    osg::Depth* depth = new osg::Depth;
    depth->setWriteMask(false);
    _stateset->setAttributeAndModes(depth, osg::StateAttribute::ON);
  }
  else
  {
    _stateset->setRenderingHint(osg::StateSet::OPAQUE_BIN);
    _stateset->setMode(GL_DEPTH_TEST, osg::StateAttribute::ON);
    osg::Depth* depth = new osg::Depth;
    depth->setWriteMask(true);
    _stateset->setAttributeAndModes(depth, osg::StateAttribute::ON);

    _stateset->setMode(GL_BLEND, osg::StateAttribute::OFF);
  }

  return;
}

bool StateSetManipulator::handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
{
  if (!(ea.getEventType()==osgGA::GUIEventAdapter::KEYDOWN
    && ea.getKey() == _keyEventToggleTransparency))
    return osgGA::StateSetManipulator::handle(ea, aa);

  if (!_initialized)
    _transparency = (_stateset->getMode(GL_BLEND)&osg::StateAttribute::ON);
  setTransparencyEnabled(!getTransparencyEnabled());
  aa.requestRedraw();

  return true;
}