#pragma once
#ifndef STATESET_MANIPULATOR_H
#define STATESET_MANIPULATOR_H

#include <osgGA/StateSetManipulator>

class StateSetManipulator : public osgGA::StateSetManipulator
{
public:

  StateSetManipulator(osg::StateSet* stateset=0);
  virtual ~StateSetManipulator(void);

  virtual bool handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& us);

  void setTransparencyEnabled(bool transparency);
  bool getTransparencyEnabled() const {return _transparency;};
  void setKeyEventToggleTransparency(int key) { _keyEventToggleTransparency = key; }
  int getKeyEventToggleTransparency() const { return _keyEventToggleTransparency; }
protected:
  bool  _transparency;
  int   _keyEventToggleTransparency;
};


#endif // STATESET_MANIPULATOR_H