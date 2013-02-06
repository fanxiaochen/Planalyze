#pragma once
#ifndef AXIS_INDICATOR_H
#define AXIS_INDICATOR_H

#include "renderable.h"

class AxisIndicator : public Renderable
{
public:
  AxisIndicator(void);
  virtual ~AxisIndicator(void);

  virtual const char* className() const {return "AxisIndicator";}

protected:
  virtual void updateImpl();
};

#endif // AXIS_INDICATOR_H