#pragma once
#ifndef INFORMATION_H
#define INFORMATION_H

#include "Renderable.h"

class Information : public Renderable
{
public:
  Information(void);
  ~Information(void);

  virtual const char* className() const {return "Information";}

  void setText(const std::string& text, float x, float y);
  void resize(double width, double height);

protected:
  virtual void updateImpl();

private:
  float                       width_, height_;
  std::string                 text_;
  float                       x_,y_;
};

#endif // INFORMATION_H