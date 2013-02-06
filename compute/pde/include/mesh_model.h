#pragma once
#ifndef MESH_MODEL_H
#define MESH_MODEL_H

#include "Renderable.h"

class MeshModel : public Renderable
{
public:
  MeshModel(void);
  virtual ~MeshModel(void);

  virtual const char* className() const {return "MeshModel";}

  bool open(const std::string& filename);
  void virtualScan(double noise, double distance, double resolution);
protected:
  virtual void updateImpl();

protected:
  std::string                     filename_;

private:
  osg::ref_ptr<osg::Node>         model_node_;
};

#endif // MESH_MODEL_H