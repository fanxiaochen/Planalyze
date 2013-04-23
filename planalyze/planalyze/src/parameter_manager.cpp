#include <iostream>
#include <QDomElement>
#include <QDomDocument>
#include <QTextStream>

#include "parameter.h"
#include "main_window.h"
#include "parameter_dialog.h"
#include "file_system_model.h"
#include "parameter_manager.h"

ParameterManager::ParameterManager(void)
  :orientation_radius_(new DoubleParameter("Orientation Radius", "Orientation Radius", 1.5, 0.5, 16, 0.5)),
  normal_radius_(new DoubleParameter("Normal Radius", "Normal Radius", 1.5, 0.5, 16, 0.5)),
  thickness_radius_(new DoubleParameter("Thickness Radius", "Thickness Radius", 3, 1, 16, 0.5)),
  virtual_scan_noise_(new DoubleParameter("Virtual Scan Noise", "Virtual Scan Noise", 0.1, 0.00, 1, 0.01)),
  virtual_scan_distance_(new DoubleParameter("Virtual Scan Distance", "Virtual Scan Distance", 200, 10, 1000, 1.0)),
  virtual_scan_resolution_(new DoubleParameter("Virtual Scan Resolution", "Virtual Scan Resolution", 1, 0.01, 100, 0.01)),
  registration_max_iterations_(new IntParameter("Max Iterations", "Max Iterations", 1024, 1, 1024)),
  registration_max_distance_(new DoubleParameter("Max Distance", "Max Distance", 16, 1, 128, 1.0)),
  rename_offset_(new IntParameter("Rename Offset", "Rename Offset", 0, -11, 11, 1)),
  rename_frame_offset_(new IntParameter("Rename Frame Offset", "Rename Frame Offset", 0, -10000, 10000, 1)),
  frame_offset_(new IntParameter("Frame Offset", "Frame Offset", 1, -10000, 10000, 1)),
  delta_(new IntParameter("Delta", "Delta", 0, -100, 100, 1)),
  frame_delta_(new IntParameter("Frame Delta", "Frame Delta", 1, 1, 100, 1)),
  stop_delta_(new IntParameter("Stop Delta", "Stop Delta", 1, 1, 100, 1)),
  camera_delta_(new IntParameter("Camera Delta", "Camera Delta", 1, 1, 100, 1)),
  camera_number_(new IntParameter("Camera Number", "Camera Number", 1, 1, 1000, 1)),
  frame_multiple_(new IntParameter("Frame Multiple", "Frame Multiple", 1, 1, 1000, 1)),
  angle_(new IntParameter("Rotate Angle", "Rotate Angle", 0, -12, 12, 1)),
  is_point_(new BoolParameter("Points or Not", "Points or Not", true)),
  generator_ctr_threshold_(new IntParameter("CTR Threshold", "CTR Threshold", 25, 1, 255, 1)),
  generator_sat_threshold_(new IntParameter("SAT Threshold", "SAT Threshold", 500, 1, 1000, 1)),
  start_frame_(new IntParameter("Start Frame", "Start Frame", -1, -1, -1, 1)),
  end_frame_(new IntParameter("End Frame", "End Frame", -1, -1, -1, 1)),
  current_frame_(new IntParameter("Current Frame", "Current Frame", -1, -1, -1, 1)),
  segment_threshold_(new IntParameter("Segment Threshold", "Segment Threshold", 8, 3, 2048, 1)),
  smooth_cost_(new IntParameter("Smooth Cost", "Smooth Cost", 10, 1, 1000, 1)),
  downsampling_(new IntParameter("Downsampling", "Downsampling", 1, 1, 100, 1)),
  leaf_component_size_threshold_(new IntParameter("Leaf Component Size", "Leaf Component Size", 128, 4, 10240)),
  stem_component_size_threshold_(new IntParameter("Stem Component Size", "Stem Component Size", 64, 4, 10240)),
  stem_skeleton_radius_(new DoubleParameter("Stem Skeleton Radius", "Stem Skeleton Radius", 4, 0.5, 16, 0.5)),
  stem_length_threshold_(new DoubleParameter("Stem Skeleton Length", "Stem Skeleton Length", 8, 0.5, 32, 0.5)),
  curvature_quantize_(new DoubleParameter("Curvature Quantize", "Threshold Quantize", 0.0015, 0.00001, 0.1, 0.00001)),
  stem_thickness_(new DoubleParameter("Stem Thickness", "Stem Thickness", 2.0, 0.1, 16.0, 0.1)),
  triangle_length_(new DoubleParameter("Triangle Length", "Triangle Length", 1.5, 1.0, 8.0, 0.1)),
  radius_(new IntParameter("Radius", "Radius", 500, 500, 1000, 1)),
  times_(new IntParameter("Times", "Times", 5, 1, 100)),
  is_using_pot_(new BoolParameter("Pot or Not", "Pot or Not", true))
{
}

ParameterManager::~ParameterManager(void)
{
  delete orientation_radius_;
  delete normal_radius_;
  delete thickness_radius_;
  delete registration_max_distance_;
  delete registration_max_iterations_;
  delete rename_offset_;
  delete rename_frame_offset_;
  delete delta_;
  delete generator_ctr_threshold_;
  delete generator_sat_threshold_;
  delete start_frame_;
  delete end_frame_;
  delete current_frame_;
  delete virtual_scan_noise_;
  delete virtual_scan_distance_;
  delete virtual_scan_resolution_;
  delete smooth_cost_;
  delete leaf_component_size_threshold_;
  delete stem_component_size_threshold_;
  delete stem_skeleton_radius_;
  delete stem_length_threshold_;
  delete curvature_quantize_;
  delete stem_thickness_;
  delete triangle_length_;
}

void ParameterManager::initFrameNumbers(void)
{
  int start_frame, end_frame;
  MainWindow::getInstance()->getFileSystemModel()->getFrameRange(start_frame, end_frame);

  start_frame_->setDefaultValue(start_frame);
  start_frame_->setValue(start_frame);
  start_frame_->setLow(start_frame);
  start_frame_->setHigh(end_frame);

  end_frame_->setDefaultValue(end_frame);
  end_frame_->setValue(end_frame);
  end_frame_->setLow(start_frame);
  end_frame_->setHigh(end_frame);

  current_frame_->setDefaultValue(start_frame);
  current_frame_->setValue(start_frame);
  current_frame_->setLow(start_frame);
  current_frame_->setHigh(end_frame);

  return;
}

double ParameterManager::getVirtualScanDistance(void) const
{
  return *virtual_scan_distance_;
}

int ParameterManager::getSegmentThreshold(void) const
{
  return *segment_threshold_;
}

double ParameterManager::getTriangleLength(void) const
{
  return *triangle_length_;
}

double ParameterManager::getNormalRadius(void) const
{
  return *normal_radius_;
}

double ParameterManager::getThicknessRadius(void) const
{
  return *thickness_radius_;
}

double ParameterManager::getOrientationRadius(void) const
{
  return *orientation_radius_;
}

double ParameterManager::getRegistrationMaxDistance(void) const
{
  return *registration_max_distance_;
}

int ParameterManager::getSmoothCost(void) const
{
  return *smooth_cost_;
}

double ParameterManager::getCurvatureQuantize(void) const
{
  return *curvature_quantize_;
}

double ParameterManager::getStemThickness(void) const
{
  return *stem_thickness_;
}

void ParameterManager::addFrameParameters(ParameterDialog* parameter_dialog, bool with_frames)
{
  if (!with_frames)
    return;

  parameter_dialog->addParameter(start_frame_);
  parameter_dialog->addParameter(end_frame_);

  return;
}

bool ParameterManager::getVirtualScanParameters(double& virtual_scan_noise, double& virtual_scan_distance, double& virtual_scan_resolution,
                                                int& start_frame, int& end_frame)
{
  ParameterDialog parameter_dialog("Virtual Scan Parameters", MainWindow::getInstance());
  parameter_dialog.addParameter(virtual_scan_noise_);
  parameter_dialog.addParameter(virtual_scan_distance_);
  parameter_dialog.addParameter(virtual_scan_resolution_);
  addFrameParameters(&parameter_dialog, true);
  if (!parameter_dialog.exec() == QDialog::Accepted)
    return false;

  virtual_scan_noise = *virtual_scan_noise_;
  virtual_scan_distance = *virtual_scan_distance_;
  virtual_scan_resolution = *virtual_scan_resolution_;
  getFrameparametersImpl(start_frame, end_frame, true);

  return true;
}

bool ParameterManager::getFrameParameter(int& frame)
{
  ParameterDialog parameter_dialog("Frame Parameter", MainWindow::getInstance());
  parameter_dialog.addParameter(current_frame_);
  if (!parameter_dialog.exec() == QDialog::Accepted)
    return false;

  frame = *current_frame_;
  return true;
}

double ParameterManager::getStemSkeletonRadius(void) const
{
  return *stem_skeleton_radius_;
}

bool ParameterManager::getStemSkeletonRadius(double& stem_skeleton_radius)
{
  ParameterDialog parameter_dialog("Stem Skeleton Radius Parameter", MainWindow::getInstance());
  parameter_dialog.addParameter(stem_skeleton_radius_);
  if (!parameter_dialog.exec() == QDialog::Accepted)
    return false;

  stem_skeleton_radius = *stem_skeleton_radius_;
  return true;
}

double ParameterManager::getStemSkeletonLength(void) const
{
  return *stem_length_threshold_;
}

bool ParameterManager::getStemSkeletonLength(double& stem_skeleton_length)
{
  ParameterDialog parameter_dialog("Stem Skeleton Length Parameter", MainWindow::getInstance());
  parameter_dialog.addParameter(stem_length_threshold_);
  if (!parameter_dialog.exec() == QDialog::Accepted)
    return false;

  stem_skeleton_length = *stem_length_threshold_;
  return true;
}

bool ParameterManager::getTrackAndEvolveParameters(double& smooth_cost)
{
  int place_holder_1, place_holder_2;
  return getTrackAndEvolveParameters(smooth_cost, place_holder_1, place_holder_2, false);
}

bool ParameterManager::getTrackAndEvolveParameters(double& smooth_cost, int& start_frame, int& end_frame, bool with_frames)
{
  ParameterDialog parameter_dialog("Track and Evolve Parameters", MainWindow::getInstance());
  parameter_dialog.addParameter(smooth_cost_);
  addFrameParameters(&parameter_dialog, with_frames);
  if (!parameter_dialog.exec() == QDialog::Accepted)
    return false;

  smooth_cost = *smooth_cost_;
  getFrameparametersImpl(start_frame, end_frame, with_frames);

  return true;
}

bool ParameterManager::getFrameParameters(int& start_frame, int& end_frame, int& downsampling)
{
  ParameterDialog parameter_dialog("Frame Parameters", MainWindow::getInstance());
  parameter_dialog.addParameter(downsampling_);
  addFrameParameters(&parameter_dialog, true);
  if (!parameter_dialog.exec() == QDialog::Accepted)
    return false;

  downsampling = *downsampling_;
  getFrameparametersImpl(start_frame, end_frame, true);

  return true;
}

void ParameterManager::getFrameparametersImpl(int& start_frame, int& end_frame, bool with_frames)
{
  if (!with_frames)
    return;

  if ((int)(*start_frame_) > (int)(*end_frame_))
  {
    int temp = *start_frame_;
    start_frame_->setValue(*end_frame_);
    end_frame_->setValue(temp);
  }

  start_frame = *start_frame_;
  end_frame = *end_frame_;

  return;
}

bool ParameterManager::getGenerationParameters(int& ctr_threshold, int& sat_threshold)
{
  int place_holder_1, place_holder_2;
  return getGenerationParameters(ctr_threshold, sat_threshold, place_holder_1, place_holder_2, false);
}
bool ParameterManager::getGenerationParameters(int& ctr_threshold, int& sat_threshold,
                                               int& start_frame, int& end_frame, bool with_frames)
{
  ParameterDialog parameter_dialog("Points Generation Parameters", MainWindow::getInstance());
  parameter_dialog.addParameter(generator_ctr_threshold_);
  parameter_dialog.addParameter(generator_sat_threshold_);
  addFrameParameters(&parameter_dialog, with_frames);
  if (!parameter_dialog.exec() == QDialog::Accepted)
    return false;

  ctr_threshold = *generator_ctr_threshold_;
  sat_threshold = *generator_sat_threshold_;
  getFrameparametersImpl(start_frame, end_frame, with_frames);

  return true;
}

bool ParameterManager::getRegistrationLUMParameters(int& segment_threshold, int& max_iterations, double& max_distance)
{
  int place_holder_1, place_holder_2;
  return getRegistrationLUMParameters(segment_threshold, max_iterations, max_distance, place_holder_1, place_holder_2, false);

  return true;
}

bool ParameterManager::getRegistrationLUMParameters(int& segment_threshold, int& max_iterations, double& max_distance,
                                                 int& start_frame, int& end_frame, bool with_frames)
{
  ParameterDialog parameter_dialog("Registration Parameters", MainWindow::getInstance());
  parameter_dialog.addParameter(segment_threshold_);
  parameter_dialog.addParameter(registration_max_iterations_);
  parameter_dialog.addParameter(registration_max_distance_);
  addFrameParameters(&parameter_dialog, with_frames);
  if (!parameter_dialog.exec() == QDialog::Accepted)
    return false;

  segment_threshold = *segment_threshold_;
  max_iterations = *registration_max_iterations_;
  max_distance = *registration_max_distance_;
  getFrameparametersImpl(start_frame, end_frame, with_frames);

  return true;
}

bool ParameterManager::getRegistrationLUMParameters(int& segment_threshold, int& max_iterations, double& max_distance, int& frame)
{
  ParameterDialog parameter_dialog("Registration Parameters", MainWindow::getInstance());
  parameter_dialog.addParameter(segment_threshold_);
  parameter_dialog.addParameter(registration_max_iterations_);
  parameter_dialog.addParameter(registration_max_distance_);
  parameter_dialog.addParameter(current_frame_);
  if (!parameter_dialog.exec() == QDialog::Accepted)
    return false;

  segment_threshold = *segment_threshold_;
  max_iterations = *registration_max_iterations_;
  max_distance = *registration_max_distance_;
  frame = *current_frame_;

  return true;
}

bool ParameterManager::getRegistrationICPParameters(int& max_iterations, double& max_distance, int& frame, bool& is_using_pot, int& times)
{
  ParameterDialog parameter_dialog("Registration Parameters", MainWindow::getInstance());
  parameter_dialog.addParameter(registration_max_iterations_);
  parameter_dialog.addParameter(registration_max_distance_);
  parameter_dialog.addParameter(current_frame_);
  parameter_dialog.addParameter(times_);
  parameter_dialog.addParameter(is_using_pot_);
  if (!parameter_dialog.exec() == QDialog::Accepted)
    return false;

  max_iterations = *registration_max_iterations_;
  max_distance = *registration_max_distance_;
  frame = *current_frame_;
  times = *times_;
  is_using_pot = *is_using_pot_;

  return true;
}

bool ParameterManager::getEstimateNormalParameters(double& normal_radius, int& start_frame, int &end_frame)
{
  ParameterDialog parameter_dialog("Estimate Normal Parameters", MainWindow::getInstance());
  parameter_dialog.addParameter(normal_radius_);
  addFrameParameters(&parameter_dialog, true);
  if (!parameter_dialog.exec() == QDialog::Accepted)
    return false;

  normal_radius = *normal_radius_;
  getFrameparametersImpl(start_frame, end_frame, true);

  return true;
}

bool ParameterManager::getEstimateOrientationParameters(double& orientation_radius, int& start_frame, int &end_frame)
{
  ParameterDialog parameter_dialog("Estimate Orientation Parameters", MainWindow::getInstance());
  parameter_dialog.addParameter(orientation_radius_);
  addFrameParameters(&parameter_dialog, true);
  if (!parameter_dialog.exec() == QDialog::Accepted)
    return false;

  orientation_radius = *orientation_radius_;
  getFrameparametersImpl(start_frame, end_frame, true);

  return true;
}

bool ParameterManager::getEstimateThicknessParameters(double& thickness_radius, int& start_frame, int &end_frame)
{
  ParameterDialog parameter_dialog("Estimate Normal Parameters", MainWindow::getInstance());
  parameter_dialog.addParameter(thickness_radius_);
  addFrameParameters(&parameter_dialog, true);
  if (!parameter_dialog.exec() == QDialog::Accepted)
    return false;

  thickness_radius = *thickness_radius_;
  getFrameparametersImpl(start_frame, end_frame, true);

  return true;
}

bool ParameterManager::getRenameViewsParameters(int& rename_offset)
{
  int place_holder_1, place_holder_2;
  return getRenameViewsParameters(rename_offset, place_holder_1, place_holder_2, false);
}

bool ParameterManager::getRenameViewsParameters(int& rename_offset, int& start_frame, int& end_frame, bool with_frames)
{
  ParameterDialog parameter_dialog("Rename View Parameters", MainWindow::getInstance());
  parameter_dialog.addParameter(rename_offset_);
  addFrameParameters(&parameter_dialog, with_frames);
  if (!parameter_dialog.exec() == QDialog::Accepted)
    return false;

  rename_offset = *rename_offset_;
  getFrameparametersImpl(start_frame, end_frame, with_frames);

  return true;
}

bool ParameterManager::getRenameFramesParameters(int& rename_offset, int& start_frame, int& end_frame, bool with_frames)
{
   ParameterDialog parameter_dialog("Rename Frames Parameters", MainWindow::getInstance());
   parameter_dialog.addParameter(rename_frame_offset_);
   addFrameParameters(&parameter_dialog, with_frames);
   if(!parameter_dialog.exec() == QDialog::Accepted)
      return false;
   
   rename_offset = *rename_frame_offset_;
   getFrameparametersImpl(start_frame, end_frame, with_frames);

   return true;
}

bool ParameterManager::getExtractKeyFramesParameters(int& frame_offset, int& start_frame, int& end_frame, bool& is_point, bool with_frames)
{
  ParameterDialog parameter_dialog("Extract Key Frames Parameters", MainWindow::getInstance());
  parameter_dialog.addParameter(frame_offset_);
  parameter_dialog.addParameter(is_point_);
  addFrameParameters(&parameter_dialog, with_frames);
  if(!parameter_dialog.exec() == QDialog::Accepted)
    return false;

  frame_offset = *frame_offset_;
  is_point = *is_point_;
  getFrameparametersImpl(start_frame, end_frame, with_frames);

  return true;
}

bool ParameterManager::getRotateCloudParameters(int& angle, int& start_frame, int& end_frame, bool with_frames)
{
  ParameterDialog parameter_dialog("Rotate Point Cloud", MainWindow::getInstance());
  parameter_dialog.addParameter(angle_);
  addFrameParameters(&parameter_dialog, with_frames);
  if(!parameter_dialog.exec() == QDialog::Accepted)
    return false;

  angle = *angle_;
  getFrameparametersImpl(start_frame, end_frame, with_frames);

  return true;
}

bool ParameterManager::getConvertPcdParameters(int& start_frame, int& end_frame, bool with_frames)
{
  ParameterDialog parameter_dialog("Convert Pcd Parameters", MainWindow::getInstance());
  addFrameParameters(&parameter_dialog, with_frames);
  if(!parameter_dialog.exec() == QDialog::Accepted)
    return false;

  getFrameparametersImpl(start_frame, end_frame, with_frames);

  return true;
}

bool ParameterManager::getRemoveErrorPointsParameters(int& start_frame, int& end_frame, int& radius, bool with_frames)
{
  ParameterDialog parameter_dialog("Remove Error Points Parameters", MainWindow::getInstance());
  parameter_dialog.addParameter(radius_);
  addFrameParameters(&parameter_dialog, with_frames);
  if(!parameter_dialog.exec() == QDialog::Accepted)
    return false;

  radius = *radius_;
  getFrameparametersImpl(start_frame, end_frame, with_frames);

  return true;
}

bool ParameterManager::getReverseFramesParameters(int& start_frame, int& end_frame, bool with_frames)
{
  ParameterDialog parameter_dialog("Reverse Frames Parameters", MainWindow::getInstance());
  addFrameParameters(&parameter_dialog, with_frames);
  if(!parameter_dialog.exec() == QDialog::Accepted)
    return false;

  getFrameparametersImpl(start_frame, end_frame, with_frames);

  return true;
}

bool ParameterManager::getExtractPlantParameters(int& start_frame, int& end_frame , int& delta, bool with_frames)
{
  ParameterDialog parameter_dialog("Get Extract Plant Parameters", MainWindow::getInstance());
  parameter_dialog.addParameter(delta_);
  addFrameParameters(&parameter_dialog, with_frames);
  if(!parameter_dialog.exec() == QDialog::Accepted)
    return false;

  delta = *delta_;
  getFrameparametersImpl(start_frame, end_frame, with_frames);

  return true;
}

bool ParameterManager::getPovrayDataParameters(int& start_frame, int& end_frame , 
                                               int& frame_delta, int& frame_multiple, int& camera_number, 
                                               int& stop_delta, int& camera_delta, bool with_frames)
{
  ParameterDialog parameter_dialog("Generate Povray Data Parameters", MainWindow::getInstance());
  parameter_dialog.addParameter(frame_delta_);
  parameter_dialog.addParameter(frame_multiple_);
  parameter_dialog.addParameter(camera_number_);
  parameter_dialog.addParameter(stop_delta_);
  parameter_dialog.addParameter(camera_delta_);
  addFrameParameters(&parameter_dialog, with_frames);
  if(!parameter_dialog.exec() == QDialog::Accepted)
    return false;

  frame_delta = *frame_delta_;
  frame_multiple = *frame_multiple_;
  camera_number = *camera_number_;
  stop_delta = *stop_delta_;
  camera_delta = *camera_delta_;
  getFrameparametersImpl(start_frame, end_frame, with_frames);

  return true;
}

bool ParameterManager::getBadFramesParameters(int& start_frame, int& end_frame ,bool with_frames)
{
  ParameterDialog parameter_dialog("Remove Bad Frames Parameters", MainWindow::getInstance());
  addFrameParameters(&parameter_dialog, with_frames);
  if(!parameter_dialog.exec() == QDialog::Accepted)
    return false;

  getFrameparametersImpl(start_frame, end_frame, with_frames);

  return true;
}


bool ParameterManager::getAllParameters(void)
{
  ParameterDialog parameter_dialog("All Parameters", MainWindow::getInstance());
  parameter_dialog.addParameter(orientation_radius_);
  parameter_dialog.addParameter(smooth_cost_);
  parameter_dialog.addParameter(stem_component_size_threshold_);
  parameter_dialog.addParameter(leaf_component_size_threshold_);
  parameter_dialog.addParameter(stem_skeleton_radius_);
  parameter_dialog.addParameter(stem_length_threshold_);
  parameter_dialog.addParameter(curvature_quantize_);
  parameter_dialog.addParameter(triangle_length_);
  parameter_dialog.addParameter(stem_thickness_);
  if (!parameter_dialog.exec() == QDialog::Accepted)
    return false;

  return true;
}

int ParameterManager::getStemComponentSize(void) const
{
  return *stem_component_size_threshold_;
}

int ParameterManager::getLeafComponentSize(void) const
{
  return *leaf_component_size_threshold_;
}


void ParameterManager::loadParameters(const QString& filename)
{
  QFile file(filename);
  if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
    return;
  }

  QDomDocument doc("parameters");
  if (!doc.setContent(&file)) {
    file.close();
    return;
  }

  QDomElement root = doc.documentElement();

  double double_value = 0.0;
  int int_value = 0;

  QDomElement leaf_component_size = root.firstChildElement("leaf_component_size");
  int_value = leaf_component_size.attribute("value", "").toInt();
  if (int_value != 0.0) leaf_component_size_threshold_->setValue(int_value);

  QDomElement stem_component_size = root.firstChildElement("stem_component_size");
  int_value = stem_component_size.attribute("value", "").toInt();
  if (int_value != 0.0) stem_component_size_threshold_->setValue(int_value);

  QDomElement stem_skeleton_radius = root.firstChildElement("stem_skeleton_radius");
  double_value = stem_skeleton_radius.attribute("value", "").toDouble();
  if (double_value != 0.0) stem_skeleton_radius_->setValue(double_value);

  QDomElement stem_skeleton_length = root.firstChildElement("stem_skeleton_length");
  double_value = stem_skeleton_length.attribute("value", "").toDouble();
  if (double_value != 0.0) stem_length_threshold_->setValue(double_value);

  QDomElement curvature_quantize = root.firstChildElement("curvature_quantize");
  double_value = curvature_quantize.attribute("value", "").toDouble();
  if (double_value != 0.0) curvature_quantize_->setValue(double_value);

  QDomElement stem_thickness = root.firstChildElement("stem_thickness");
  double_value = stem_thickness.attribute("value", "").toDouble();
  if (double_value != 0.0) stem_thickness_->setValue(double_value);

  QDomElement triangle_length = root.firstChildElement("triangle_length");
  double_value = triangle_length.attribute("value", "").toDouble();
  if (double_value != 0.0) triangle_length_->setValue(double_value);

  return;
}

void ParameterManager::saveParameters(const QString& filename)
{
  QFile file(filename);
  if (!file.open(QIODevice::WriteOnly)) {
    return;
  }

  QDomDocument doc("parameters");
  QDomProcessingInstruction xml_declaration = doc.createProcessingInstruction("xml", "version=\"1.0\"");
  doc.appendChild(xml_declaration);

  QDomElement root = doc.createElement(QString("parameters"));
  doc.appendChild(root);

  QDomElement leaf_component_size = doc.createElement(QString("leaf_component_size"));
  leaf_component_size.setAttribute("value", (int)(*leaf_component_size_threshold_));
  root.appendChild(leaf_component_size);

  QDomElement stem_component_size = doc.createElement(QString("stem_component_size"));
  stem_component_size.setAttribute("value", (int)(*stem_component_size_threshold_));
  root.appendChild(stem_component_size);

  QDomElement stem_skeleton_radius = doc.createElement(QString("stem_skeleton_radius"));
  stem_skeleton_radius.setAttribute("value", (double)(*stem_skeleton_radius_));
  root.appendChild(stem_skeleton_radius);

  QDomElement stem_skeleton_length = doc.createElement(QString("stem_skeleton_length"));
  stem_skeleton_length.setAttribute("value", (double)(*stem_length_threshold_));
  root.appendChild(stem_skeleton_length);

  QDomElement curvature_quantize = doc.createElement(QString("curvature_quantize"));
  curvature_quantize.setAttribute("value", (double)(*curvature_quantize_));
  root.appendChild(curvature_quantize);

  QDomElement stem_thickness = doc.createElement(QString("stem_thickness"));
  stem_thickness.setAttribute("value", (double)(*stem_thickness_));
  root.appendChild(stem_thickness);

  QDomElement triangle_length = doc.createElement(QString("triangle_length"));
  triangle_length.setAttribute("value", (double)(*triangle_length_));
  root.appendChild(triangle_length);

  QTextStream text_stream(&file);
  text_stream << doc.toString();
  file.close();

  return;
}