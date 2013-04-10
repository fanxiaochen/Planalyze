#pragma once
#ifndef PARAMETER_MANAGER_H_
#define PARAMETER_MANAGER_H_

#include <QString>

class IntParameter;
class DoubleParameter;
class BoolParameter;
class ParameterDialog;

class ParameterManager
{
public:
  static ParameterManager& getInstance() {
    static ParameterManager theSingleton;
    return theSingleton;
  }

  void initFrameNumbers(void);
  void loadParameters(const QString& filename);
  void saveParameters(const QString& filename);

  int getSegmentThreshold(void) const;
  double getTriangleLength(void) const;
  double getNormalRadius(void) const;
  double getThicknessRadius(void) const;
  double getOrientationRadius(void) const;
  double getRegistrationMaxDistance(void) const;
  int getColorizeMod(void) const;
  int getSmoothCost(void) const;
  int getStemComponentSize(void) const;
  int getLeafComponentSize(void) const;

  double getCurvatureQuantize(void) const;
  double getStemThickness(void) const;

  bool getFrameParameter(int& frame);
  bool getFrameParameters(int& start_frame, int& end_frame, int& downsampling);

  bool getTrackAndEvolveParameters(double& smooth_cost);
  bool getTrackAndEvolveParameters(double& smooth_cost, int& start_frame, int& end_frame, bool with_frames=true);

  double getStemSkeletonRadius(void) const;
  bool getStemSkeletonRadius(double& stem_skeleton_radius);

  double getStemSkeletonLength(void) const;
  bool getStemSkeletonLength(double& stem_skeleton_length);

  double getVirtualScanDistance(void) const;
  bool getVirtualScanParameters(double& virtual_scan_noise, double& virtual_scan_distance, double& virtual_scan_resolution,
    int& start_frame, int& end_frame);

  bool getEstimateNormalParameters(double& normal_radius, int& start_frame, int &end_frame);
  bool getEstimateOrientationParameters(double& thickness_radius, int& start_frame, int &end_frame);
  bool getEstimateThicknessParameters(double& thickness_radius, int& start_frame, int &end_frame);

  bool getGenerationParameters(int& ctr_threshold, int& sat_threshold);
  bool getGenerationParameters(int& ctr_threshold, int& sat_threshold,
    int& start_frame, int& end_frame, bool with_frames=true);

  bool getRegistrationLUMParameters(int& segment_threshold, int& max_iterations, double& max_distance);
  bool getRegistrationLUMParameters(int& segment_threshold, int& max_iterations, double& max_distance,
    int& start_frame, int& end_frame, bool with_frames=true);
  bool getRegistrationLUMParameters(int& segment_threshold, int& max_iterations, double& max_distance, int& frame);
  bool getRegistrationICPParameters(int& max_iterations, double& max_distance, int& frame, int& times);

  bool getRenameViewsParameters(int& rename_offset);
  bool getRenameViewsParameters(int& rename_offset,
    int& start_frame, int& end_frame, bool with_frames=true);

  bool getRenameFramesParameters(int& rename_offset,
    int& start_frame, int& end_frame, bool with_frames=true);

  bool getExtractKeyFramesParameters(int& frame_offset,
    int& start_frame, int& end_frame, bool& is_point, bool with_frames=true);

  bool getPovrayDataParameters(int& start_frame, int& end_frame, int& frame_delta, int& frame_multiple,
    int& camera_number, int& stop_delta, int& camera_delta, bool with_frames = true);

  bool getBadFramesParameters(int& start_frame, int& end_frame, bool with_frames = true);

  bool getExtractPlantParameters(int& start_frame, int& end_frame, int& delta, bool with_frames = true);

  bool getRotateCloudParameters(int& angle, int& start_frame, int& end_frame, bool with_frames = true);

  bool getConvertPcdParameters(int& start_frame, int& end_frame, bool with_frames = true);

  bool getRemoveErrorPointsParameters(int& start_frame, int& end_frame, int& radius, bool with_frames = true);

  bool getAllParameters(void);

protected:
  void addFrameParameters(ParameterDialog* parameter_dialog, bool with_frames);
  void getFrameparametersImpl(int& start_frame, int& end_frame, bool with_frames);
private:
  ParameterManager(void);
  ParameterManager(const ParameterManager &) {}            // copy ctor hidden
  ParameterManager& operator=(const ParameterManager &) {return (*this);}   // assign op. hidden
  virtual ~ParameterManager();

  DoubleParameter*                                    orientation_radius_;
  DoubleParameter*                                    stem_skeleton_radius_;
  DoubleParameter*                                    normal_radius_;
  DoubleParameter*                                    thickness_radius_;

  IntParameter*                                       segment_threshold_;
  IntParameter*                                       registration_max_iterations_;
  DoubleParameter*                                    registration_max_distance_;
  IntParameter*                                       times_;

  IntParameter*                                       rename_offset_;
  IntParameter*                                       rename_frame_offset_;
  IntParameter*                                       frame_offset_;

  BoolParameter*                                      is_point_;

  IntParameter*                                       delta_;
  IntParameter*                                       downsampling_;
  IntParameter*                                       frame_delta_;
  IntParameter*                                       camera_number_;
  IntParameter*                                       stop_delta_;
  IntParameter*                                       camera_delta_;
  IntParameter*                                       frame_multiple_;

  IntParameter*                                       radius_;

  IntParameter*                                       angle_;

  IntParameter*                                       generator_ctr_threshold_;
  IntParameter*                                       generator_sat_threshold_;

  IntParameter*                                       start_frame_;
  IntParameter*                                       end_frame_;
  IntParameter*                                       current_frame_;

  IntParameter*                                       colorize_mod_;

  IntParameter*                                       smooth_cost_;

  DoubleParameter*                                    virtual_scan_noise_;
  DoubleParameter*                                    virtual_scan_distance_;
  DoubleParameter*                                    virtual_scan_resolution_;

  IntParameter*                                       leaf_component_size_threshold_;
  IntParameter*                                       stem_component_size_threshold_;
  DoubleParameter*                                    stem_length_threshold_;

  DoubleParameter*                                    curvature_quantize_;
  DoubleParameter*                                    stem_thickness_;

  DoubleParameter*                                    triangle_length_;
};

#endif // PARAMETER_MANAGER_H_
