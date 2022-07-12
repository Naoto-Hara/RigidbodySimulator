#ifndef settings_h
#define settings_h

#include <Eigen/Core>
#include <iostream>

struct SimulatorSettings
{
  double dt;
  double dx;
  double dx_sample_points;
  double ugrd_dx;
  double max_time;

  double kN;
  double gammaN;
  double kT;
  double gammaT;
  double mu;

  Eigen::Vector2d g;
  
  std::string resume_folder;
  std::string templates_file_name_to_resume;
  std::string objects_file_name_to_resume;
  
  std::string serialization_folder;
  std::string templates_file_name_for_serialization;
  std::string objects_file_name_for_serialization;

  bool auto_save;
  double auto_save_interval;
  std::string intermediate_objects_file_template_for_serialization;
};

#endif
