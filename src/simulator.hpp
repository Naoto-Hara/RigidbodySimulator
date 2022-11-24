#ifndef simulator_hpp
#define simulator_hpp
#include <vector>
#include "signeddistance2d.h"
#include "uniformgrid2d.h"
#include "collisiondetection2d.h"
#include "settings.h"

class RigidBody2D;

struct Simulator
{
  Simulator(): animation( false ), ugrd( nullptr ), frame_idx( 0 ), auto_save_idx( 0 ), prev_auto_save_time( -1000.0 ) {}
  
  void clearData()
  {
    for( int i=0; i<shape_templates.size(); i++ )
    {
      delete shape_templates[i];
      shape_templates[i] = nullptr;
    }
    shape_templates.clear();
    
    for( int i=0; i<rigid_bodies.size(); i++ )
    {
      delete rigid_bodies[i];
      rigid_bodies[i] = nullptr;
    }
    rigid_bodies.clear();
  }
  
  int window_width;
  int window_height;
  double frame_window_size_scale_x;
  double frame_window_size_scale_y;
  
  bool animation;
  
  int numCircleTemplates;
  int numPolygonTemplates;
  std::vector< ObjectTemplate2D* > shape_templates;
  std::vector< RigidBody2D* > rigid_bodies;
  UniformGrid2D* ugrd;
  BroadPhaseActiveSet broad_phase_active_set;
  int frame_idx;
  int auto_save_idx;
  double prev_auto_save_time;
  
  SimulatorSettings settings;
};

void stepSystem( Simulator& io_Simulator, double in_dt );

#endif
