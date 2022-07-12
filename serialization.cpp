#define EIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT
#define EIGEN_DONT_VECTORIZE

#include "serialization.h"
#include "simulator.hpp"
#include "polygonobj.h"
#include "circleobj.h"

#pragma warning(disable: 4996)

void serializeSim( const std::string& in_Templates_filename, const std::string& in_Objects_filename, ShapeTemplates& in_Templates, ObjectList& in_Objects )
{
  FILE* file;
  if( file = fopen( in_Templates_filename.c_str(), "r" ) )
  {
    fclose( file );
    remove( in_Templates_filename.c_str() );
  }
  
  in_Templates.serialize( in_Templates_filename );
  std::cout << "Serialized " << in_Templates.numTemplates() << " templates" << std::endl;
  
  if( file = fopen( in_Objects_filename.c_str(), "r" ) )
  {
    fclose( file );
    remove( in_Objects_filename.c_str() );
  }
  
  in_Objects.serialize( in_Objects_filename );
  std::cout << "Serialized " << in_Objects.numObjects() << " objects" << std::endl;
}

void serializeSim( const std::string& in_Templates_filename, const std::string& in_Objects_filename, const Simulator& in_Simulator )
{
  ShapeTemplates shape_templates;
  for( int i=0; i<in_Simulator.shape_templates.size(); i++ )
  {
    if( in_Simulator.shape_templates[i]->type() == CIRCLE )
    {
      shape_templates.setTemplate( reinterpret_cast< CircleObjTemplate* >( in_Simulator.shape_templates[i] )->getRadius0() );
    }
    else
    {
      shape_templates.setTemplate( reinterpret_cast< PolygonObjTemplate* >( in_Simulator.shape_templates[i] )->getVertices0() );
    }
  }
  
  ObjectList object_list;
  
  for( int i=0; i<in_Simulator.rigid_bodies.size(); i++ )
  {
    ObjectInitInfo object;
    object.scale = in_Simulator.rigid_bodies[i]->scale();
    object.isStatic = in_Simulator.rigid_bodies[i]->isStatic();
    object.template_index = ( in_Simulator.rigid_bodies[i]->getType() == CIRCLE ) ? in_Simulator.rigid_bodies[i]->templateIndex() : in_Simulator.rigid_bodies[i]->templateIndex() - in_Simulator.numCircleTemplates;
    object.x = in_Simulator.rigid_bodies[i]->centerOfMass();
    object.v = in_Simulator.rigid_bodies[i]->velocity();
    object.theta = in_Simulator.rigid_bodies[i]->rotationAngle();
    object.omega = in_Simulator.rigid_bodies[i]->AngularVelocity();
    object.density = in_Simulator.rigid_bodies[i]->density();
    object.type = in_Simulator.rigid_bodies[i]->getType();
        
    object_list.pushBackObject( object );
  }
  
  object_list.time = in_Simulator.frame_idx * in_Simulator.settings.dt;
  object_list.frame_idx = in_Simulator.frame_idx;
  object_list.auto_save_idx = in_Simulator.auto_save_idx;
  object_list.prev_auto_save_time = in_Simulator.prev_auto_save_time;
  
  serializeSim( in_Templates_filename, in_Objects_filename, shape_templates, object_list );
}

void serializeObj( const std::string& in_Objects_filename, ObjectList& in_Objects )
{
  FILE* file;
  if( file = fopen( in_Objects_filename.c_str(), "r" ) )
  {
    fclose( file );
    remove( in_Objects_filename.c_str() );
  }
  
  in_Objects.serialize( in_Objects_filename );
  std::cout << "Serialized " << in_Objects.numObjects() << " objects" << std::endl;
}

void serializeObj( const std::string& in_Objects_filename, const Simulator& in_Simulator )
{
  ObjectList object_list;
  
  for( int i=0; i<in_Simulator.rigid_bodies.size(); i++ )
  {
    ObjectInitInfo object;
    object.scale = in_Simulator.rigid_bodies[i]->scale();
    object.isStatic = in_Simulator.rigid_bodies[i]->isStatic();
    object.template_index = ( in_Simulator.rigid_bodies[i]->getType() == CIRCLE ) ? in_Simulator.rigid_bodies[i]->templateIndex() : in_Simulator.rigid_bodies[i]->templateIndex() - in_Simulator.numCircleTemplates;
    object.x = in_Simulator.rigid_bodies[i]->centerOfMass();
    object.v = in_Simulator.rigid_bodies[i]->velocity();
    object.theta = in_Simulator.rigid_bodies[i]->rotationAngle();
    object.omega = in_Simulator.rigid_bodies[i]->AngularVelocity();
    object.density = in_Simulator.rigid_bodies[i]->density();
    object.type = in_Simulator.rigid_bodies[i]->getType();
        
    object_list.pushBackObject( object );
  }
  
  object_list.time = in_Simulator.frame_idx * in_Simulator.settings.dt;
  object_list.frame_idx = in_Simulator.frame_idx;
  object_list.auto_save_idx = in_Simulator.auto_save_idx;
  object_list.prev_auto_save_time = in_Simulator.prev_auto_save_time;
  
  serializeObj( in_Objects_filename, object_list );
}

void deserializeSim( const std::string& in_Templates_filename, const std::string& in_Objects_filename, ShapeTemplates& out_Templates, ObjectList& out_Objects )
{
  out_Templates.clearData();
  
  std::ifstream in_templates( in_Templates_filename, std::ios::binary );
  cereal::BinaryInputArchive archive_i_templates( in_templates );
  archive_i_templates( out_Templates );
  
  out_Objects.clearData();
  
  std::ifstream in_objects( in_Objects_filename, std::ios::binary );
  cereal::BinaryInputArchive archive_i_objects( in_objects );
  archive_i_objects( out_Objects );
}

void initializeSimulator( const ShapeTemplates& in_Templates, const ObjectList& in_ObjectList, Simulator& io_Simulator )
{
  io_Simulator.clearData();
  io_Simulator.rigid_bodies.clear();
  
  const int numCircleTemplates = in_Templates.circle_templates.size();
  const int numPolygonTemplates = in_Templates.polygon_templates.size();
  
  std::cout << "circle templates: " << numCircleTemplates << std::endl;
  std::cout << "polygon templates: " << numPolygonTemplates << std::endl;
  
  for( int i=0; i<numCircleTemplates; i++ )
  {
    ObjectTemplate2D* obj_template = new CircleObjTemplate( in_Templates.circle_templates[i], io_Simulator.settings.dx );
    io_Simulator.shape_templates.push_back( obj_template );
  }
  io_Simulator.numCircleTemplates = numCircleTemplates;
  
  std::cout << "set up circle templates" << std::endl;
  
  for( int i=0; i<numPolygonTemplates; i++ )
  {
    ObjectTemplate2D* obj_template = new PolygonObjTemplate( in_Templates.polygon_templates[i], io_Simulator.settings.dx );
    io_Simulator.shape_templates.push_back( obj_template );
  }
  io_Simulator.numPolygonTemplates = numPolygonTemplates;
  
  std::cout << "set up polygon templates" << std::endl;
  
  const int numObjects = in_ObjectList.numObjects();
  
  std::cout << "objects: " << numObjects << std::endl;
  
  for( int i=0; i<numObjects; i++ )
  {
    const int template_idx_offset = ( in_ObjectList.type_list[i] == CIRCLE ) ? 0 : numCircleTemplates;
    const int template_idx = template_idx_offset + in_ObjectList.template_index_list[i];
  
    io_Simulator.rigid_bodies.push_back( new RigidBody2D( io_Simulator.shape_templates[ template_idx ], template_idx, i, in_ObjectList.isStatic_list[i], in_ObjectList.density_list[i], in_ObjectList.scale_list[i], in_ObjectList.x_list[i], in_ObjectList.theta_list[i], in_ObjectList.v_list[i], in_ObjectList.omega_list[i], io_Simulator.settings.dx_sample_points ) );
  }
  
  std::cout << "set up objects" << std::endl;
  
  io_Simulator.frame_idx = in_ObjectList.frame_idx;
  io_Simulator.auto_save_idx = in_ObjectList.auto_save_idx;
  io_Simulator.prev_auto_save_time = in_ObjectList.prev_auto_save_time;
}
