#ifndef polygon_obj_h
#define polygon_obj_h

#define _USE_MATH_DEFINES
#include <math.h>
#include "rigidbody2d.h"
#include "constants.h"
#include <vector>

// m_Scale( in_Scale ), m_Density( in_Density ), m_isStatic( in_isStatic ), m_TemplateIndex( in_CircleTemplate_Index ), m_Index( in_ObjectIndex ), m_x( in_x0 ), m_v( in_V ), m_force( Eigen::Vector2d::Zero() ), m_theta( in_Theta ), m_omega( in_Omega ), m_torque( 0.0 )

//using PolygonObjTemplate = std::vector< Eigen::Vector2d >;


class PolygonObjTemplate : public ObjectTemplate2D
{
  PolygonObjTemplate();
public:
  PolygonObjTemplate( const std::vector< Eigen::Vector2d >& in_Vertices0, const double dx );
  ~PolygonObjTemplate() { if( m_SDF ){ delete m_SDF; m_SDF = nullptr; } }
  
  double computeWindingNumber( const Eigen::Vector2d& in_x0 ) const;
  double computeMinimumDistance( const Eigen::Vector2d& in_x0 ) const;
  double getSignedDistanceAt( const Eigen::Vector2d& in_x0 ) const;
  Eigen::Vector2d getNormalAt( const Eigen::Vector2d& in_x0 ) const;
  const SignedDistanceFunction2D* getSDF() const { return m_SDF; }
  
  Eigen::Vector2d templateCenter() const { return m_x0; }
      
  void getBoundingBox( BoundingBox2D& out_BB, const double in_scale, const double in_theta, const Eigen::Vector2d& in_center_current ) const;
  
  RigidBodyType2D type() const { return POLYGON; }
  
  // scales with length^2
  double computeArea() const;
  // scales with length^4
  double computeSecondMomentOfArea() const;
  
  int numVertices() const { return m_Vertices0.size(); }
  Eigen::Vector2d getVertexPosition0( const int in_Vertex_Idx0 ) const { return m_Vertices0[ in_Vertex_Idx0 ]; }
  const std::vector< Eigen::Vector2d >& getVertices0() const { return m_Vertices0; }
  
  void generateCollisionSamples( std::vector< CollisionSample2D >& out_CollisionSamples, const double in_dx_sample_points ) const;
  
private:
  void computeInitialCenterOfMass();
  void setCenterOfMassToOrigin();
  
  double computeThetaFromSinCos( const Eigen::Vector2d& in_v, const Eigen::Vector2d& in_x0, const Eigen::Vector2d& in_x1 ) const;
  double computeDistanceToSegment( const Eigen::Vector2d& in_Point, const Eigen::Vector2d& in_SegmentFrom, const Eigen::Vector2d& in_SegmentTo ) const;
  double computeDistanceToSegment( const Eigen::Vector2d& in_Point, const Eigen::Vector2d& in_SegmentFrom, const Eigen::Vector2d& in_SegmentTo, Eigen::Vector2d& out_ClosestPoint ) const;
  Eigen::Vector2d computeClosestPoint( const Eigen::Vector2d& in_x0 ) const;
    
  void computeSignedDistanceFunction( const double dx );
  
  std::vector< Eigen::Vector2d > m_Vertices0;
  const SignedDistanceFunction2D* m_SDF;
  Eigen::Vector2d m_x0;
};

#endif

