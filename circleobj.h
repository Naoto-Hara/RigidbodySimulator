#ifndef circle_obj_h
#define circle_obj_h

#define _USE_MATH_DEFINES
#include <math.h>
#include "rigidbody2d.h"
#include "constants.h"
#include <vector>

class CircleObjTemplate : public ObjectTemplate2D
{
  CircleObjTemplate();
public:
  CircleObjTemplate( const double in_radius, const double dx );
  ~CircleObjTemplate() { if( m_SDF ){ delete m_SDF; m_SDF = nullptr; } }
  
  double computeWindingNumber( const Eigen::Vector2d& in_x0 ) const;
  double computeMinimumDistance( const Eigen::Vector2d& in_x0 ) const;
  double getSignedDistanceAt( const Eigen::Vector2d& in_x0 ) const;
  Eigen::Vector2d getNormalAt( const Eigen::Vector2d& in_x0 ) const;
  const SignedDistanceFunction2D* getSDF() const { return m_SDF; }
  
  Eigen::Vector2d templateCenter() const { return m_x0; }
      
  void getBoundingBox( BoundingBox2D& out_BB, const double in_scale, const double in_theta, const Eigen::Vector2d& in_center_current ) const;
  
  RigidBodyType2D type() const { return CIRCLE; }
  
  // scales with length^2
  double computeArea() const { return M_PI * m_Radius0 * m_Radius0; }
  // scales with length^4
  double computeSecondMomentOfArea() const { return 0.5 * M_PI * m_Radius0 * m_Radius0 * m_Radius0 * m_Radius0; }
  
  void generateCollisionSamples( std::vector< CollisionSample2D >& out_CollisionSamples, const double in_dx_sample_points ) const;
  
  double getRadius0() const { return m_Radius0; }
  
private:
  void computeSignedDistanceFunction( const double dx );
  
  double m_Radius0;
  const SignedDistanceFunction2D* m_SDF;
  Eigen::Vector2d m_x0;
};

#endif

