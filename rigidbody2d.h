#ifndef rigid_body_2d_h
#define rigid_body_2d_h

#include <Eigen/Core>
#include <vector>
#include "collisionsample2d.h"
#include "constants.h"

enum RigidBodyType2D
{
  POLYGON,
  CIRCLE
};

class SignedDistanceFunction2D;

inline Eigen::Vector2d getCurrentPosition( const Eigen::Vector2d& in_x0, const Eigen::Vector2d& in_center0, const double in_scale, const double in_theta, const Eigen::Vector2d& in_center_current )
{
  const Eigen::Matrix2d mat{ rotationMat( in_theta ) };
  return mat * ( in_x0 - in_center0 ) * in_scale + in_center_current;
}

inline Eigen::Vector2d getMaterialPosition( const Eigen::Vector2d& in_x_current, const Eigen::Vector2d& in_center0, const double in_scale, const double in_theta, const Eigen::Vector2d& in_center_current )
{
  const Eigen::Matrix2d mat{ rotationMat( in_theta ) };
  return mat.inverse() * ( in_x_current - in_center_current ) / in_scale + in_center0;
}

class ObjectTemplate2D
{
public:
  virtual ~ObjectTemplate2D() {};
  virtual double computeWindingNumber( const Eigen::Vector2d& in_x0 ) const = 0;
  virtual double computeMinimumDistance( const Eigen::Vector2d& in_x0 ) const = 0;
  virtual double getSignedDistanceAt( const Eigen::Vector2d& in_x0 ) const = 0;
  virtual Eigen::Vector2d getNormalAt( const Eigen::Vector2d& in_x0 ) const = 0;
  virtual const SignedDistanceFunction2D* getSDF() const = 0;
  
  virtual Eigen::Vector2d templateCenter() const = 0;
  
  virtual void getBoundingBox( BoundingBox2D& out_BB, const double in_scale, const double in_theta, const Eigen::Vector2d& in_center_current ) const = 0;
  
  virtual RigidBodyType2D type() const = 0;
  
  virtual double computeArea() const = 0;
  virtual double computeSecondMomentOfArea() const = 0;
  
  virtual void generateCollisionSamples( std::vector< CollisionSample2D >& out_CollisionSamples, const double in_dx_sample_points ) const = 0;
};

class RigidBody2D
{
public:
	RigidBody2D( const ObjectTemplate2D* in_Template, const int in_TemplateIndex, const int in_ObjectIndex, const bool in_isStatic, const double in_Density, double in_Scale, const Eigen::Vector2d& in_center_of_mass, double in_Theta, const Eigen::Vector2d& in_V, double in_Omega, const double in_dx_sample_points )
  : m_Scale( in_Scale ), m_isStatic( in_isStatic ), m_TemplateIndex( in_TemplateIndex ), m_ObjectIndex( in_ObjectIndex ), m_x( in_center_of_mass ), m_v( in_V ), m_Theta( in_Theta ), m_Omega( in_Omega ), m_Density( in_Density ), m_Type( in_Template->type() ), m_Template( in_Template ), m_x0( in_Template->templateCenter() ), m_mass( in_Template->computeArea() * in_Density * in_Scale * in_Scale ), m_inertia( in_Template->computeSecondMomentOfArea() * m_mass * in_Scale * in_Scale ), m_force( Eigen::Vector2d::Zero() ), m_torque( 0.0 )
  {
    m_Template->generateCollisionSamples( m_CollisionSamples, in_dx_sample_points );
  }
  
  ~RigidBody2D()
  {}
  
  void initialize()
  {
    clearForceAndTorque();
  }
  
  void getBoundingBox( BoundingBox2D& out_BB ) const
  {
    m_Template->getBoundingBox( out_BB, m_Scale, m_Theta, m_x );
  }
  
  // For signed distance function:
  double getSignedDistanceAt( const Eigen::Vector2d& in_p_current ) const
  {
    const Eigen::Vector2d p0 = getMaterialPosition( in_p_current, m_x0, m_Scale, m_Theta, m_x );
    return m_Template->getSignedDistanceAt( p0 ) * m_Scale;
  }
  
  Eigen::Vector2d getNormalAt( const Eigen::Vector2d& in_p_current ) const
  {
    const Eigen::Matrix2d mat{ rotationMat( m_Theta ) };
    const Eigen::Vector2d p0 = getMaterialPosition( in_p_current, m_x0, m_Scale, m_Theta, m_x );
    
    return mat * m_Template->getNormalAt( p0 );
  }
 
  // For collisions:
  int numCollisionSamples() const { return m_CollisionSamples.size(); }
  const CollisionSample2D& collisionSample( int in_Idx ) const { return m_CollisionSamples[ in_Idx ]; }
  CollisionSample2D& collisionSample( int in_Idx ) { return m_CollisionSamples[ in_Idx ]; }

  // For dynamics:
  int index() const { return m_ObjectIndex; }
  int templateIndex() const { return m_TemplateIndex; }
  
  bool isStatic() const { return m_isStatic; }
  double density() const { return m_Density; }
  
  double mass() const { return m_mass; }
  Eigen::Vector2d centerOfMass() const { return m_x; }
  Eigen::Vector2d velocity() const { return m_v; }
	double rotationAngle() const { return m_Theta; }
	double AngularVelocity() const { return m_Omega; }

  void clearForceAndTorque()
  {
    m_force.setZero();
    m_torque = 0.0;
  }
  
  void accumulateForceAndTorque( const Eigen::Vector2d& in_Force, const double in_Torque )
  {
    m_force += in_Force;
    m_torque += in_Torque;
  }
  
  void stepSymplecticEuler( const double in_dt )
  {
    m_v += m_force * in_dt / m_mass;
    m_Omega += m_torque * in_dt / m_inertia;
    
    m_x += m_v * in_dt;
    m_Theta += m_Omega * in_dt;
  }

  // For mapping reference frame to current frame:
  Eigen::Vector2d getCurrentPosition( const Eigen::Vector2d& in_p0 ) const { return ::getCurrentPosition( in_p0, m_Template->templateCenter(), m_Scale, m_Theta, m_x ); }
  double scale() const { return m_Scale; }
  
  // For display:
  RigidBodyType2D getType() const { return m_Template->type(); }

  const ObjectTemplate2D* getTemplate() const { return m_Template; }

  //’Ç‹L
  void setVelocity( const Eigen::Vector2d& v ) { m_v = v; }
  void setOmega( double o ) { m_Omega = o; }
  void setTheta( double t ) { m_Theta = t; }
  //‚±‚±‚Ü‚Å

private:
  // To be serialized
  double m_Scale;
  bool m_isStatic;
  int m_TemplateIndex;
  int m_ObjectIndex;
  
  Eigen::Vector2d m_x;
  Eigen::Vector2d m_v;
  double m_Theta;
  double m_Omega;
  
  double m_Density;
  
  RigidBodyType2D m_Type;
  
  // Not to be serialized
  const ObjectTemplate2D* m_Template;
  Eigen::Vector2d m_x0;
  
  double m_mass;
  double m_inertia;
  Eigen::Vector2d m_force;
  double m_torque;
  
  std::vector< CollisionSample2D > m_CollisionSamples;
};

#endif





