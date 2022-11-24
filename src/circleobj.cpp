#define EIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT
#define EIGEN_DONT_VECTORIZE

#include "circleobj.h"
#include "signeddistance2d.h"
#include "constants.h"
#include <Eigen/Dense>
#include <iostream>

CircleObjTemplate::CircleObjTemplate( const double in_radius0, const double dx )
: m_Radius0( in_radius0 ), m_SDF( nullptr ), m_x0( Eigen::Vector2d::Zero() )
{
  computeSignedDistanceFunction( dx );
}

double CircleObjTemplate::computeWindingNumber( const Eigen::Vector2d& in_x0 ) const
{
  if( in_x0.norm() > m_Radius0 ) return 0.0;
  else return 1.0;
}

double CircleObjTemplate::computeMinimumDistance( const Eigen::Vector2d& in_x0 ) const
{
  const double s = atan2( in_x0(1), in_x0(0) );
  const Eigen::Vector2d p0{ m_Radius0 * cos(s), m_Radius0 * sin(s) };
  return ( p0 - in_x0 ).norm();
}

double CircleObjTemplate::getSignedDistanceAt( const Eigen::Vector2d& in_x0 ) const
{
  double winding_number = computeWindingNumber( in_x0 );
  double minimumdistance = computeMinimumDistance( in_x0 );
  return winding_number < 0.5 ? minimumdistance : -minimumdistance;
  
  //return m_SDF->signedDistance( in_x0 );
}

Eigen::Vector2d CircleObjTemplate::getNormalAt( const Eigen::Vector2d& in_x0 ) const
{
  //const Eigen::Vector2d n0 = m_SDF->normal( in_x0 );
  const Eigen::Vector2d n0 = in_x0.normalized();
  
  return n0;
}

void CircleObjTemplate::getBoundingBox( BoundingBox2D& out_BB, const double in_scale, const double in_theta, const Eigen::Vector2d& in_center_current ) const
{
  out_BB.bb_min(0) = m_x0(0) - ( m_Radius0 * in_scale ) + in_center_current(0);
  out_BB.bb_min(1) = m_x0(1) - ( m_Radius0 * in_scale ) + in_center_current(1);
  out_BB.bb_max(0) = m_x0(0) + ( m_Radius0 * in_scale ) + in_center_current(0);
  out_BB.bb_max(1) = m_x0(1) + ( m_Radius0 * in_scale ) + in_center_current(1);
}

void CircleObjTemplate::generateCollisionSamples( std::vector< CollisionSample2D >& out_CollisionSamples, const double in_dx_sample_points ) const
{
  out_CollisionSamples.clear();
  const double length = 2.0 * M_PI * m_Radius0;
  const int nSegs = std::max<int>( 2, int( ceil( length / in_dx_sample_points ) ) );
  
  for( int i=0; i<nSegs; i++ )
  {
    const double theta = 2.0 * M_PI * ( i + 0.5 ) / nSegs;
    const Eigen::Vector2d p { m_Radius0 * cos( theta ), m_Radius0 * sin( theta ) };
    CollisionSample2D sample(p);
    out_CollisionSamples.push_back( sample );
  }
}

void CircleObjTemplate::computeSignedDistanceFunction( const double dx )
{
  const Eigen::Vector2d min_C{ -m_Radius0, -m_Radius0 };
  const Eigen::Vector2d max_C{ m_Radius0, m_Radius0 };

  const Eigen::Vector2d center = ( max_C + min_C ) * 0.5;
  const Eigen::Vector2d half_width = ( max_C - min_C ) * 0.5 * 2.0;

  const Eigen::Vector2i resolution = ( half_width * 2.0 / dx ).cast<int>();

  const Eigen::Vector2d gridMin = center - resolution.cast<double>() * dx * 0.5;
  SignedDistanceFunction2D* _sdf = new SignedDistanceFunction2D( gridMin, dx, resolution );
  _sdf->computeSignedDistanceFunction( this );
  m_SDF = _sdf;
}
