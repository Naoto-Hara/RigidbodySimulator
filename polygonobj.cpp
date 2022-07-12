#define EIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT
#define EIGEN_DONT_VECTORIZE

#include "polygonobj.h"
#include "signeddistance2d.h"
#include "constants.h"
#include <Eigen/Dense>

PolygonObjTemplate::PolygonObjTemplate( const std::vector< Eigen::Vector2d >& in_Vertices0, const double dx )
: m_Vertices0( in_Vertices0 ), m_SDF( nullptr )
{
  computeInitialCenterOfMass();
  setCenterOfMassToOrigin();
  computeSignedDistanceFunction( dx );
}

double PolygonObjTemplate::computeWindingNumber( const Eigen::Vector2d& in_x0 ) const
{
  double totalRad = 0.0;
  for( int i = 0; i < m_Vertices0.size(); i++ )
  {
    const int ip = ( i + 1 ) % m_Vertices0.size();
    totalRad += computeThetaFromSinCos( in_x0, m_Vertices0[i], m_Vertices0[ip] );
  }
  return totalRad / ( 2.0 * M_PI );
}

double PolygonObjTemplate::computeMinimumDistance( const Eigen::Vector2d& in_x0 ) const
{
  double minDistance = 1.0e33;
  for( int i = 0; i < m_Vertices0.size(); i++ )
  {
    const int ip = ( i + 1 ) % m_Vertices0.size();
    double _dist = computeDistanceToSegment( in_x0, m_Vertices0[i], m_Vertices0[ip] );
    minDistance = std::min<double>( minDistance, _dist );
  }
  return minDistance;
}

double PolygonObjTemplate::getSignedDistanceAt( const Eigen::Vector2d& in_x0 ) const
{
  double winding_number = computeWindingNumber( in_x0 );
  double minimumdistance = computeMinimumDistance( in_x0 );
  return winding_number < 0.5 ? minimumdistance : -minimumdistance;
  
  // return m_SDF->signedDistance( in_x0 );
}

Eigen::Vector2d PolygonObjTemplate::getNormalAt( const Eigen::Vector2d& in_x0 ) const
{
  const Eigen::Vector2d q0 = computeClosestPoint( in_x0 );
  double winding_number = computeWindingNumber( in_x0 );
  
  const Eigen::Vector2d n0 = winding_number < 0.5 ? ( in_x0 - q0 ).normalized() : ( q0 - in_x0 ).normalized();
  
  // const Eigen::Vector2d n0 = m_SDF->normal( in_x0 );
  
  return n0;
}

void PolygonObjTemplate::getBoundingBox( BoundingBox2D& out_BB, const double in_scale, const double in_theta, const Eigen::Vector2d& in_center_current ) const
{
  out_BB.bb_min(0) = 1.0e33; out_BB.bb_min(1) = 1.0e33;
  out_BB.bb_max(0) = -1.0e33; out_BB.bb_max(1) = -1.0e33;

  for( int i = 0; i < m_Vertices0.size(); i++ )
  {
    const Eigen::Vector2d p = getCurrentPosition( m_Vertices0[i], m_x0, in_scale, in_theta, in_center_current );
    out_BB.bb_min(0) = std::min<double>( out_BB.bb_min(0), p(0) );
    out_BB.bb_min(1) = std::min<double>( out_BB.bb_min(1), p(1) );
    out_BB.bb_max(0) = std::max<double>( out_BB.bb_max(0), p(0) );
    out_BB.bb_max(1) = std::max<double>( out_BB.bb_max(1), p(1) );
  }
}

double PolygonObjTemplate::computeArea() const
{
  double area = 0.0;
  for( int i=0; i<m_Vertices0.size(); i++ )
  {
    int ip = ( i+1 ) % m_Vertices0.size();
    int im = ( i-1+m_Vertices0.size() ) % m_Vertices0.size();
    
    area += m_Vertices0[i](0) * ( m_Vertices0[ip](1) - m_Vertices0[im](1) );
  }
  return area * 0.5;
}

double PolygonObjTemplate::computeSecondMomentOfArea() const
{
  double _I = 0.0;
  for( int i=0; i<m_Vertices0.size(); i++ )
  {
    int ip = (i+1) % m_Vertices0.size();
    int im = (i-1+m_Vertices0.size()) % m_Vertices0.size();
    
    const double _Iy_x12 = ( m_Vertices0[i](0) * m_Vertices0[ip](1) - m_Vertices0[ip](0) * m_Vertices0[i](1) ) * ( m_Vertices0[i](0) * m_Vertices0[i](0) + m_Vertices0[i](0) * m_Vertices0[ip](0) + m_Vertices0[ip](0) * m_Vertices0[ip](0) );
    
    const double _Ix_x12 = ( m_Vertices0[i](0) * m_Vertices0[ip](1) - m_Vertices0[ip](0) * m_Vertices0[i](1) ) * ( m_Vertices0[i](1) * m_Vertices0[i](1) + m_Vertices0[i](1) * m_Vertices0[ip](1) + m_Vertices0[ip](1) * m_Vertices0[ip](1) );
    
    _I += _Ix_x12 + _Iy_x12;
  }
  return _I / 12.0;
}




/* Private functions */

void PolygonObjTemplate::computeInitialCenterOfMass()
{
  m_x0.setZero();
  for( int i = 0; i < m_Vertices0.size(); i++ )
  {
    m_x0 += m_Vertices0[i];
  }
  m_x0 = m_x0 / m_Vertices0.size();
}

void PolygonObjTemplate::setCenterOfMassToOrigin()
{
  for( int i = 0; i < m_Vertices0.size(); i++ )
  {
    m_Vertices0[i] -= m_x0;
  }
  m_x0.setZero();
}

double PolygonObjTemplate::computeDistanceToSegment( const Eigen::Vector2d& in_Point, const Eigen::Vector2d& in_SegmentFrom, const Eigen::Vector2d& in_SegmentTo ) const
{
  const double t = ( in_SegmentTo - in_SegmentFrom ).dot( in_Point - in_SegmentFrom ) / ( in_SegmentTo - in_SegmentFrom ).squaredNorm();
  const Eigen::Vector2d Pt = in_SegmentFrom + ( in_SegmentTo - in_SegmentFrom ) * t;
  const double distance = ( Pt - in_Point ).norm();

  if( t < 0 )
    return ( in_SegmentFrom - in_Point ).norm();
  else if( t > 1 )
    return ( in_SegmentTo - in_Point ).norm();
  else
    return distance;
}

double PolygonObjTemplate::computeDistanceToSegment( const Eigen::Vector2d& in_Point, const Eigen::Vector2d& in_SegmentFrom, const Eigen::Vector2d& in_SegmentTo, Eigen::Vector2d& out_ClosestPoint ) const
{
  const double t = ( in_SegmentTo - in_SegmentFrom ).dot( in_Point - in_SegmentFrom ) / ( in_SegmentTo - in_SegmentFrom ).squaredNorm();
  const Eigen::Vector2d Pt = in_SegmentFrom + ( in_SegmentTo - in_SegmentFrom ) * t;
  const double distance = ( Pt - in_Point ).norm();

  if( t < 0 )
  {
    out_ClosestPoint = in_SegmentFrom;
    return ( in_SegmentFrom - in_Point ).norm();
  }
  else if( t > 1 )
  {
    out_ClosestPoint = in_SegmentTo;
    return ( in_SegmentTo - in_Point ).norm();
  }
  else
  {
    out_ClosestPoint = Pt;
    return distance;
  }
}

double PolygonObjTemplate::computeThetaFromSinCos( const Eigen::Vector2d& in_v, const Eigen::Vector2d& in_x0, const Eigen::Vector2d& in_x1 ) const
{
  double numeratorSin = 0.0;
  double numeratorCos = 0.0;
  double denominator = 0.0;

  numeratorSin = cross2d( in_x0 - in_v, in_x1 - in_v );
  denominator = ( in_x0 - in_v ).norm() * ( in_x1 - in_v ).norm();
  double sinTheta = numeratorSin / denominator;

  numeratorCos = ( in_x0 - in_v ).dot( in_x1 - in_v );
  double cosTheta = numeratorCos / denominator;

  return atan2( sinTheta, cosTheta );
}

void PolygonObjTemplate::generateCollisionSamples( std::vector< CollisionSample2D >& out_CollisionSamples, const double in_dx_sample_points ) const
{
  out_CollisionSamples.clear();
  for( int k = 0; k < numVertices(); k++ )
  {
    const int kp = ( k + 1 ) % numVertices();
    const Eigen::Vector2d xk0 = m_Vertices0[k];
    const Eigen::Vector2d xkp0 = m_Vertices0[kp];
    const double length = ( xkp0 - xk0 ).norm();
    
    const int nSegs = std::max<int>( 2, int( ceil( length / in_dx_sample_points ) ) );
    
    for( int i=0; i<nSegs; i++ )
    {
      const Eigen::Vector2d p = xk0 + ( xkp0 - xk0 ) * ( double(i) / double(nSegs) );
      CollisionSample2D sample(p);
      out_CollisionSamples.push_back( sample );
    }
  }
}

Eigen::Vector2d PolygonObjTemplate::computeClosestPoint( const Eigen::Vector2d& in_x0 ) const
{
  double closest_dist = 1.0e33;
  Eigen::Vector2d res;
  for( int i=0; i<m_Vertices0.size(); i++ )
  {
    const int ip = ( i + 1 ) % m_Vertices0.size();
    Eigen::Vector2d cp;
    const double _dist = computeDistanceToSegment( in_x0, m_Vertices0[i], m_Vertices0[ip], cp );
    if( _dist < closest_dist )
    {
      closest_dist = _dist;
      res = cp;
    }
  }
  
  return res;
}


void PolygonObjTemplate::computeSignedDistanceFunction( const double dx )
{
  double min_x = 1.0e33;
  double min_y = 1.0e33;
  double max_x = -1.0e33;
  double max_y = -1.0e33;

  for( int i = 0; i < m_Vertices0.size(); i++ )
  {
    min_x = std::min<double>( min_x, m_Vertices0[i](0) );
    min_y = std::min<double>( min_y, m_Vertices0[i](1) );
    max_x = std::max<double>( max_x, m_Vertices0[i](0) );
    max_y = std::max<double>( max_y, m_Vertices0[i](1) );
  }

  const Eigen::Vector2d min_C{ min_x, min_y };
  const Eigen::Vector2d max_C{ max_x, max_y };

  const Eigen::Vector2d center = ( max_C + min_C ) * 0.5;
  const Eigen::Vector2d half_width = ( max_C - min_C ) * 0.5 * 2.0;

  const Eigen::Vector2i resolution = ( half_width * 2.0 / dx ).cast<int>();

  const Eigen::Vector2d gridMin = center - resolution.cast<double>() * dx * 0.5;
  SignedDistanceFunction2D* _sdf = new SignedDistanceFunction2D( gridMin, dx, resolution );
  _sdf->computeSignedDistanceFunction( this );
  m_SDF = _sdf;
}
