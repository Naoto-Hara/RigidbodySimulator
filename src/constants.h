#ifndef constants_h
#define constants_h

#include <Eigen/Core>

inline Eigen::Matrix2d rot90()
{
  Eigen::Matrix2d res;
  res << 0.0, -1.0, 1.0, 0.0;
  return res;
};

inline Eigen::Matrix2d rotationMat( const double in_theta )
{
  Eigen::Matrix2d res;
  res << cos( in_theta ), -sin( in_theta ), sin( in_theta ), cos( in_theta );
  return res;
}

inline double cross2d( const Eigen::Vector2d& in_a, const Eigen::Vector2d& in_b )
{
  return in_a.x() * in_b.y() - in_a.y() * in_b.x();
}

#endif
