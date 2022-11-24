#define EIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT
#define EIGEN_DONT_VECTORIZE

#define _USE_MATH_DEFINES
#include <cmath>
#include <iostream>
#include <assert.h>
#include "simulator.hpp"
#include "rigidbody2d.h"
#include "collisiondetection2d.h"
#include "constants.h"

inline double sgn( const double x )
{
  return x >= 0.0 ? 1.0 : -1.0;
}

double secondRootOfQuadratic( const double& a, const double& b, const double& c, const double& dscr_sqrt )
{
  double root;
  if( b > 0.0 )
  {
    assert( ( -b - dscr_sqrt ) != 0.0 );
    root = ( 2.0 * c ) / ( -b - dscr_sqrt );
    assert(root == root);
  }
  else
  {
    assert( a != 0.0 );
    root = ( -b + dscr_sqrt ) / ( 2.0 * a );
    assert(root == root);
  }
  return root;
}

void stepSystem( Simulator& io_Simulator, double in_dt )
{
  for( int i=0; i<io_Simulator.rigid_bodies.size(); i++ )
  {
    io_Simulator.rigid_bodies[i]->clearForceAndTorque();
  }
  
  //collisionDetection( io_Simulator.rigid_bodies, io_Simulator.frame_idx );
  collisionDetection( io_Simulator.rigid_bodies, io_Simulator.settings.ugrd_dx, &io_Simulator.ugrd, io_Simulator.broad_phase_active_set, io_Simulator.frame_idx );
  
  for( int i=0; i<io_Simulator.rigid_bodies.size(); i++ )
  {
    for( int p=0; p<io_Simulator.rigid_bodies[i]->numCollisionSamples(); p++ )
    {
      for( auto q=io_Simulator.rigid_bodies[i]->collisionSample(p).collision_cache.begin(); q!=io_Simulator.rigid_bodies[i]->collisionSample(p).collision_cache.end(); q++ )
      {
        const Eigen::Vector2d collision_point = io_Simulator.rigid_bodies[i]->getCurrentPosition( io_Simulator.rigid_bodies[i]->collisionSample(p).x0 );
        
        const Eigen::Vector2d arm_i = collision_point - io_Simulator.rigid_bodies[i]->centerOfMass();
        const Eigen::Vector2d arm_j = collision_point - io_Simulator.rigid_bodies[q->first]->centerOfMass();
        
        const Eigen::Vector2d contact_normal = -q->second.normal;
        
        const Eigen::Vector2d v_rel = io_Simulator.rigid_bodies[i]->velocity() + rot90() * arm_i * io_Simulator.rigid_bodies[i]->AngularVelocity() - io_Simulator.rigid_bodies[q->first]->velocity() - rot90() * arm_j * io_Simulator.rigid_bodies[q->first]->AngularVelocity();
        const Eigen::Vector2d vn = contact_normal * contact_normal.dot( v_rel );
        
        const Eigen::Vector2d normal_force = -contact_normal * q->second.penetrationDepth * io_Simulator.settings.kN - vn * io_Simulator.settings.gammaN * 0.5;

        q->second.delta += v_rel * in_dt;
        q->second.delta -= q->second.normal * q->second.normal.dot( q->second.delta );
        const Eigen::Vector2d vt = v_rel - vn;
        
        Eigen::Vector2d friction_force = -q->second.delta * io_Simulator.settings.kT - vt * io_Simulator.settings.gammaT * 0.5;
        const double mu_fn = std::max<double>( 0.0, - io_Simulator.settings.mu * normal_force.norm() * sgn( normal_force.dot( contact_normal ) ) );
        const double ft = friction_force.norm();
        
        if( 0.5 * io_Simulator.settings.gammaT * vt.norm() > mu_fn )
        {
          q->second.delta.setZero();
          friction_force = - vt;
          friction_force.normalize();
          friction_force *= mu_fn;
        }
        else if( ft > mu_fn )
        {
          const double a = io_Simulator.settings.kT * io_Simulator.settings.kT * q->second.delta.squaredNorm();
          assert( a >= 0.0 );
          const double b = io_Simulator.settings.kT * io_Simulator.settings.gammaT * q->second.delta.dot( vt );
          const double c = 0.25 * io_Simulator.settings.gammaT * io_Simulator.settings.gammaT * vt.squaredNorm() - mu_fn * mu_fn;
          const double dscr = b * b - 4.0 * a * c;
          assert( dscr >= 0.0 );
          const double dscr_sqrt = sqrt( dscr );
          double root = secondRootOfQuadratic( a, b, c, dscr_sqrt );
          if( root < 0.0 || root > 1.0 )
          {
            std::cout << "invalid root? " << root << std::endl;
            root = std::max<double>( 0.0, std::min<double>( 1.0, root ) );
          }
          assert( fabs( a * root * root + b * root + c ) <= 1.0e-5 );
          
          const Eigen::Vector2d new_delta = q->second.delta * root;
          const Eigen::Vector2d new_friction_force = -new_delta * io_Simulator.settings.kT - vt * 0.5 * io_Simulator.settings.gammaT;
          assert( fabs( new_friction_force.norm() - mu_fn ) <= 1.0e-5 );
          friction_force = new_friction_force;
          q->second.delta = new_delta;
        }
        
        const Eigen::Vector2d total_force = normal_force + friction_force;
        
        io_Simulator.rigid_bodies[i]->accumulateForceAndTorque( total_force, cross2d( arm_i, total_force ) );
        io_Simulator.rigid_bodies[q->first]->accumulateForceAndTorque( -total_force, -cross2d( arm_j,  total_force ) );
      }
    }
    
    // gravity
    io_Simulator.rigid_bodies[i]->accumulateForceAndTorque( io_Simulator.settings.g * io_Simulator.rigid_bodies[i]->mass() , 0.0 );
  }
  
  for( int i=0; i<io_Simulator.rigid_bodies.size(); i++ )
  {
    if( io_Simulator.rigid_bodies[i]->isStatic() ) continue;
    io_Simulator.rigid_bodies[i]->stepSymplecticEuler( in_dt );
  }
  
  io_Simulator.frame_idx++;
}
