

#include "Dpropagate.hpp"
#include "DfisX.hpp"

#include "dvd_maths.hpp"

namespace DfisX
{
  const double hacky_spin_drag_rate = 5.0;

  void propagate(Throw_Container *throw_container, const float dt)
  {
    const float dt2 = dt*dt  / 2.0;
    const float dt3 = dt*dt2 / 3.0;

    //// Compute linear acceleration so we can use that as the integrator for the lower order states
    // Note: collision_force is zero for the moment
    // Add gravity here as well
    d_forces.net_force = d_forces.aero_force + d_forces.collision_force + Eigen::Vector3d(0, 0, (-GRAV * d_object.mass));
    d_state.disc_acceleration = d_forces.net_force / d_object.mass;

    //// Compute the angular acceleration (just spin for now) so we can use that as the integrator for the lower order states
    //d_forces.aero_torque = _____; // this should get moved to DAero!
    //d_forces.net_torque = ____; // use inertia here to compute the resulting rotation accel
    d_state.disc_rotation_accel = -signum(d_state.disc_rotation_vel) * hacky_spin_drag_rate;

    // store old state before state propagation
    throw_container->disc_state_array.push_back(d_state);

    //// Propagate linear states, starting with the lowest order/derivative, 
    // so that propagation occurs using the PREVIOUS [k-1] higher order states
    d_state.disc_location += d_state.disc_velocity      * dt + d_state.disc_acceleration * dt2;
    d_state.disc_velocity += d_state.disc_acceleration  * dt;    

    //// Propagate angular states, starting with the lowest order/derivative,
    // so that propagation occurs using the PREVIOUS [k-1] higher order states
    d_state.disc_rotation     += d_state.disc_rotation_vel    * dt + d_state.disc_rotation_accel * dt2;
    d_state.disc_rotation_vel += d_state.disc_rotation_accel  * dt;

    // tack on change to Roll/Pitch DIRECTLY from the gyroscopic precession
    // we'll need to update this when collisions get added!
    d_state.disc_orientation += d_forces.gyro_orientation_delta;

    d_forces.step_count++;
  }
}