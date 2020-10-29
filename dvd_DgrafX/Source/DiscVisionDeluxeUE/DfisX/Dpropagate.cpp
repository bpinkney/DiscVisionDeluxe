

#include "Dpropagate.hpp"
#include "DfisX.hpp"

#include "dvd_maths.hpp"

namespace DfisX
{
  void propagate(Throw_Container *throw_container, const float dt)
  {
    const float dt2 = dt*dt  / 2.0;
    const float dt3 = dt*dt2 / 3.0;

    // Inertia of a thin disc:
    // Iz =      1/2 * m * r^2
    // Ix = Iy = 1/4 * m * r^2
    const double Ix = 1.0/4.0 * d_object.mass * (d_object.radius*d_object.radius);
    const double Iz = 1.0/2.0 * d_object.mass * (d_object.radius*d_object.radius);

    //// Compute linear acceleration so we can use that as the integrator for the lower order states
    // Note: collision_force is zero for the moment
    // Add gravity here as well
    d_forces.net_force = d_forces.aero_force + d_forces.collision_force + Eigen::Vector3d(0, 0, (-GRAV * d_object.mass));
    d_state.disc_acceleration = d_forces.net_force / d_object.mass;

    ////// Compute the angular acceleration (just spin and pitching moment for now) so we can use that as the integrator for the lower order states
    //// Pitching moment
    // tack on change to Pitch from the gyroscopic precession only
    // we'll need to update this when collisions get added!
    d_forces.net_torque_x = d_forces.aero_torque_x;
    // use inertia here to compute the resulting rotation accel (only about 'pitching' axis for now)
    d_state.disc_pitching_accel = d_forces.net_torque_x / Ix;

    //// Spin/Rotation moment
    d_forces.net_torque_z = d_forces.aero_torque_z;
    // use inertia here to compute the resulting rotation accel (only about 'spin' axis for now)
    d_state.disc_rotation_accel = d_forces.net_torque_z / Iz;

    // store old state before state propagation
    throw_container->disc_state_array.push_back(d_state);

    ////// Propagate linear states, starting with the lowest order/derivative, 
    // so that propagation occurs using the PREVIOUS [k-1] higher order states
    d_state.disc_location += d_state.disc_velocity      * dt + d_state.disc_acceleration * dt2;
    d_state.disc_velocity += d_state.disc_acceleration  * dt;    

    ////// Propagate angular states, starting with the lowest order/derivative,
    // so that propagation occurs using the PREVIOUS [k-1] higher order states

    //// Pitching axis propagation
    // using disc unit vectors to set rotational change only for the pitching axis
    d_forces.gyro_orientation_delta = -d_forces.disc_x_unit_vector * tan(d_state.disc_pitching_vel * dt + d_state.disc_pitching_accel * dt2);
    // propagate the normal vector using the last pitching vel and accel
    d_state.disc_orientation += d_forces.gyro_orientation_delta; 
    // re-normalize unit vector
    d_state.disc_orientation /= d_state.disc_orientation.norm();
    // update pitching vel for next timestep
    // remember that this is slightly wrong, wince the frame will change by time[k+1], but probably not enough to matter for now
    d_state.disc_pitching_vel += d_state.disc_pitching_accel  * dt;

    //// Spinning/rotation axis propagation
    d_state.disc_rotation     += d_state.disc_rotation_vel    * dt + d_state.disc_rotation_accel * dt2;
    d_state.disc_rotation_vel += d_state.disc_rotation_accel  * dt;    
    

    d_forces.step_count++;
  }
}