

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
    const double Iy = 1.0/4.0 * d_object.mass * (d_object.radius*d_object.radius);
    const double Iz = 1.0/2.0 * d_object.mass * (d_object.radius*d_object.radius);

    //// Compute linear acceleration so we can use that as the integrator for the lower order states
    // Note: collision_force is zero for the moment
    // Add gravity here as well
    d_forces.net_force = d_forces.aero_force + d_forces.collision_force + Eigen::Vector3d(0, 0, (-GRAV * d_object.mass));
    d_state.disc_acceleration = d_forces.net_force / d_object.mass;

    ////// Compute the angular acceleration (just spin and rolling moment for now) so we can use that as the integrator for the lower order states
    //// Pitching moment
    // tack on change to Roll from the gyroscopic precession only
    // we'll need to update this when collisions get added!
    d_forces.net_torque_x = d_forces.gyro_torque_x + d_forces.aero_torque_x + d_forces.collision_torque_xyz[0];
    d_forces.net_torque_y = d_forces.gyro_torque_y + d_forces.aero_torque_y + d_forces.collision_torque_xyz[1];
    // use inertia here to compute the resulting rotation accel (only about 'rolling' axis for now)
    d_state.disc_rolling_accel  = d_forces.net_torque_x / Ix; // roll about X axis (roll to the right positive relative to airspeed vector)
    d_state.disc_pitching_accel = d_forces.net_torque_y / Iy; // pitch about Y axis (nose up positive realtive to airspeed vect)

    //// Spin/Rotation moment
    d_forces.net_torque_z = d_forces.aero_torque_z + d_forces.collision_torque_xyz[2];
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
    // using disc unit vectors to set rotational change only for the rolling axis
    d_forces.gyro_orientation_delta = -d_state.disc_orient_y_vect * tan(d_state.disc_pitching_vel  * dt + d_state.disc_pitching_accel  * dt2);
    // propagate the normal vector using the last rolling vel and accel
    // ADD ON PITCH NOW TOO!
    d_state.disc_orient_z_vect += d_forces.gyro_orientation_delta + (d_state.disc_orient_x_vect * tan(d_state.disc_rolling_vel * dt + d_state.disc_rolling_accel * dt2));
    // re-normalize unit vector
    d_state.disc_orient_z_vect /= d_state.disc_orient_z_vect.norm();
    // update rolling vel for next timestep
    // remember that this is slightly wrong, wince the frame will change by time[k+1], but probably not enough to matter for now
    d_state.disc_rolling_vel  += d_state.disc_rolling_accel   * dt;
    d_state.disc_pitching_vel += d_state.disc_pitching_accel  * dt;

    //// Spinning/rotation axis propagation
    d_state.disc_rotation     += d_state.disc_rotation_vel    * dt + d_state.disc_rotation_accel * dt2;
    d_state.disc_rotation_vel += d_state.disc_rotation_accel  * dt;    
    

    d_forces.step_count++;
  }
}