

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
    // unit vector X and Z are changed by a change to pitch (about Y)
    // unit vector Y and Z are changed by a change to roll (about X)
    // Deprecated method, keep this here for reference for now
   /* d_forces.gyro_orientation_delta = 
          (d_state.disc_orient_x_vect * tan(d_state.disc_pitching_vel * dt + d_state.disc_pitching_accel * dt2)) +
         -(d_state.disc_orient_y_vect * tan(d_state.disc_rolling_vel  * dt + d_state.disc_rolling_accel  * dt2));
    //d_forces.gyro_orientation_delta = -d_state.disc_orient_y_vect * tan(d_state.disc_pitching_vel  * dt + d_state.disc_pitching_accel  * dt2);
    // propagate the normal vector using the last rolling vel and accel
    // ADD ON PITCH NOW TOO!
    d_state.disc_orient_z_vect += d_forces.gyro_orientation_delta;*/
    // re-normalize unit vector
    //d_state.disc_orient_z_vect /= d_state.disc_orient_z_vect.norm();

    // Determine the change to the Z unit vector due to the roll and pitch changes (rotate in that order!)
    // Should we be updating the X and Y unit vectors here as well?? They are not orthonormal after this if not
    const float roll_propagation_rad  = d_state.disc_rolling_vel  * dt + d_state.disc_rolling_accel  * dt2;
    const float pitch_propagation_rad = d_state.disc_pitching_vel * dt + d_state.disc_pitching_accel * dt2;

    // compute the change in rotation we need to apply to the Z unit vector by using a rotation matrix
    // Note this is column major, so it is really the transpose of what you see here.
    Eigen::Matrix3d Rx; 
    Rx <<   1, 0, 0,
            0, cos(roll_propagation_rad), -sin(roll_propagation_rad),
            0, sin(roll_propagation_rad),  cos(roll_propagation_rad);
    // per the note above, we need to transpose this to get column major (which makes the most sense with how this is written out! bah Eigen!)
     Rx = Rx.transpose();
        // Note this is column major, so it is really the transpose of what you see here.
    Eigen::Matrix3d Ry; 
    Ry <<   cos(pitch_propagation_rad), 0, sin(pitch_propagation_rad),
            0, 1, 0,
            -sin(pitch_propagation_rad), 0, cos(pitch_propagation_rad);
    // per the note above, we need to transpose this to get column major (which makes the most sense with how this is written out! bah Eigen!)
    Ry = Ry.transpose();

    // formulate the rotation matrix to bring this back to the world frame next
    // Rememnber this is column-major (ok, so I just wrote this one backward)
    Eigen::Matrix3d Rdw; 
    Rdw << d_state.disc_orient_x_vect[0], d_state.disc_orient_y_vect[0], d_state.disc_orient_z_vect[0],
           d_state.disc_orient_x_vect[1], d_state.disc_orient_y_vect[1], d_state.disc_orient_z_vect[1],
           d_state.disc_orient_x_vect[2], d_state.disc_orient_y_vect[2], d_state.disc_orient_z_vect[2];

    // it is perhaps simpler to just compute the z unit vector change for the two axes?
    // this almost works around the roll -> pitch sequential rotation losses?
    Eigen::Vector3d base_z = {0,0,1};

    Eigen::Vector3d z_unit_delta = {0,0,0};
    z_unit_delta += (Rx * base_z - base_z);
    z_unit_delta += (Ry * base_z - base_z);

    z_unit_delta = Rdw * z_unit_delta;
    // Update the vector
    d_state.disc_orient_z_vect += z_unit_delta;

    // Now let's re-compute the unit X and Y vectors (like we do in Daero) to ensure that they are still
    // orthonormal to the disx Z normal
    // re-normalize just in case
    d_state.disc_orient_z_vect /= d_state.disc_orient_z_vect.norm();
    d_state.disc_orient_y_vect = d_forces.disc_velocity_unit_vector.cross(d_state.disc_orient_z_vect);
    d_state.disc_orient_y_vect /= d_state.disc_orient_y_vect.norm();
    d_state.disc_orient_x_vect = d_state.disc_orient_y_vect.cross(d_state.disc_orient_z_vect);
    d_state.disc_orient_x_vect /= d_state.disc_orient_x_vect.norm();


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