#include "DfisX.hpp"
#include "Daero.hpp"
#include "Dgyro.hpp"

#include "dvd_maths.hpp"

#include <iostream> 
#include <math.h>


namespace DfisX
{
  void step_Dgyro(Throw_Container *throw_container, const float dt, const bool mode)
  {
    // old
    //#moment of inertia      %%%%0.9 is approx k value
    //double Im = mass * (0.9*disc.radius)**2  
    //double Im = d_object.mass * (0.9*0.105)*(0.9*0.105);
    ///below is set as a constant because k and disc.radius are amalgamated into Disc_Model properties
    //double Im = d_object.mass * 0.00893025;

    //new
    // Inertia of a thin disc:
    // Iz =      1/2 * m * r^2
    // Ix = Iy = 1/4 * m * r^2
    const double Ix = 1.0/4.0 * d_object.mass * (d_object.radius*d_object.radius);
    const double Iy = 1.0/4.0 * d_object.mass * (d_object.radius*d_object.radius);
    const double Iz = 1.0/2.0 * d_object.mass * (d_object.radius*d_object.radius);

    // ######Gyroscopic Precession
    //    #Wp  is rads/sec of off axis rotation (rolling)
    //    Wp = Pitching moment / (Moment of intertia * angular velocity (of gyro))
    double Wp = 0.0;
    double Wq = 0.0;
    d_forces.gyro_torque_y = 0.0;
    d_forces.gyro_torque_x = 0.0;
    if(abs(d_state.disc_rotation_vel) > 0.1)
    {
        const float new_pitch_moment = (d_forces.lift_induced_pitching_moment + d_forces.aero_torque_y + d_forces.collision_torque_xyz[1] + throw_container->friction_input.ang_torque_XYZ[1]); // we are already applying the precession in the collicion handler, but this is the resulting collsion torque, hmm
        const float new_roll_moment =  (0.0                                   + d_forces.aero_torque_x + d_forces.collision_torque_xyz[0] + throw_container->friction_input.ang_torque_XYZ[0]);

        Wp = -(new_roll_moment)  / (Iz * d_state.disc_rotation_vel);
        Wq =  (new_pitch_moment) / (Iz * d_state.disc_rotation_vel);
        // compute resulting angular torque from applied pitching moment vel        
        double Wq_d  = (Wq - d_state.disc_rolling_vel)  / MAX(dt, CLOSE_TO_ZERO);
        double Wp_d  = (Wp - d_state.disc_pitching_vel) / MAX(dt, CLOSE_TO_ZERO);
        d_forces.gyro_torque_y = Wp_d * Iy;
        d_forces.gyro_torque_x = Wq_d * Ix;

        // From 2.5 Flight Dynamics of a Spin-stabilised Disc-wing 
        // "Dynamics and Performance of Flying Discs"
        //Pitching moment = Ix * roll accel - 1/2 * pitch rate * yaw rate *Iz
        //Rolling moment = Iy * pitch accel + 1/2 * roll rate * yaw rate *Iz
        // for Z axis defined upward from the disc top
        // similar to above, solve for the induced roll and pitch rates, and compute the torque from that

        if(mode)
        {
            /*Wp = (new_pitch_moment)  / (Iz * d_state.disc_rotation_vel);
            Wr = (new_roll_moment) / (Iz * d_state.disc_rotation_vel);
            // compute resulting angular torque from applied pitching moment vel
            Wp_d  = (Wp - d_state.disc_pitching_vel) / MAX(dt, CLOSE_TO_ZERO);
            Wr_d  = (Wr - d_state.disc_rolling_vel)  / MAX(dt, CLOSE_TO_ZERO);
            d_forces.gyro_torque_y = Wp_d * Iy;
            d_forces.gyro_torque_x = Wr_d * Ix;*/
           /* // just going to pull these back out, but keep things explicit for now
            

            Wp = -(1.0 * (new_roll_moment  - d_state.disc_rolling_accel * Ix*0))  / (Iz * d_state.disc_rotation_vel);
            Wr =  (1.0 * (new_pitch_moment - d_state.disc_pitching_accel * Iy*0)) / (Iz * d_state.disc_rotation_vel);

            Wp_d  = (Wp - d_state.disc_pitching_vel) / MAX(dt, CLOSE_TO_ZERO);
            Wr_d  = (Wr - d_state.disc_rolling_vel)  / MAX(dt, CLOSE_TO_ZERO);
            d_forces.gyro_torque_y = Wp_d * Iy;
            d_forces.gyro_torque_x = Wr_d * Ix;    */        
            
            //d_forces.gyro_torque_y = Ix * d_state.disc_rolling_accel  - 0.5 * d_state.disc_pitching_vel * d_state.disc_rotation_vel * Iz;
            //d_forces.gyro_torque_x = Iy * d_state.disc_pitching_accel + 0.5 * d_state.disc_rolling_vel  * d_state.disc_rotation_vel * Iz;
        }
    }


    //d_forces.gyro_torque_x = Iy * d_state.disc_pitching_accel + 0.5 * d_state.disc_rolling_vel  * d_state.disc_rotation_vel * Iz;

    // does this ever get applied?
    //d_forces.gyro_torque_x = d_forces.lift_induced_pitching_moment;

    // Mike: what do we need this for? is this actually the roll change??
    //d_statistics.disc_cumulative_roll += RAD_TO_DEG(Wp) * dt;

    //always do this after affecting disc_orient_z_vect
    make_unit_vector(d_state.disc_orient_z_vect);
  }
}