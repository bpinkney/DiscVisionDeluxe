#include "DfisX.hpp"
#include "Daero.hpp"
#include "Dgyro.hpp"

#include "dvd_maths.hpp"

#include <iostream> 
#include <math.h>


namespace DfisX
{
  void step_Dgyro(Throw_Container *throw_container, const float dt)
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
    const double Iz = 1.0/2.0 * d_object.mass * (d_object.radius*d_object.radius);

    // ######Gyroscopic Precession
    //    #Wp  is rads/sec of off axis rotation
    //    Wp = Pitching moment / (Moment of intertia * angular velocity (of gyro))
    const double Wp = d_forces.lift_induced_pitching_moment / (Iz * d_state.disc_rotation_vel);

    // compute resulting angular torque from applied pitching moment vel
    const double Wp_d  = (Wp - d_state.disc_pitching_vel) / MAX(dt, CLOSE_TO_ZERO);
    d_forces.aero_torque_x = Wp_d * Ix;

    // Mike: what do we need this for? is this actually the roll change??
    d_statistics.disc_cumulative_roll += RAD_TO_DEG(Wp) * dt;

    //always do this after affecting disc_orientation
    make_unit_vector(d_state.disc_orientation);
  }
}