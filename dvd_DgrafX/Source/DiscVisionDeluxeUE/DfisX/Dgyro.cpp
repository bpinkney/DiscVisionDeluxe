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
    
    //#moment of inertia      %%%%0.9 is approx k value
    //double Im = mass * (0.9*disc.radius)**2  
    //double Im = d_object.mass * (0.9*0.105)*(0.9*0.105);
    ///below is set as a constant because k and disc.radius are amalgamated into Disc_Model properties
    double Im = d_object.mass * 0.00893025;
    double Wp = 0;

    // ######Gyroscopic Precession
    //    #Wp  is rads/sec of off axis rotation
    //    Wp = Pitching moment / (Moment of intertia * angular velocity (of gyro))
    Wp = d_forces.lift_induced_pitching_moment / (Im * d_state.disc_rotation_vel);

    //using disc vectors to set rotational change to apply to disc_orientation
    d_forces.gyro_orientation_delta = -d_forces.disc_x_unit_vector * (tan(Wp * dt));

    // Mike: what do we need this for? is this actually the roll change??
    d_statistics.disc_cumulative_roll += RAD_TO_DEG(Wp) * dt;

    //always do this after affecting disc_orientation
    make_unit_vector(d_state.disc_orientation);
  }
}