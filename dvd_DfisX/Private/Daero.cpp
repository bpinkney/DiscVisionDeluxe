#include "DfisX.hpp"
#include "Daero.hpp"
#include <iostream> 
#include <math.h>
/*




||||||||||||Daero|||||||||||||||||
Handles the aerodynamic forces of disc simulation.
Also gravity.

*/

// unreal junk
#include "ModuleManager.h"
IMPLEMENT_MODULE(FDefaultModuleImpl, dvd_DfisX);
// unreal junk

namespace DfisX
{
///for display purposes     see Sim_State
// determines the division between
// SIM_STATE_FLYING_TURN  -  TURN_CONST   -   SIM_STATE_FLYING   -  FADE_CONST  -  SIM_STATE_FLYING_FADE
const double TURN_CONST = -0.015;
const double FADE_CONST =  0.025;






void             make_unit_vector         (Eigen::Vector3d &vector_to_unitize)
{
    vector_to_unitize /= vector_to_unitize.norm();
}



Eigen::Vector3d  get_unit_vector          (Eigen::Vector3d vector_to_unitize)
{
    return vector_to_unitize /= vector_to_unitize.norm();
}



double           angle_between_vectors    (Eigen::Vector3d a, Eigen::Vector3d b) 
{
    double angle = 0.0;
    angle = std::atan2(a.cross(b).norm(), a.dot(b));
    return angle;
}





//main file function
//this takes a throw container reference and a step time in seconds and performs the aerdynamic force and torque calculations
//step_daero saves these calculations into the throw container
void step_Daero (Throw_Container &active_throw, float step_time)

{


    /* ripped from dfisx.py, naming scheme isnt accurate yet
    #####Unit Vectors
    #vel_unit:  unit vector of total disc velocity
    #lift_unit: unit vector 90 degrees from vel_unit in line with discs 'up' vector
    
    #disc_normal_unit:  unit vector of disc which points 'up' relative to the disc
    #disc_unit_y:  unit vector of velocity vector projected onto discs planes (angle between this and vel_unit is angle of attack)
    #disc_unit_x:  unit vector of disc which points perpendicular to direction of travel but lays on the discs plane
    
    #rotation direction goes from disc_unit_x to disc_unit_y
    """
    */



	Eigen::Vector3d disc_air_velocity_vector = d_velocity - get_global_wind_vel ();



  d_forces.disc_velocity_unit_vector = get_unit_vector (disc_air_velocity_vector);
  make_unit_vector (d_orientation);



////////////////////////////////////////////////////////////Div//By//Zero//Protection///////////////////////////////////////////////////////////////
    //division by zero protection......maybe not necessary
    if (!d_orientation.isApprox(d_forces.disc_velocity_unit_vector))
    {
      d_forces.disc_x_unit_vector = d_forces.disc_velocity_unit_vector.cross(d_orientation);
      make_unit_vector (d_forces.disc_x_unit_vector);
      d_forces.disc_y_unit_vector = d_forces.disc_x_unit_vector.cross(d_orientation);
      make_unit_vector (d_forces.disc_y_unit_vector);

      d_forces.disc_lift_unit_vector = d_forces.disc_x_unit_vector.cross (d_forces.disc_velocity_unit_vector);
    }
    ///divide by zero case (disc is travelling perdicularily through the air)
    else
    {
      d_forces.disc_x_unit_vector =    Eigen::Vector3d    (0,0,0);
      d_forces.disc_y_unit_vector =    Eigen::Vector3d    (0,0,0);
      d_forces.disc_lift_unit_vector = Eigen::Vector3d    (0,0,0);
    if (basic_console_logging) std::cout << "This would produce an error if there was no divide by zero protection in Daero unit vector creation process!!!!!!!!!!!!!!!!!";
    }
////////////////////////////////////////////////////////////Div//By//Zero//Protection/////////////////////////////////////////////////////////








    //////////////////////////////////////////Multiuse Variables/////////////////////////////////
    //#AoAr angle of attack (radians)
    //aoar = vel_unit.angle(disc_normal_unit)-np.deg2rad(90)
    d_forces.aoar = angle_between_vectors (d_forces.disc_velocity_unit_vector, d_orientation) - RAD_90;
    
    //#velocity squared
    //V2 = (vel.magnitude()) ** 2
    d_forces.velocity_magnitude = disc_air_velocity_vector.norm();
    d_forces.v2 =                 d_forces.velocity_magnitude * d_forces.velocity_magnitude;

    //#0.5 * pressure * area * velocity^2
    //pav2by2 = p * a * V2 / 2
    d_forces.pav2by2 = RHO * d_object.area * d_forces.v2 / 2;
    //////////////////////////////////////////Multiuse Variables/////////////////////////////////



    /////////////Calculating the realized flight coefficients////////////////////////////////////
    //normal flight conditions
    if (d_forces.aoar > -0.52 && d_forces.aoar > -0.52)
    {
      d_forces.coefficient_curve =  0.5 * std::sin(6*d_forces.aoar) + std::sin(2*d_forces.aoar);
      d_forces.realized_lift_coefficient =            d_object.lift_coefficient_base +     d_object.lift_coefficient_per_radian * d_forces.coefficient_curve;
      d_forces.realized_pitching_moment_coefficient = d_object.pitching_moment_base  +     d_object.pitching_moment_per_radian  * d_forces.coefficient_curve;
      d_forces.stall_induced_drag = 0.0;

    }
    //stall conditions
    else
    {
      d_forces.stall_curve =         std::sin(2*d_forces.aoar);
      d_forces.realized_lift_coefficient =            d_object.lift_coefficient_base + d_object.lift_coefficient_per_radian * d_forces.stall_curve;
      d_forces.realized_pitching_moment_coefficient = d_object.pitching_moment_base  + d_object.pitching_moment_per_radian  * d_forces.stall_curve;
      d_forces.stall_induced_drag = -std::cos(2*d_forces.aoar)+0.55;
    }
    /////////////Calculating the realized flight coefficients////////////////////////////////////



    /////////////Sim_State calculations for display purposes////////////////////////////////////////
    if      (d_forces.aoar < 0)                                                d_state.sim_state = SIM_STATE_FLYING_HIGH_SPEED_TURN;
    else if (d_forces.realized_pitching_moment_coefficient <= TURN_CONST)      d_state.sim_state = SIM_STATE_FLYING_TURN;
    else if (d_forces.realized_pitching_moment_coefficient >  TURN_CONST && 
    	       d_forces.realized_pitching_moment_coefficient <  FADE_CONST)      d_state.sim_state = SIM_STATE_FLYING;
    else if (d_forces.realized_pitching_moment_coefficient >= FADE_CONST)      d_state.sim_state = SIM_STATE_FLYING_FADE;
    /////////////Sim_State calculations for display purposes////////////////////////////////////////






/*
induced drag Cdi = Cl**2 / pi AR

AR = 1.27 for a circular disc
pi * AR = PI_X_AR = 3.99
 */

    d_forces.induced_drag_coefficient  = d_forces.realized_lift_coefficient * d_forces.realized_lift_coefficient / PI_X_AR;
    d_forces.realized_drag_coefficient = d_object.drag_coefficient + d_forces.induced_drag_coefficient + d_forces.stall_induced_drag;



    d_forces.lift_induced_pitching_moment = d_forces.pav2by2 * d_forces.realized_pitching_moment_coefficient * d_object.diameter;
    d_forces.lift_force_magnitude         = d_forces.pav2by2 * d_forces.realized_lift_coefficient;
    d_forces.drag_force_magnitude         = d_forces.pav2by2 * d_forces.realized_drag_coefficient;



    d_forces.lift_force_vector =  d_forces.lift_force_magnitude * d_forces.disc_lift_unit_vector;
    d_forces.drag_force_vector = -d_forces.drag_force_magnitude * d_forces.disc_velocity_unit_vector;

    d_forces.aero_force = d_forces.lift_force_vector + d_forces.drag_force_vector;
    ///gravity
    d_velocity[2] -= 9.81 * step_time;
    

    
  }
}