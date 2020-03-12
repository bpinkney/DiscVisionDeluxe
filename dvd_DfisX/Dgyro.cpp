#include "DfisX.hpp"
#include "Daero.hpp"
#include "Dgyro.hpp"
#include <iostream> 
#include <math.h>



namespace DfisX
{

double cumulative_roll = 0;
//#moment of inertia      %%%%0.9 is approx k value, update and add to discs%%%%%%
//double Im = mass * (0.9*disc.radius)**2
double Im = 0.175 * (0.9*0.105)*(0.9*0.105);
double Wp = 0;
Eigen::Vector3d orientation_change (0,0,0);

void step_Dgyro (Throw_Container &active_throw, float step_time)
{

// ######Gyroscopic Precession
//    #Wp  is rads/sec of off axis rotation



Wp = lift_induced_pitching_moment / (Im * d_forces.angular_velocity);

//
orientation_change = disc_x_unit_vector * (tan(Wp * step_time));
d_state.disc_orientation += orientation_change;
cumulative_roll +=  lift_induced_pitching_moment * 57.3;
std::cout << "     Total Roll: " << cumulative_roll;

make_unit_vector (d_state.disc_orientation);


//disc_normal_unit= disc_normal_unit.sum(orientation_change)
}

}