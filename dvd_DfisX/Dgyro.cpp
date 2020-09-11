#include "DfisX.hpp"
#include "Daero.hpp"
#include "Dgyro.hpp"
#include <iostream> 
#include <math.h>



namespace DfisX
{



	void step_Dgyro (Throw_Container &active_throw, float step_time)
	{
		
		//#moment of inertia      %%%%0.9 is approx k value
		//double Im = mass * (0.9*disc.radius)**2  
		//double Im = d_object.mass * (0.9*0.105)*(0.9*0.105);
		///below is set as a constant because k and disc.radius are amalgamated into Disc_Object properties
		double Im = d_object.mass * 0.00893025;
		double Wp = 0;
		Eigen::Vector3d orientation_change (0,0,0);

		// ######Gyroscopic Precession
		//    #Wp  is rads/sec of off axis rotation
        //    Wp = Pitching moment / (Moment of intertia * angular velocity (of gyro))
		Wp = d_forces.lift_induced_pitching_moment / (Im * d_forces.angular_velocity);



		//using disc vectors to apply rotational change to disc_orientation
		orientation_change = d_forces.disc_x_unit_vector * (tan(Wp * step_time));
		d_state.disc_orientation -= orientation_change;
		d_statistics.disc_cumulative_roll += Wp * 57.3 * step_time;

		//always do this after affecting disc_orientation
		make_unit_vector (d_state.disc_orientation);

	}

}