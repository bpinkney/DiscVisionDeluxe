#include "DfisX.hpp"
#include "Daero.hpp"
#include <iostream> 
#include <math.h>
/*




||||||||||||Daero|||||||||||||||||
Handles the aerodynamic forces of disc simulation.
Also gravity.

Does not apply the forces into velocity or displacement
Except for gravity.





Throw Container
	Forces_State 
	Disc_State 
	Disc_State  (prev)
	Disc_Mold 



struct Disc_State
//the location, orientation, and velocity of a disc
//everything needed for display purposes

	Eigen::Vector3d disc_location;  	//in meters
	Eigen::Vector3d disc_orientation;	//orientation vector is the normal of the plane inscribed by the disc
	Eigen::Vector3d disc_velocity;  	//in meters per second
	double disc_rotation;    			//in radians

struct Forces_State

double force_sum;
double acceleration_sum;
double angular_velocity;   //radians_per_second

double aero_force;
double aero_torque;

double collision_force;
double collision_torque;

*/
namespace DfisX
{



Eigen::Vector3d disc_x_unit_vector (0,0,0);
Eigen::Vector3d disc_y_unit_vector (0,0,0);
Eigen::Vector3d disc_lift_unit_vector (0,0,0);
Eigen::Vector3d disc_velocity_unit_vector (0,0,0);
Eigen::Vector3d lift_force_vector (0,0,0);
Eigen::Vector3d drag_force_vector (0,0,0);

double aoar = 0.0;
double velocity_magnitude = 0.0;
double v2 = 0.0;
double pav2by2 = 0.0;

double coefficient_curve = 0.0;
double stall_curve = 0.0;

double lift_force_magnitude = 0.0;
double drag_force_magnitude = 0.0;

double realized_lift_coefficient = 0.0;
double realized_drag_coefficient = 0.0;
double induced_drag_coefficient = 0.0;
double realized_pitching_moment_coefficient = 0.0;
double stall_induced_drag = 0.0;

double lift_induced_pitching_moment = 0.0;




void 				make_unit_vector 		(Eigen::Vector3d &vector_to_unitize)

{
	vector_to_unitize /= vector_to_unitize.norm();
}



Eigen::Vector3d 	get_unit_vector 		(Eigen::Vector3d vector_to_unitize)

{
	return vector_to_unitize /= vector_to_unitize.norm();
}



double				angle_between_vectors	(Eigen::Vector3d a, Eigen::Vector3d b) 

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
	disc_velocity_unit_vector = get_unit_vector (d_velocity);
	make_unit_vector (d_orientation);


	//division by zero protection......maybe not necessary
	if (!d_orientation.isApprox(disc_velocity_unit_vector))
	{
		disc_x_unit_vector = disc_velocity_unit_vector.cross(d_orientation);
		make_unit_vector (disc_x_unit_vector);
		disc_y_unit_vector = disc_x_unit_vector.cross(d_orientation);
		make_unit_vector (disc_y_unit_vector);

		disc_lift_unit_vector = disc_x_unit_vector.cross (disc_velocity_unit_vector);
	}
	///divide by zero case (disc is travelling perdicularily through the air)
	else
	{
		disc_x_unit_vector =Eigen::Vector3d (0,0,0);
		disc_y_unit_vector =Eigen::Vector3d (0,0,0);
		disc_lift_unit_vector =Eigen::Vector3d (0,0,0);
		std::cout << "This would produce an error if there was no divide by zero protection in Daero unit vector creation process!!!!!!!!!!!!!!!!!";
	}



    //#####Multiuse Variables
    //#AoAr angle of attack (radians)
	//aoar = vel_unit.angle(disc_normal_unit)-np.deg2rad(90)
	aoar = angle_between_vectors (disc_velocity_unit_vector, d_orientation) - RAD_90;
	
    //#velocity squared
    //V2 = (vel.magnitude()) ** 2
    velocity_magnitude = d_velocity.norm();
    v2 = velocity_magnitude * velocity_magnitude;

    //#0.5 * pressure * area * velocity^2
    //pav2by2 = p * a * V2 / 2
    pav2by2 = RHO * d_object.area * v2 / 2;




//////Calculating the realized flight coefficients
    //non stall conditions
    if (aoar > -0.52 && aoar > -0.52)
    {
    	coefficient_curve = 0.5 * std::sin(6*aoar) + std::sin(2*aoar);
    	realized_lift_coefficient = 		 d_object.lift_coefficient_base + 	d_object.lift_coefficient_per_radian*coefficient_curve;
		realized_pitching_moment_coefficient = d_object.pitching_moment_base + 	d_object.pitching_moment_per_radian	*coefficient_curve;
		stall_induced_drag = 0.0;

    }
    //stall conditions
    else
    {
    	stall_curve = std::sin(2*aoar);
    	realized_lift_coefficient = 		 d_object.lift_coefficient_base + 	d_object.lift_coefficient_per_radian*stall_curve;
		realized_pitching_moment_coefficient = d_object.pitching_moment_base + 	d_object.pitching_moment_per_radian	*stall_curve;
		stall_induced_drag = -std::cos(2*aoar)+0.55;
    }



/*Aerial State detection  ..... Not working yet 
if 		(aoar < 0) 																							sim_state = SIM_STATE_FLYING_HIGH_SPEED_TURN;
else if (realized_pitching_moment_coefficient <= -0.0005) 													sim_state = SIM_STATE_FLYING_TURN;
else if (realized_pitching_moment_coefficient > -0.0005 && realized_pitching_moment_coefficient < 0.0005) 	sim_state = SIM_STATE_FLYING;
else if (realized_pitching_moment_coefficient >= 0.0005) 													sim_state = SIM_STATE_FLYING_FADE;


std::cout << "   rpc:" << realized_pitching_moment_coefficient;
*/
std::cout << "   aoar:" << aoar;
//std::cout << "\n \n orientaton is x: " << d_orientation[0] << " y: " << d_orientation[1] << " z: " << d_orientation[2] ;




/*
induced drag Cdi = Cl**2 / pi AR

AR = 1.27 for a circular disc
pi * AR = PI_X_AR = 3.99
 */

    induced_drag_coefficient = realized_lift_coefficient * realized_lift_coefficient / PI_X_AR;
    realized_drag_coefficient = d_object.drag_coefficient + induced_drag_coefficient + stall_induced_drag;



    lift_induced_pitching_moment = pav2by2 * realized_pitching_moment_coefficient * d_object.diameter;
	lift_force_magnitude = pav2by2 * realized_lift_coefficient;
    drag_force_magnitude = pav2by2 * realized_drag_coefficient;



    lift_force_vector =  lift_force_magnitude * disc_lift_unit_vector;
    drag_force_vector = -drag_force_magnitude * disc_velocity_unit_vector;

	d_forces.aero_force = lift_force_vector + drag_force_vector;

	d_velocity[2] -= 9.81 * step_time;
    

    
}


}