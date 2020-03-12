
#include "Dpropagate.hpp"
#include "DfisX.hpp"

/*
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


void propagate (Throw_Container &active_throw, float step_time)
{
	d_velocity += d_forces.aero_force / d_object.mass * step_time;

	d_location += d_velocity * step_time;
	d_forces.step_count ++;
	active_throw.previous_disc_state = active_throw.current_disc_state;
}

}