#pragma once
#include <Eigen/Dense>


#define d_velocity 		active_throw.current_disc_state.disc_velocity
#define d_location 		active_throw.current_disc_state.disc_location
#define d_orientation 	active_throw.current_disc_state.disc_orientation
#define d_state 		active_throw.current_disc_state
#define p_state 		active_throw.previous_disc_state
#define d_forces 		active_throw.forces_state
#define d_object 		active_throw.disc_object

#define RAD_360 	6.28318531
#define RAD_180 	3.14159265
#define RAD_90 		1.57079633


#define RHO 		1.225
#define PI_X_AR 	3.99

namespace DfisX
{

enum Disc_Mold_Enum
{
	AVIAR,
	SHARK,
	LEOPARD
	
};


//SIMULATION STATE

enum Sim_State
{
	SIM_STATE_STOPPED,
	SIM_STATE_STARTED,
	SIM_STATE_FLYING_HIGH_SPEED_TURN,
	SIM_STATE_FLYING_TURN,
	SIM_STATE_FLYING,
	SIM_STATE_FLYING_FADE,
	SIM_STATE_SKIPPING,
	SIM_STATE_TREE_HIT,
	SIM_STATE_ROLLING,
	SIM_STATE_SLIDING
	
};

void test();
void simulate_throw();



//used to 'step' the physics simulation forward once





struct Disc_State
//the location, orientation, and velocity of a disc
//everything needed for display purposes
{
	Eigen::Vector3d disc_location;  	//in meters
	Eigen::Vector3d disc_velocity;  	//in meters per second
	Eigen::Vector3d disc_orientation;	//orientation vector is the normal of the plane inscribed by the disc
	double disc_rotation;    			//in radians
	Sim_State sim_state; 				//describes the current state of the disc
};

const Eigen::Vector3d location_throwing_height_origin (0,0,1);

const Eigen::Vector3d direction_up (0,0,1);
const Eigen::Vector3d direction_down (0,0,-1);

const Disc_State default_hard_throw 	{location_throwing_height_origin,	direction_up,	Eigen::Vector3d(22,0,2),0};
const Disc_State default_soft_throw		{location_throwing_height_origin,	direction_up,	Eigen::Vector3d(15,0,0.75),0};
const Disc_State default_putting_throw 	{location_throwing_height_origin,	direction_up,	Eigen::Vector3d(10,0,0.5),0};




struct Forces_State
//simulation step variables
//not needed for display
{
Eigen::Vector3d force_sum;
double acceleration_sum;
double angular_velocity;   //radians_per_second

Eigen::Vector3d aero_force;
double aero_torque;

Eigen::Vector3d collision_force;
Eigen::Vector3d collision_location;
//Eigen::Vector3d collision_angle;
double collision_torque;

int step_count;

};




struct Disc_Object
//contains the name and the aerodynamic properties of a disc mold
//all the static variables
{
std::string mold_name;
float lift_coefficient_base;
float lift_coefficient_per_radian;
float drag_coefficient;
float pitching_moment_base;

float pitching_moment_per_radian;
float mass;
float diameter;
float radius;
float area;
};

const Disc_Object disc_aviar {"Aviar",0.18,0.5,0.015,-0.025,	0.25,0.175,0.210};
//const Disc_Object disc_aviar {"Aviar",0.15,0.44,0.055,-0.0010,	0.010,0.175,0.210};



struct Throw_Container
//contains all of the info needed to simulate and display a disc
{
Forces_State forces_state;
Disc_State current_disc_state;
Disc_State previous_disc_state;
Disc_Object disc_object;
};




void step_simulation (Throw_Container &throw_container, float step_time);

Disc_State get_disc_state ();
//used to return the variables needed to display/save a disc flight

void new_throw (Disc_Mold_Enum disc_mold_enum,Eigen::Vector3d thrown_disc_position,Eigen::Vector3d thrown_disc_velocity, double thrown_disc_roll, double thrown_disc_pitch, double thrown_disc_radians_per_second, double thrown_disc_wobble);
/*	Used to initialize a new throw simulation 
	
*/	






Sim_State get_simulation_state ();

}