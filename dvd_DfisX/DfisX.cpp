





#include <fstream>
#include <iostream> 
#include <Eigen/Dense>
#include <typeinfo>   
#include <iomanip>

#include "DfisX.hpp"
#include "Daero.hpp"
#include "Dgyro.hpp"
#include "Dpropagate.hpp"

namespace DfisX
{

Sim_State sim_state = SIM_STATE_STOPPED; 
Throw_Container active_throw;

}


	

namespace DfisX
{

void test() 
{
    std::cout << "Hello, World!\n   this is an exmaple throw\n";
    
    // Standard test
    new_throw (AVIAR,location_throwing_height_origin,Eigen::Vector3d(20,0,2), 0.52, 0.1, 70, 0);

    // from Brandon's matlab KF:
    // (override starting location since it seems to end up in the ground otherwise, haha)
    // maybe the camera was a little offcentre, --> should now be fixed by groundplane rotation and translation!
    // drive15
    //new_throw (AVIAR,Eigen::Vector3d(0.1882 * 0,0.0608 * 0,-1.4049 * 0 + 1.5),Eigen::Vector3d(18.3007,-6.9504,-2.1581), -0.6294, -0.1353, -74.9837*2, 0);
    // drive17
    //new_throw (AVIAR,Eigen::Vector3d(0.2192 * 0,0.4152 * 0,-1.3299 * 0 + 1.5),Eigen::Vector3d(17.7919,-1.6811,-1.9496*0), -0.8069, -0.2617*0, -78.3925, 0);
    // angle4
    //new_throw (AVIAR,Eigen::Vector3d(0.0861 * 0,-0.0154 * 0,-0.7044 * 0 + 1.5),Eigen::Vector3d(6.7812,0.1229,2.9423), -1.3580, -0.0793, -44.8209, 0);
    //new_throw (AVIAR,Eigen::Vector3d(-0.0006 * 0,0.3567 * 0,-1.4461 * 0 + 1.5),Eigen::Vector3d(17.9716,-2.7589,-4.2515), -0.6021, -0.2259, 393.4330, 0);
    simulate_throw();

    
}


void  simulate_throw()
{
    std::cout << "Hello, World!\n   this is a simulated throw\n";


    double total_time = 0;
    std::ofstream myfile;
    myfile.open ("simulated_throw.csv");
    myfile << "time_elapsed,pos_x,pos_y,pos_z,disc_state,orient_x,orient_y,orient_z\n";
    while (d_state.sim_state != SIM_STATE_STOPPED)
    {
    	DfisX::step_simulation (DfisX::active_throw, 0.01);

    	total_time += 0.01;
    	myfile << total_time  << ",";
    	myfile << d_location [0] << ",";
    	myfile << d_location [1] << ",";
    	myfile << d_location [2] << ",";
    	myfile << d_state.sim_state << ",";
    	myfile << d_orientation [0] << ",";
    	myfile << d_orientation [1] << ",";
    	myfile << d_orientation [2] << "\n";
    	
    }
    std::cout << "\n";

    myfile.close();
}













void new_throw (Disc_Mold_Enum disc_mold_enum,Eigen::Vector3d thrown_disc_position,Eigen::Vector3d thrown_disc_velocity, double thrown_disc_roll, double thrown_disc_pitch, double thrown_disc_radians_per_second, double thrown_disc_wobble)
//used to start a new simulation

/*
Takes the following inputs


	
d state
	Eigen::Vector3d thrown_disc_position
	Eigen::Vector3d thrown_disc_velocity
	double thrown_disc_roll
	double thrown_disc_pitch
d force
	double thrown_disc_radians_per_second
	double thrown_disc_wobble	
d object
	Disc_Mold_Enum disc_mold_enum

	


Does the following things

*/
{

	

//convert world frame roll/pitch into and orientation vector
	double x_component = sin (-thrown_disc_pitch) * cos (thrown_disc_roll);
	double y_component = sin (thrown_disc_roll) * cos (thrown_disc_pitch);
	double z_component = cos (thrown_disc_pitch) * cos (thrown_disc_roll);

	Eigen::Vector3d thrown_disc_orientation = {x_component,y_component,z_component};


	double thrown_disc_rotation = 0;

//create the starting d state
	d_state = {thrown_disc_position,thrown_disc_velocity,thrown_disc_orientation,thrown_disc_rotation};
	std::cout << "orientaton is x: " << d_orientation[0] << " y: " << d_orientation[1] << " z: " << d_orientation[2] ;

	d_state.sim_state = SIM_STATE_STARTED;
	std::cout << "new_throw!\n";
	

	d_forces = {};
	d_forces.angular_velocity = thrown_disc_radians_per_second;


	
	p_state = {};



	d_object = disc_aviar;
	d_object.mass = 0.175;
	d_object.diameter = 0.25;
	d_object.radius = d_object.diameter / 2;
	d_object.area = 3.1415 * d_object.radius * d_object.radius;





}







/*
   used to simulate one 'step' of physics

   aiming for 120 hz

 */
void step_simulation (Throw_Container &active_throw, float step_time)
 
{
	
	/*
	step_Daero 				(active_throw);
	step_Dcollision 		(active_throw);
	step_Dgyro 				(active_throw);
	step_Dpropagate 		(active_throw);
	*/

	step_Daero (active_throw, step_time);
	step_Dgyro (active_throw, step_time);
	propagate (active_throw, step_time); 
	std::cout << std::setprecision(3) << std::fixed << "\nX: " << d_location[0] << "   Y: " << d_location[1] << "    Height: " << d_location[2] << "    State: " << d_state.sim_state << "    Velocity: " << d_state.disc_velocity.norm() << "\t    " << std::setprecision(5);

		



    if (active_throw.current_disc_state.disc_location[2] <= 0) 
    ////When to stop the simulation
    {
    	std::cout << std::setprecision(3) << "The throw simulated " << d_forces.step_count << " steps before ending normally.";
    	d_state.sim_state = SIM_STATE_STOPPED;
    }

    else if (d_forces.step_count > 1000)

    {
    	std::cout << "The throw simulation aborted due to reaching maximum steps (1000)";
    	d_state.sim_state = SIM_STATE_STOPPED;
    }
}







Disc_State get_disc_state ()
/*
 used by the control program to get the needed variables for display/ control

 returns 	position vector
			orientation vector
			

*/
{
	std::cout << "get_state\n";
}

Sim_State get_simulation_state ()
/*
 used by the control program to get the needed variable for program state control

 returns 	simulation_state  "flying"/"skipping"/"rolling"/"treehit"/"stopped"

*/
{
	//std::cout << "get_state";
	//return SIM_STATE_FLYING;
}

}