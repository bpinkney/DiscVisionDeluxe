





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
    //new_throw (AVIAR,location_throwing_height_origin,Eigen::Vector3d(20,0,2), 0.52, 0.1, 70, 0);

    // from Brandon's matlab KF:
    // (override starting location since it seems to end up in the ground otherwise, haha)
    // maybe the camera was a little offcentre, --> should now be fixed by groundplane rotation and translation!
    // Why does the disc go farther if hyzer and spin are positive (vs. both negative??)
    // throws which actually seem to make some distance (or an appropriate one) are marked with ***
    // we probably need to get some more data with a better ground plane definition
    // possibly collected outside so we can actually track the flight distance (might be simple to use a drone to capture the flight path)

    // drive15
    //new_throw (AVIAR,Eigen::Vector3d(0.1792 * 0,0.5830 * 0,1.6704 * 0 + 1.5),Eigen::Vector3d(18.2379,7.4158,-2.5641), fabs(0.0815), -0.1048, fabs(-79.1582), 0);
    // drive13 ***
    new_throw (AVIAR,Eigen::Vector3d(0.1991 * 0,0.3180 * 0,1.7996 * 0 + 1.5),Eigen::Vector3d(20.8845,6.0413,0.8772), fabs(-0.0924), 0.1999, fabs(-77.9917), 0);
    // angle4
    //new_throw (AVIAR,Eigen::Vector3d(-0.2786 * 0,0.8777 * 0,2.0911 * 0 + 1.5),Eigen::Vector3d(4.1654,-0.1151,-0.7052), fabs(-0.7771), -0.1169, fabs(-45.4723), 0);
    // wobble0
    //new_throw (AVIAR,Eigen::Vector3d(-0.2234 * 0,0.7923 * 0,2.3306 * 0 + 1.5),Eigen::Vector3d(8.8370,2.7146,4.7251), fabs(-0.3033), 0.2270, fabs(-22.8755), 0);
    // putt2
    //new_throw (AVIAR,Eigen::Vector3d(0.5079 * 0,0.8690 * 0,1.8733 * 0 + 1.5),Eigen::Vector3d(11.0294,3.5728,1.6619), fabs(0.2241), -0.1919, fabs(-45.9233), 0);
    //spin2
    //new_throw (AVIAR,Eigen::Vector3d(-0.1317 * 0,0.5205 * 0,2.0303 * 0 + 1.5),Eigen::Vector3d(5.4661,0.4827,-0.6882), fabs(0.4373), 0.4299, fabs(-42.6808), 0);

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