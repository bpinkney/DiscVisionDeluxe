





#include <vector>
#include <fstream>
#include <iostream> 
#include <sstream>
#include <string> 
#include <Eigen/Dense>
#include <typeinfo>   
#include <iomanip>

#include "DfisX.hpp"
#include "Daero.hpp"
#include "Dgyro.hpp"
#include "Dpropagate.hpp"

namespace DfisX
{

//Sim_State sim_state = SIM_STATE_STOPPED; 
Throw_Container active_throw;
std::vector<DfisX::Disc_Object> disc_object_array;


	void test() { 
    std::cout << "     This is an example throw\n\n";
    
    // Standard test
    //new_throw (AVIAR,location_throwing_height_origin,Eigen::Vector3d(20,0,2), 0.52, 0.1, 70, 0);

    // from Brandon's matlab KF:
    // (override starting location since it seems to end up in the ground otherwise, haha)
    // maybe the camera was a little offcentre, --> should now be fixed by groundplane rotation and translation!
    // Why does the disc go farther if hyzer and spin are positive (vs. both negative??)
    // throws which actually seem to make some distance (or an appropriate one) are marked with ***
    // we probably need to get some more data with a better ground plane definition
    // possibly collected outside so we can actually track the flight distance (might be simple to use a drone to capture the flight path)

    // drive13 ***
   //new_throw (AVIAR,Eigen::Vector3d(0.1991 * 0,0.3180 * 0,1.7996 * 0 + 1.5),Eigen::Vector3d(20.8845,6.0413,0.8772), fabs(-0.0924), 0.1999, fabs(-77.9917), 0);
    // angle4
    // wobble0
    //new_throw (AVIAR,Eigen::Vector3d(-0.2234 * 0,0.7923 * 0,2.3306 * 0 + 1.5),Eigen::Vector3d(8.8370,2.7146,4.7251), fabs(-0.3033), 0.2270, fabs(-22.8755), 0);
    // putt2
    new_throw (AVIAR,Eigen::Vector3d(0,0,2),Eigen::Vector3d(11.0294,3.5728,1.6619), 0.2241, -0.1919, -45.9233, 0);
    //spin2
    //new_throw (AVIAR,Eigen::Vector3d(-0.1317 * 0,0.5205 * 0,2.0303 * 0 + 1.5),Eigen::Vector3d(5.4661,0.4827,-0.6882), fabs(0.4373), 0.4299, fabs(-42.6808), 0);

    simulate_throw();
}

    



void  simulate_throw()
{
    std::cout << "This is a simulated throw using a " << d_object.mold_name << "\n\n";


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
    load_disc_parameters ();
//convert world frame roll/pitch into and orientation vector
	double x_component = sin (-thrown_disc_pitch) * cos (thrown_disc_roll);
	double y_component = sin (thrown_disc_roll) * cos (thrown_disc_pitch);
	double z_component = cos (thrown_disc_pitch) * cos (thrown_disc_roll);

	Eigen::Vector3d thrown_disc_orientation = {x_component,y_component,z_component};


	double thrown_disc_rotation = 0;

//create the starting d state
	d_state = {thrown_disc_position,thrown_disc_velocity,thrown_disc_orientation,thrown_disc_rotation};
	//std::cout << "orientaton is x: " << d_orientation[0] << " y: " << d_orientation[1] << " z: " << d_orientation[2] ;

	d_state.sim_state = SIM_STATE_STARTED;
	//std::cout << "new_throw!\n";	

	d_forces = {};
	d_forces.angular_velocity = thrown_disc_radians_per_second;
	
	p_state = {};

	d_object = disc_object_array[disc_mold_enum];
	d_object.mass = 0.175;
	d_object.diameter = 0.25;
	d_object.radius = d_object.diameter / 2;
	d_object.area = 3.1415 * d_object.radius * d_object.radius;

}  //end of new throw ()










void step_simulation (Throw_Container &active_throw, float step_time)
//used to simulate one 'step' of physics
{
	
	step_Daero (active_throw, step_time);
	step_Dgyro (active_throw, step_time);
	propagate (active_throw, step_time); 
	std::cout << std::setprecision(3) << std::fixed << "\nX: " << d_location[0] << "   Y: " << d_location[1] << "    Height: " << d_location[2] << "    State: " << d_state.sim_state << "    Velocity: " << d_state.disc_velocity.norm() << "\t    " << std::setprecision(5);


	//temporary ground collision detection 
    if (active_throw.current_disc_state.disc_location[2] <= 0) 
    {
    	std::cout << std::setprecision(3) << "The throw simulated " << d_forces.step_count << " steps before ending normally.";
    	d_state.sim_state = SIM_STATE_STOPPED;
    }
    //in case of a hang
    else if (d_forces.step_count > 1000)
    {
    	std::cout << "The throw simulation aborted due to reaching maximum steps (1000)";
    	d_state.sim_state = SIM_STATE_STOPPED;
    }

} /// end of step_simulation (Throw_Container &active_throw, float step_time)








void load_disc_parameters ()
//loads a disc_params.csv file into an array of disc object type
{
	
    std::cout << "Loading disc params\n";
    std::ifstream disc_params;
    disc_params.open ("disc_params.csv");
     
    char  comma;  // to eat the commas
    std::string mold_name;
    std::string lift_coefficient_base;
    std::string lift_coefficient_per_radian;
    std::string drag_coefficient;
    std::string pitching_moment_base;

    std::string pitching_moment_per_radian;
    std::string mass;
    std::string diameter;
    std::string radius;
    std::string area;

    //while there is data left in the file
    while (disc_params)
    {
        std::string s;
        if (!getline( disc_params, s )) break;
        std::istringstream ss( s );
    
        //each line loads into the variables used to creat a disc_object
        while (ss)
        {
        std::string s;
        getline( ss, mold_name, ',' );
        //std::cout << "\n mold_name: " << mold_name;
        getline( ss, lift_coefficient_base, ',' );
        //std::cout << "\t    lift_coefficient_base:" << lift_coefficient_base;
        getline( ss, lift_coefficient_per_radian, ',' );
        //std::cout << "    lift_coefficient_per_radian:" << lift_coefficient_per_radian;
        getline( ss, drag_coefficient, ',' );
        //std::cout << "    drag_coefficient:" << drag_coefficient;
        getline( ss, pitching_moment_base, ',' );
        //std::cout << "    pitching_moment_base:" << pitching_moment_base;

        getline( ss, pitching_moment_per_radian, ',' );
        //std::cout << "    pitching_moment_per_radian:" << pitching_moment_per_radian;
        getline( ss, mass, ',' );
        //std::cout << "    mass:" << mass;
        getline( ss, diameter, ',' );
        //std::cout << "    diameter:" << diameter;
        disc_object_array.push_back(DfisX::Disc_Object {mold_name,std::stof (lift_coefficient_base),std::stof (lift_coefficient_per_radian),std::stof (drag_coefficient),std::stof (pitching_moment_base),
                                    std::stof (pitching_moment_per_radian),std::stof (mass),std::stof (diameter)});
        break;
    }
}
    disc_params.close();
}  //end of load_disc_parameters ()


}