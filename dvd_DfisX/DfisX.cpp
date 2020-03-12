






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


	

int main() 
{
    std::cout << "Hello, World!\n   this is an exmaple throw\n";
    DfisX::new_throw (DfisX::disc_aviar,DfisX::default_hard_throw,20, 0);
    
    while (DfisX::sim_state != DfisX::SIM_STATE_STOPPED)
    {
    	DfisX::step_simulation (DfisX::active_throw, 0.01);
    	
    }
}

void test() 
{
    std::cout << "Hello, World!\n   this is an exmaple throw\n";
    DfisX::new_throw (DfisX::disc_aviar,DfisX::default_hard_throw,20, 0);
    
    while (DfisX::sim_state != DfisX::SIM_STATE_STOPPED)
    {
    	DfisX::step_simulation (DfisX::active_throw, 0.01);
    	
    }
}


void simulate_throw() 
{
    std::cout << "Hello, World!\n   this is a simulated throw\n";
    
    while (DfisX::sim_state != DfisX::SIM_STATE_STOPPED)
    {
    	DfisX::step_simulation (DfisX::active_throw, 0.01);
    	
    }
}




namespace DfisX
{








void new_throw (Disc_Object thrown_disc_object, Disc_State thrown_disc_state, double thrown_disc_radians_per_second, double thrown_disc_wobble)
//used to start a new simulation

/*
Takes the following inputs
	
	thrown_disc_object				a struct which defines the aerodynamic qualities of a disc
	thrown_disc_state				a struct which contains location,orientation,velocity vectors and rotation
	thrown_disc_radians_per_second	a double
	double thrown_disc_wobble		a double whichs describe the amount of off axis rotation of a thrown disc (needs to be defined better)

Does the following things

	
*/
{
	sim_state = SIM_STATE_STARTED;
	std::cout << "new_throw!\n";
	
	d_forces = {};
	d_forces.angular_velocity = thrown_disc_radians_per_second;


	d_state = thrown_disc_state;
	p_state = {};



	d_object = thrown_disc_object;
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
	std::cout << std::setprecision(3) << std::fixed << "\nX: " << d_location[0] << "   Y: " << d_location[1] << "    Height: " << d_location[2] << "    State: " << sim_state << "    Velocity: " << d_state.disc_velocity.norm() << "\t    " << std::setprecision(5);

		



    if (active_throw.current_disc_state.disc_location[2] <= 0) 
    ////When to stop the simulation
    {
    	std::cout << std::setprecision(3) << "The throw simulated " << d_forces.step_count << " steps before ending normally.";
    	sim_state = SIM_STATE_STOPPED;
    }

    else if (d_forces.step_count > 1000)

    {
    	std::cout << "The throw simulation aborted due to reaching maximum steps";
    	sim_state = SIM_STATE_STOPPED;
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