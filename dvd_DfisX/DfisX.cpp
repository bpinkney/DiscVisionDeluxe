//std includes
#include <vector>
#include <fstream>
#include <iostream> 
#include <sstream>
#include <string> 
#include <typeinfo>   
#include <iomanip>
#include <stdio.h>

//lib includes
#include <Eigen/Dense>

//DfisX includes
#include "DfisX.hpp"
#include "Daero.hpp"
#include "Dgyro.hpp"
#include "Dpropagate.hpp"
#include "Dio.hpp"


namespace DfisX
{



//Various inilizations
  Throw_Container active_throw;
  std::vector<DfisX::Disc_Object> disc_object_array;
  Global_Variables global_variables;

  bool   basic_console_logging   = true;
  bool   verbose_console_logging = false;
  double step_time_global        = 0.01;




  void  init()

  {
    set_save_path ("flight_saves\\saved_throw.csv");
    global_variables.global_wind_vel = Eigen::Vector3d(0,0,0);
    global_variables.matlab_export   = false;
    global_variables.install_path    = "C:\\DiscVisionDeluxe";
    load_disc_parameters ();
  }





//Simulate Throw
//simulates the current throw to completion
void  simulate_throw()

{
    if (basic_console_logging) std::cout << "\n This is a simulated throw using a " << d_object.mold_name;
    if (basic_console_logging) std::cout << "\n It is being saved to the file " << global_variables.save_path;

    double total_time = 0;
    std::ofstream myfile;
    myfile.open (global_variables.save_path);
    myfile << "time_elapsed,pos_x,pos_y,pos_z,disc_state,orient_x,orient_y,orient_z\n";
    while (d_state.sim_state != SIM_STATE_STOPPED)
    {
      DfisX::step_simulation (DfisX::active_throw, step_time_global);


      //file saving here temporarily
      // TODO  move to DIO
      total_time += step_time_global;
      myfile << total_time        << ",";
      myfile << d_location [0]    << ",";
      myfile << d_location [1]    << ",";
      myfile << d_location [2]    << ",";
      myfile << d_state.sim_state << ",";
      myfile << d_orientation [0] << ",";
      myfile << d_orientation [1] << ",";
      myfile << d_orientation [2] << "\n";
      
    }
    std::cout << "\n";
    myfile.close();
}





//Step Simulation
//used to simulate one 'step' of physics
void step_simulation (Throw_Container &active_throw, float step_time)

{
  step_Daero (active_throw, step_time);
//step_Dcollision (active_throw, step_time);
  step_Dgyro (active_throw, step_time);
  propagate (active_throw, step_time); 

  if (verbose_console_logging) std::cout << std::setprecision(3) << std::fixed << "\nX: " << d_location[0] << " Y: " << d_location[1] << " Height: " << d_location[2] << " State: " << d_state.sim_state << " Velocity: " << d_state.disc_velocity.norm() << " Spinrate: " << d_forces.angular_velocity << "         " <<std::setprecision(5);
  
  //temporary ground collision detection 
  if (active_throw.current_disc_state.disc_location[2] <= 0) 
  {
    finish_throw        (active_throw);
  }
    //in case of a hang
  else if (d_forces.step_count > 10000)
  {
    std::cout << "\nThe throw simulation aborted due to reaching maximum steps (10000)";
    d_state.sim_state = SIM_STATE_STOPPED;
  }

} 






//Finish Throw
//used calculate various throw statistics and send the saved throw to the desired output
void finish_throw (Throw_Container &throw_container)

{

    ///Time aloft and distance travelled 
    double test_time_aloft = active_throw.disc_state_array.size() * step_time_global;
    std::cout << "\n         The throw spent " << test_time_aloft << "s in the air";
    Eigen::Vector3d throw_distance_vector = active_throw.disc_state_array[0].disc_location - active_throw.disc_state_array[active_throw.disc_state_array.size()-1].disc_location;
    double throw_distance_magnitude = throw_distance_vector.norm();
    std::cout << "\n         The throw went " << throw_distance_magnitude << "m  (" << 3.28*throw_distance_magnitude << "feet)";
    Eigen::Vector3d throw_velocity_vector = active_throw.disc_state_array[0].disc_velocity;
    double throw_velocity_magnitude = throw_velocity_vector.norm();
    std::cout << "\n         The throw's max speed was " << throw_velocity_magnitude << "m/s  (" << 3.28*throw_velocity_magnitude << "feet/s)\n";


    std::cout << std::setprecision(3) << "\nThe throw simulated " << d_forces.step_count << " steps before ending normally.";
    d_state.sim_state = SIM_STATE_STOPPED;
    if (global_variables.matlab_export)
    {
      if (basic_console_logging) std::cout << "\nSending output file to matlab for viewing\n";   
      std::string system_call = ("matlab -nosplash -nodesktop -r \"cd('" + global_variables.install_path + "\\matlab\\visualizers'); dvd_DfisX_plot_disc_trajectory('" + global_variables.install_path + "\\dvd_DfisX\\" + global_variables.save_path + "'); exit\"");
      system(system_call.c_str());
    }
}





//New Throw
//used to start a new simulation
void new_throw (Disc_Mold_Enum disc_mold_enum,Eigen::Vector3d thrown_disc_position,Eigen::Vector3d thrown_disc_velocity, double thrown_disc_roll, double thrown_disc_pitch, double thrown_disc_radians_per_second, double thrown_disc_wobble)
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
  double y_component = sin (thrown_disc_roll)   * cos (thrown_disc_pitch);
  double z_component = cos (thrown_disc_pitch)  * cos (thrown_disc_roll);

  Eigen::Vector3d thrown_disc_orientation = {x_component,y_component,z_component};
  double thrown_disc_rotation = 0;

//create the starting d state
  d_state = {thrown_disc_position,thrown_disc_velocity,thrown_disc_orientation,thrown_disc_rotation};
  d_state.sim_state = SIM_STATE_STARTED;  
  d_forces = {};
  d_forces.angular_velocity = thrown_disc_radians_per_second;
  p_state = {};


  d_object = disc_object_array[disc_mold_enum];
  d_object.mass = 0.175;
  d_object.diameter = 0.25;
  d_object.radius = d_object.diameter / 2;
  d_object.area = 3.1415 * d_object.radius * d_object.radius;

  std::vector <Disc_State> disc_state_array = {};
  d_array.clear ();
}





//Load Disc Parameters
//loads a disc_params.csv file into an array of disc object type
void load_disc_parameters ()

{
    if (basic_console_logging) std::cout << "Loading disc params\n";
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





//Various Getters anmd Setters

void set_save_path(std::string save_path)
{
global_variables.save_path = save_path;
if (basic_console_logging) std::cout << "\n Save file was set to" << save_path << "\n";
}

void set_global_wind_vel (Eigen::Vector3d global_wind_vel)
{
global_variables.global_wind_vel = global_wind_vel;
if (basic_console_logging) std::cout << "\n Global wind was set to  x: " << global_wind_vel[0] << " y: " << global_wind_vel[1] << " z: " << global_wind_vel[2] <<"\n";
}

Eigen::Vector3d get_global_wind_vel ()
{
return global_variables.global_wind_vel;
}

void activate_matlab_export () 
{
global_variables.matlab_export = true;
}





void test(Disc_Mold_Enum disc_mold_enum,Eigen::Vector3d thrown_disc_position,Eigen::Vector3d thrown_disc_velocity,double thrown_disc_roll,double thrown_disc_pitch,double thrown_disc_radians_per_second,double thrown_disc_wobble)

{ 
//potentially unused for now
//commented out code here was moved to bottom of file temporarily for neatness
}


}





/*
const double desired_time_aloft = 3.9;
const double desired_distance = 54;
const double acceptable_variability = 0.9;

std::string generated_disc_name = "BigZ Buzzz";
float generated_mass = 0.175;
float generated_diameter = 0.217;

if (basic_console_logging) std::cout << "\nThis is an example throw using test function";



    //i  lift_coefficient_base                        i
    for (float i = 0.15; i <= 0.3; i+=0.02) {
        //j  lift_coefficient_per_radian              j
        for (float j = 0.1; j <= 0.6; j+=0.05) {
            //k   drag_coefficient                    k
            for (float k = 0.005; k <= 0.06; k+=0.0025) {
                //l   pitching_moment_base            l
                for (float l = -0.06; l <= 0.00; l+=0.01) {
                    //m  pitching_moment_per_radian   m 
                    for (float m = 0.06; m <= 0.24; m+=0.02) {

      std::string generated_mold_params = (generated_disc_name + "," + std::to_string (i)+","+std::to_string (j)+","+std::to_string (k)+","+std::to_string (l)+","+std::to_string (m)+","+std::to_string (generated_mass)+","+std::to_string (generated_diameter));
      set_save_path("flight_saves\\"+generated_mold_params+".csv");
      

   //   disc_object_array.push_back(Disc_Object {generated_disc_name,i,j,k,l,m,generated_mass,generated_diameter});

 //     DfisX::new_throw ((disc_object_array.size()-1),thrown_disc_position,thrown_disc_velocity,thrown_disc_roll,thrown_disc_pitch,thrown_disc_radians_per_second,thrown_disc_wobble);

    // Standard test
    //new_throw (AVIAR,location_throwing_height_origin,Eigen::Vector3d(20,0,2), 0.52, 0.1, 70, 0);

 
    // drive13 ***
   //new_throw (AVIAR,Eigen::Vector3d(0.1991 * 0,0.3180 * 0,1.7996 * 0 + 1.5),Eigen::Vector3d(20.8845,6.0413,0.8772), 0.0924, 0.1999, -77.9917, 0);
    // angle4
    // wobble0
    //new_throw (AVIAR,Eigen::Vector3d(-0.2234 * 0,0.7923 * 0,2.3306 * 0 + 1.5),Eigen::Vector3d(8.8370,2.7146,4.7251), fabs(-0.3033), 0.2270, fabs(-22.8755), 0);
    // putt2
   // new_throw (AVIAR,Eigen::Vector3d(0,0,2),Eigen::Vector3d(11.0294,3.5728,1.6619), 0.2241, 0.1919, -45.9233, 0);
    //spin2
    //new_throw (AVIAR,Eigen::Vector3d(-0.1317 * 0,0.5205 * 0,2.0303 * 0 + 1.5),Eigen::Vector3d(5.4661,0.4827,-0.6882), fabs(0.4373), 0.4299, fabs(-42.6808), 0);

    simulate_throw();
    
/*
    if ((test_time_aloft > desired_time_aloft * (1-acceptable_variability))        && (test_time_aloft < desired_time_aloft * (1+acceptable_variability)) &&
        (throw_distance_magnitude > desired_distance * (1-acceptable_variability)) && (throw_distance_magnitude < desired_distance * (1+acceptable_variability)))
    {
    std::cout << "\nThrow is within acceptable parameters for sweep testing !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!";
    }
    else
    {
    std::cout << "\nThrow is not within acceptable parameters for sweep testing ...............................................";
    std::remove (global_variables.save_path.c_str());
    }


    }}}}}
    */