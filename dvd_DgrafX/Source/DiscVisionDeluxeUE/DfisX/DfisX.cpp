//std includes
#include <vector>
#include <fstream>
#include <iostream> 
#include <sstream>
#include <string> 
#include <typeinfo>   
#include <iomanip>
#include <stdio.h>
#include <stdexcept>

//lib includes
#include <Eigen/Dense>

//DfisX includes
#include "DfisX.hpp"
#include "Daero.hpp"
#include "Dgyro.hpp"
#include "Dpropagate.hpp"
#include "Dio.hpp"

#include "disc_params.hpp"
#include "dvd_maths.hpp"

//#define basic_console_logging   (true)
//#define verbose_console_logging (false)

// these should all be considered static functions, and should contain NO REFERECES to global variables

namespace DfisX
{

  void consume_Dcollision(Throw_Container *throw_container, const float dt)
  {
    const bool disable = 0;

    const float collision_apply_frames = 1;
    if(!disable && throw_container->collision_input.consumed_input < collision_apply_frames && throw_container->collision_input.delta_time_s > 0)
    {
      throw_container->current_disc_state.forces_state.collision_torque_xyz[0] = 
      throw_container->current_disc_state.forces_state.collision_torque_xyz[1] = 
      throw_container->current_disc_state.forces_state.collision_torque_xyz[2] = 0.0;

      // Compute forces and torques
      // This is just a direct copy maybe?
      throw_container->current_disc_state.forces_state.collision_force[0] = throw_container->collision_input.normal_force_N[0];
      throw_container->current_disc_state.forces_state.collision_force[1] = throw_container->collision_input.normal_force_N[1];
      throw_container->current_disc_state.forces_state.collision_force[2] = throw_container->collision_input.normal_force_N[2];

      // Apply some random friction for now since this doesn't seem to be included.....
      const float mu = 0.4; // grass??
      Eigen::Vector3d lin_vel_unit = throw_container->current_disc_state.disc_velocity / throw_container->current_disc_state.disc_velocity.norm();
      const float friction_force_N = mu * throw_container->collision_input.normal_force_N.norm();
      //throw_container->current_disc_state.forces_state.collision_force += -lin_vel_unit * friction_force_N;

      /*throw_container->current_disc_state.forces_state.collision_force[0] += 
        mu * sqrt(throw_container->collision_input.normal_force_N[1]*throw_container->collision_input.normal_force_N[1] + throw_container->collision_input.normal_force_N[2]*throw_container->collision_input.normal_force_N[2]) * 
        -signum(throw_container->current_disc_state.disc_velocity[0]);
      throw_container->current_disc_state.forces_state.collision_force[1] += 
        mu * sqrt(throw_container->collision_input.normal_force_N[0]*throw_container->collision_input.normal_force_N[0] + throw_container->collision_input.normal_force_N[2]*throw_container->collision_input.normal_force_N[2]) * 
        -signum(throw_container->current_disc_state.disc_velocity[1]);
      throw_container->current_disc_state.forces_state.collision_force[2] += 
        mu * sqrt(throw_container->collision_input.normal_force_N[1]*throw_container->collision_input.normal_force_N[1] + throw_container->collision_input.normal_force_N[0]*throw_container->collision_input.normal_force_N[0]) * 
        -signum(throw_container->current_disc_state.disc_velocity[2]);*/


      // get velocity change?
     // Eigen::Vector3d lin_vel = (throw_container->collision_input.disc_position_m - throw_container->current_disc_state.disc_location) / throw_container->collision_input.delta_time_s;

      // override vel?
      //throw_container->current_disc_state.disc_velocity[0] = lin_vel[0];
      //throw_container->current_disc_state.disc_velocity[1] = lin_vel[1];
      //throw_container->current_disc_state.disc_velocity[2] = lin_vel[2];

      // override position states?
      //throw_container->current_disc_state.disc_location[0] = throw_container->collision_input.disc_position_m[0];
      //throw_container->current_disc_state.disc_location[1] = throw_container->collision_input.disc_position_m[1];
      //throw_container->current_disc_state.disc_location[2] = throw_container->collision_input.disc_position_m[2];

      // determine relative hit location in local disc frames:
      Eigen::Vector3d relative_hit_location_world_m = throw_container->collision_input.hit_location_m - throw_container->collision_input.disc_position_m;
      Eigen::Vector3d relative_hit_torque_world_Nm = throw_container->current_disc_state.forces_state.collision_force.cwiseProduct(relative_hit_location_world_m);

      const double Ix = 1.0/4.0 * throw_container->disc_object.mass * (throw_container->disc_object.radius*throw_container->disc_object.radius);
      const double Iy = 1.0/4.0 * throw_container->disc_object.mass * (throw_container->disc_object.radius*throw_container->disc_object.radius);
      const double Iz = 1.0/2.0 * throw_container->disc_object.mass * (throw_container->disc_object.radius*throw_container->disc_object.radius);

      // hackily use the unit vectors to find porjection component of this moment arm in disc local frame (this needs to be revised, it is almost certainly wrong)

      Eigen::Vector3d a;
      Eigen::Vector3d b;
      a = throw_container->current_disc_state.forces_state.disc_x_unit_vector;
      b = relative_hit_torque_world_Nm;
      Eigen::Vector3d proj_hit_world_onto_local_x = (a.dot(b) * a) / (a.norm()*a.norm());
      a = throw_container->current_disc_state.forces_state.disc_y_unit_vector;
      b = relative_hit_torque_world_Nm;
      Eigen::Vector3d proj_hit_world_onto_local_y = (a.dot(b) * a) / (a.norm()*a.norm());
      a = throw_container->current_disc_state.disc_orientation;
      b = relative_hit_torque_world_Nm;
      Eigen::Vector3d proj_hit_world_onto_local_z = (a.dot(b) * a) / (a.norm()*a.norm());

      // Get angular torques in local frame
      Eigen::Vector3d local_hit_torque_xyz_m = proj_hit_world_onto_local_x + proj_hit_world_onto_local_y + proj_hit_world_onto_local_z;
      throw_container->current_disc_state.forces_state.collision_torque_xyz[0] = -local_hit_torque_xyz_m[1];
      throw_container->current_disc_state.forces_state.collision_torque_xyz[1] = -local_hit_torque_xyz_m[0]; // roll about y axis
      //throw_container->current_disc_state.forces_state.collision_torque_xyz[2] = local_hit_torque_xyz_m[2];
      
      // get local frame rotations? (hacky)
/*      Eigen::Vector3d a;
      Eigen::Vector3d b;
      a = throw_container->current_disc_state.forces_state.disc_x_unit_vector;
      b = throw_container->collision_input.ang_vel_delta_radps;
      Eigen::Vector3d ang_vel_delta_radps_local_x = (a.dot(b) * a) / (a.norm()*a.norm());
      a = throw_container->current_disc_state.forces_state.disc_y_unit_vector;
      b = throw_container->collision_input.ang_vel_delta_radps;
      Eigen::Vector3d ang_vel_delta_radps_local_y = (a.dot(b) * a) / (a.norm()*a.norm());
      a = throw_container->current_disc_state.disc_orientation;
      b = throw_container->collision_input.ang_vel_delta_radps;
      Eigen::Vector3d ang_vel_delta_radps_local_z = (a.dot(b) * a) / (a.norm()*a.norm());

      Eigen::Vector3d ang_vel_delta_radps_local = ang_vel_delta_radps_local_x + ang_vel_delta_radps_local_y + ang_vel_delta_radps_local_z;*/

      //const double spin_ang_accel = throw_container->collision_input.ang_vel_delta_radps[2] / throw_container->collision_input.delta_time_s;
      //throw_container->current_disc_state.forces_state.collision_torque_xyz[0] = -ang_vel_delta_radps_local[0]/throw_container->collision_input.delta_time_s * Ix;
      //throw_container->current_disc_state.forces_state.collision_torque_xyz[1] = -ang_vel_delta_radps_local[1]/throw_container->collision_input.delta_time_s * Iy;
      // why is this negative???
      throw_container->current_disc_state.forces_state.collision_torque_xyz[2] = -throw_container->collision_input.ang_vel_delta_radps[2]/throw_container->collision_input.delta_time_s * Iz;

      // janked up friction
      //const float mu = 50.4; // grass??
    /*  const float normal_force_N = sqrt(
        throw_container->current_disc_state.forces_state.collision_force[0]*throw_container->current_disc_state.forces_state.collision_force[0] +
        throw_container->current_disc_state.forces_state.collision_force[1]*throw_container->current_disc_state.forces_state.collision_force[1] +
        throw_container->current_disc_state.forces_state.collision_force[2]*throw_container->current_disc_state.forces_state.collision_force[2]);
      const float F_friction = normal_force_N * mu;

      // just assume that we are using the full moment arm of the disc for friction spin torque
      const float t_friction = F_friction * throw_container->disc_object.radius;*/

      /*const float r5 = (throw_container->disc_object.radius * throw_container->disc_object.radius * throw_container->disc_object.radius * throw_container->disc_object.radius * throw_container->disc_object.radius);

      const float t_friction_pitch =
        -signum(throw_container->current_disc_state.disc_pitching_vel) *
        0.5 * 
        (throw_container->current_disc_state.disc_pitching_vel * throw_container->current_disc_state.disc_pitching_vel) * 
        r5 *
        mu;

      const float t_friction_roll =
        -signum(throw_container->current_disc_state.disc_rolling_vel) *
        0.5 * 
        (throw_container->current_disc_state.disc_rolling_vel * throw_container->current_disc_state.disc_rolling_vel) * 
        r5 *
        mu;

      const float t_friction_spin =
        -signum(throw_container->current_disc_state.disc_rotation_vel) *
        0.5 * 
        (throw_container->current_disc_state.disc_rotation_vel * throw_container->current_disc_state.disc_rotation_vel) * 
        r5 *
        mu;*/

      // apply in the opposite direction to angular spin vel
      //throw_container->current_disc_state.forces_state.collision_torque_xyz[0] = t_friction_pitch;
      //throw_container->current_disc_state.forces_state.collision_torque_xyz[1] = t_friction_roll; // roll about y axis
      //throw_container->current_disc_state.forces_state.collision_torque_xyz[2] = t_friction_spin;


      throw_container->collision_input.consumed_input++;

    }
    else
    {
      throw_container->current_disc_state.forces_state.collision_force[0] = 
      throw_container->current_disc_state.forces_state.collision_force[1] = 
      throw_container->current_disc_state.forces_state.collision_force[2] = 0.0;

      throw_container->current_disc_state.forces_state.collision_torque_xyz[0] = 
      throw_container->current_disc_state.forces_state.collision_torque_xyz[1] = 
      throw_container->current_disc_state.forces_state.collision_torque_xyz[2] = 0.0;
    }
  }

  //Simulate Throw
  //simulates the current throw to completion
  void simulate_throw(Throw_Container *throw_container, const float dt)
  {
    while (throw_container->current_disc_state.sim_state != SIM_STATE_STOPPED)
    {
      DfisX::step_simulation(throw_container, dt);
    }
  }

  //Step Simulation
  //used to simulate one 'step' of physics
  void step_simulation(Throw_Container *throw_container, const float dt)
  {
    // all steps must be completed before propagation
    step_Daero(throw_container, dt);
    // step_Dcollision (throw_container, dt);
    step_Dgyro(throw_container, dt);
    // consume incoming collisions
    consume_Dcollision(throw_container, dt);

    propagate(throw_container, dt); 

    //temporary ground collision detection
    if (throw_container->current_disc_state.disc_location[2] <= 0) 
    {
      finish_throw(throw_container, dt);
    }
      //in case of a hang
    else if (throw_container->current_disc_state.forces_state.step_count > 10000)
    {
      std::cout << "The throw simulation aborted due to reaching maximum steps (10000)\n";
      throw_container->current_disc_state.sim_state = SIM_STATE_STOPPED;
    }
  } 

  //Finish Throw
  //used calculate various throw statistics and send the saved throw to the desired output
  void finish_throw(Throw_Container *throw_container, const float dt)
  {
    ///Time aloft and distance travelled 
    double test_time_aloft = throw_container->disc_state_array.size() * dt;
    std::cout << "\n         The throw spent " << test_time_aloft << "s in the air";
    Eigen::Vector3d throw_distance_vector = throw_container->disc_state_array[0].disc_location - throw_container->disc_state_array[throw_container->disc_state_array.size()-1].disc_location;
    double throw_distance_magnitude = throw_distance_vector.norm();
    std::cout << "\n         The throw went " << throw_distance_magnitude << "m  (" << 3.28*throw_distance_magnitude << "feet)";
    Eigen::Vector3d throw_velocity_vector = throw_container->disc_state_array[0].disc_velocity;
    double throw_velocity_magnitude = throw_velocity_vector.norm();
    std::cout << "\n         The throw's max speed was " << throw_velocity_magnitude << "m/s  (" << 2.236*throw_velocity_magnitude << "mph)\n";

    std::cout << std::setprecision(3) << "The throw simulated " << throw_container->current_disc_state.forces_state.step_count << " steps before ending normally.\n";
    throw_container->current_disc_state.sim_state = SIM_STATE_STOPPED;
    /*if (dfisx_config.matlab_export)
    {
      if (basic_console_logging) std::cout << "\nSending output file to matlab for viewing...\n";   
      std::string system_call = ("matlab -nosplash -nodesktop -r \"cd('" + dfisx_config.install_path + "\\matlab\\visualizers'); dvd_DfisX_plot_disc_trajectory('" + dfisx_config.install_path + "\\dvd_DfisX\\" + dfisx_config.save_path + "'); exit\"");
      system(system_call.c_str());
    }*/
  }

  bool is_finished_simulating (Throw_Container *throw_container)
  {
    if (throw_container->current_disc_state.sim_state == SIM_STATE_STOPPED) return true;
    else                                        return false;
  }

  std::vector <Disc_State> get_disc_state_array (Throw_Container *throw_container)
  {
    return (throw_container->disc_state_array);
  }

  static uint32_t find_disc_mold_index_by_name(std::string disc_mold_name)
  {
    uint32_t i;
    for(i = 0; i < disc_object_array.size(); i++)
    {
      if(disc_object_array[i].mold_name == disc_mold_name)
      {
        return i;
      }
    }

    // return zero if there is no mapping
    return 0;
  }

  //New Throw
  //used to start a new simulation
  void new_throw(
    Throw_Container *throw_container, 
    const DiscIndex disc_index,
    const Eigen::Vector3d thrown_disc_position,
    const Eigen::Vector3d thrown_disc_velocity, 
    const double thrown_disc_roll, 
    const double thrown_disc_pitch, 
    const double thrown_disc_radians_per_second, 
    const double thrown_disc_wobble)
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
      DiscIndex disc_index

    
  Does the following things
  */
  {  
    // init throw container
    //memset(throw_container, 0, sizeof(Throw_Container));

    // don't clear Disc_Env or Disc_Model in case we want to pass those in
    // during init from the GUI
    memset(&(throw_container->current_disc_state),  0, sizeof(Disc_State));
    memset(&(throw_container->previous_disc_state), 0, sizeof(Disc_State));
    memset(&(throw_container->disc_statistics),     0, sizeof(Disc_Statistics));
    memset(&(throw_container->previous_disc_state), 0, sizeof(Disc_State));
    throw_container->disc_state_array.clear();

    //convert world frame roll/pitch into and orientation vector

    const double x_component = sin (-thrown_disc_pitch) * cos (thrown_disc_roll);
    const double y_component = sin (thrown_disc_roll)   * cos (thrown_disc_pitch);
    const double z_component = cos (thrown_disc_pitch)  * cos (thrown_disc_roll);

    const Eigen::Vector3d thrown_disc_orientation = {x_component,y_component,z_component};
    const double thrown_disc_rotation = 0;

    //create the starting d state
    throw_container->current_disc_state.disc_location = thrown_disc_position;
    throw_container->current_disc_state.disc_velocity = thrown_disc_velocity;
    throw_container->current_disc_state.disc_orientation = thrown_disc_orientation;
    throw_container->current_disc_state.disc_rotation = thrown_disc_rotation;
    throw_container->current_disc_state.sim_state = SIM_STATE_STARTED;  
    throw_container->current_disc_state.forces_state = {};
    throw_container->current_disc_state.disc_rotation_vel = thrown_disc_radians_per_second;
    throw_container->previous_disc_state = {};

    // build an abitrary 'bag' for now so we have a mapping between the DiscIndex and DiscModel
    // this will be handled by a user mapping later, and probably an encapsulating object for DiscModel
    uint32_t disc_mold = 0;
    switch(disc_index)
    {
      default:
      case DiscIndex::NONE:
        disc_mold = find_disc_mold_index_by_name("Mako3");
        break;
      // tested discs with data
      case DiscIndex::MIDRANGE:
        disc_mold = find_disc_mold_index_by_name("Buzzz Foil");
        break;
      case DiscIndex::FAIRWAY:
        disc_mold = find_disc_mold_index_by_name("TeeBird");
        break;
      case DiscIndex::DRIVER:
        disc_mold = find_disc_mold_index_by_name("Shryke");
        break;
      case DiscIndex::DRIVER_OS:
        disc_mold = find_disc_mold_index_by_name("Destroyer");
        break;
      case DiscIndex::PUTTER:
        disc_mold = find_disc_mold_index_by_name("Magnet");
        break;
      case DiscIndex::PUTTER_OS:
        disc_mold = find_disc_mold_index_by_name("Zone");
        break;
      // untested discs, just to fill out the bag
      case DiscIndex::PUTTER_US:
        disc_mold = find_disc_mold_index_by_name("Luna");
        break;
      case DiscIndex::MIDRANGE_OS:
        disc_mold = find_disc_mold_index_by_name("Zombee");
        break;
      case DiscIndex::MIDRANGE_US:
        disc_mold = find_disc_mold_index_by_name("Leopard3");
        break;
      case DiscIndex::FAIRWAY_OS:
        disc_mold = find_disc_mold_index_by_name("DDX");
        break;
      case DiscIndex::FAIRWAY_US:
        disc_mold = find_disc_mold_index_by_name("Leopard3");
        break;
      case DiscIndex::DRIVER_US:
        disc_mold = find_disc_mold_index_by_name("DDX");
        break;
      case DiscIndex::SPECIAL:
        disc_mold = find_disc_mold_index_by_name("P3");
        break;
    }    

    if(1)
    {
      // generate a random disc mold index from all possible sets
      //const float index = (float)rand() / (float)RAND_MAX;
      //disc_mold = floor(index * disc_object_array.size());

      // OR just override explicitly
      //disc_mold = find_disc_mold_index_by_name("Mako3");

      // OR just proceed through all of them in order
      // (skipping the brick)
      static uint32_t disc2throw = 1;
      if(0)
      {
        disc2throw++;
        if(disc2throw > disc_object_array.size() - 1)
        {
          disc2throw = 1;
        }
        disc_mold = disc2throw;
      }

      // OR throw a cycling set for test purposes
      // REALLY handy for tuning, try throwing a:
      // - straight midrange or putter with not much camber (e.g. Mako3) 
      // - understable or stable driver with camber and a thick rim (e.g. destroyer)
      // - stable fairway driver with a thick lower rim camber
      // If you can get all these flying OK, you've got a decent tuning!

      //disc_mold = find_disc_mold_index_by_name("Destroyer");

      if(1)
      {
        switch(disc2throw)
        {
          case 1:
            disc_mold = find_disc_mold_index_by_name("Wraith");
            disc2throw = 2;
            break;
          case 2:
            disc_mold = find_disc_mold_index_by_name("Pure");
            disc2throw = 3;
            break;
          case 3:
            disc_mold = find_disc_mold_index_by_name("Envy");
            disc2throw = 4;
            break;
          case 4:
            disc_mold = find_disc_mold_index_by_name("Swan");
            disc2throw = 5;
            break;
          case 5:
            disc_mold = find_disc_mold_index_by_name("Buzzz");
            disc2throw = 6;
            break;
          default:
            disc_mold = find_disc_mold_index_by_name("Luna");
            disc2throw = 1;
            break;
        }
      }    
    }

    // override for Loft Test Throws
    if(1)
    {
      static int test_throw = 0;
      throw_container->current_disc_state.disc_location[1] = 0.0;
      throw_container->current_disc_state.disc_location[0] = 1.0;
      throw_container->current_disc_state.disc_velocity = {80.0/3.6, 0, 0};
      throw_container->current_disc_state.disc_rotation_vel = -125.6637; // 1200 rpm righty backhand

      //disc_mold = find_disc_mold_index_by_name("Valkyrie");
      //disc_mold = find_disc_mold_index_by_name("Hydrogen");


      Eigen::Vector3d hps = {0,0,0};
      switch(test_throw)
      {
        case 0:
          hps = {DEG_TO_RAD(0), DEG_TO_RAD(0), DEG_TO_RAD(0)};
          //test_throw++;
        break;
        case 1:
          hps = {DEG_TO_RAD(15), DEG_TO_RAD(0), DEG_TO_RAD(0)};
          test_throw++;
        break;
        case 2:
          hps = {DEG_TO_RAD(-15), DEG_TO_RAD(0), DEG_TO_RAD(0)};
          test_throw = 0;
        break;
      }

      const double loft_x_component = sin (-hps[1]) * cos (hps[0]);
      const double loft_y_component = sin (hps[0])  * cos (hps[1]);
      const double loft_z_component = cos (hps[1])  * cos (hps[0]);

      Eigen::Vector3d loft_thrown_disc_orientation = {loft_x_component, loft_y_component, loft_z_component};

      throw_container->current_disc_state.disc_orientation = loft_thrown_disc_orientation;
    }

    throw_container->disc_object = disc_object_array[disc_mold];

    std::cout << std::endl << std::endl << "Throwing a " << throw_container->disc_object.manufacturer << " " << throw_container->disc_object.mold_name << std::endl;

    throw_container->disc_state_array.clear();


/////////// used to populate disc states
    step_simulation(throw_container, 0.001);
  }

  // overloaded
  

  //Various Getters and Setters
  Disc_State get_disc_state(Throw_Container *throw_container)
  {
    return throw_container->current_disc_state;
  }

  void activate_matlab_export() 
  {
    //dfisx_config.matlab_export = true;
  }
}
