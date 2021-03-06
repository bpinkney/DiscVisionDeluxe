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
    const bool filter_for_deceleration = false;

    throw_container->current_disc_state.forces_state.collision_torque_xyz[0] = 
    throw_container->current_disc_state.forces_state.collision_torque_xyz[1] = 
    throw_container->current_disc_state.forces_state.collision_torque_xyz[2] = 0.0;

    throw_container->current_disc_state.forces_state.collision_force[0] = 
    throw_container->current_disc_state.forces_state.collision_force[1] = 
    throw_container->current_disc_state.forces_state.collision_force[2] = 0.0;

    // we need to do some funky stuff here to ensure that the precession continues throughout subsequent
    // DfisX frames
    // So we'll use the 'dt' from the collision vs. the 'dt' for DfisX to compute how many DfisX loops
    // we need to apply that precession for
    // We DON'T want to override the states every time, so we add an exeption for "consumed_input == 0"
    int DfisX_frames_to_continue_collision = (int)round(throw_container->collision_input.delta_time_s / MAX(dt, CLOSE_TO_ZERO));
    // arbitrary
    BOUND_VARIABLE(DfisX_frames_to_continue_collision, 0, 10);

    const float collision_apply_frames = DfisX_frames_to_continue_collision;
    if(throw_container->collision_input.consumed_input < collision_apply_frames && throw_container->collision_input.delta_time_s > CLOSE_TO_ZERO)
    {
      const bool overwrite_states_ignore_forces_torques = true;
      if(overwrite_states_ignore_forces_torques)
      {

        const bool first_frame = throw_container->collision_input.consumed_input == 0;

        // Do we need to override positions? Even if we take the 'overwrite' states approach, I'm not too sure
        // Seems like the positions don't even get sent to unreal for the moment, and we just assume a constant velocity each dt
        // TODO: look into whether this assumption is hurting us later!
        if(first_frame)
        {
          throw_container->current_disc_state.disc_location = throw_container->collision_input.lin_pos_m;
          throw_container->current_disc_state.disc_velocity = throw_container->collision_input.lin_vel_mps;
        }

        // corect dt for impulse based torques (it was pre-divided with the incorrect one during the collision stuff)
        //throw_container->collision_input.ang_torque_from_impulses_Nm *= throw_container->collision_input.delta_time_s;
        //throw_container->collision_input.ang_torque_from_impulses_Nm /= dt;

        // Borrow the 'paddle boat' angular form drag model from Daero to evaluate our incoming 'delta ang vel'
        // Since Unreal doesn't know how to propagate things correctly, we are using their 'idea' of how much our ang vel should change
        // and comparing that with the resistance torques they would have encountered to compute the true applied torque
        // Ideally, we can then apply this as a torque (instead of overwriting the states), but we shall see.

        // Use DfisX local dt for this derivative since it is the only one we can trust (see Mike for details)
        // TODO: It would be nice to compute these in the torque space, but we don't actually know what
        // Unreal is doing wrt inertias, so subtracting things in the angular space is likely best for now
        float roll_X_collision_ang_accel  = throw_container->collision_input.ang_vel_delta_radps[0] / throw_container->collision_input.delta_time_s;
        float pitch_Y_collision_ang_accel = throw_container->collision_input.ang_vel_delta_radps[1] / throw_container->collision_input.delta_time_s;
        //const float yaw_Z_collision_ang_torque   = throw_container->collision_input.ang_vel_delta_radps[2] / dt;

        // Should we bound the incoming ang accel?
        const float max_accel_rollpitch = 50.0; // rad/s^2
        //BOUND_VARIABLE(roll_X_collision_ang_accel,  -max_accel_rollpitch, max_accel_rollpitch);
        //BOUND_VARIABLE(pitch_Y_collision_ang_accel, -max_accel_rollpitch, max_accel_rollpitch);

        const double Ix = 1.0/4.0 * throw_container->disc_object.mass * (throw_container->disc_object.radius*throw_container->disc_object.radius);
        const double Iy = 1.0/4.0 * throw_container->disc_object.mass * (throw_container->disc_object.radius*throw_container->disc_object.radius);
        const double Iz = 1.0/2.0 * throw_container->disc_object.mass * (throw_container->disc_object.radius*throw_container->disc_object.radius);

        // just check for roll and pitch for now (only parasitic skin drag for spin anyway)
        // TODO: Remove repeat defs
        const float r5 = 
          (
            throw_container->disc_object.radius *
            throw_container->disc_object.radius *
            throw_container->disc_object.radius *
            throw_container->disc_object.radius *
            throw_container->disc_object.radius
          );
        const float xy_rot_form_drag_limit = 0.13412;
        const float Cd_plate = 1.17;

        // We'll compute the delta torque here based on the proposed ang vel delta from unreal
        const float roll_reaction_dtorque_from_Unreal_propagated_ang_vel = 
          -signum(throw_container->collision_input.ang_vel_delta_radps[0]) *
          2.0 * xy_rot_form_drag_limit *
          Cd_plate *
          r5 *
          throw_container->disc_environment.air_density *
          (throw_container->collision_input.ang_vel_delta_radps[0] * throw_container->collision_input.ang_vel_delta_radps[0]);

        const float pitch_reaction_dtorque_from_Unreal_propagated_ang_vel = 
          -signum(throw_container->collision_input.ang_vel_delta_radps[1]) *
          2.0 * xy_rot_form_drag_limit *
          Cd_plate *
          r5 *
          throw_container->disc_environment.air_density *
          (throw_container->collision_input.ang_vel_delta_radps[1] * throw_container->collision_input.ang_vel_delta_radps[1]);

       
        // get the applied accel from the reaction torque
        // should this be x^3/3 * dt?
        const float roll_reaction_accel  = roll_reaction_dtorque_from_Unreal_propagated_ang_vel / Ix;
        const float pitch_reaction_accel = pitch_reaction_dtorque_from_Unreal_propagated_ang_vel / Iy;

        // If the resistance torque is more than the collision applied, don't bother
        float roll_accel_corrected  = 0.0;
        //if(fabs(roll_reaction_accel) < fabs(roll_X_collision_ang_accel))
        //{
          roll_accel_corrected = roll_X_collision_ang_accel + roll_reaction_accel;
        //}

        float pitch_accel_corrected = 0.0;
        //if(fabs(pitch_reaction_accel) < fabs(pitch_Y_collision_ang_accel))
        //{
          pitch_accel_corrected = pitch_Y_collision_ang_accel + pitch_reaction_accel;
        //}

        ////////// also dupe precession for now (we'll make non duped functions later!)
        //if(abs(throw_container->current_disc_state.disc_rotation_vel) > 0.1)
        
        // make this threshold triple DISABLE_COMPLEX_DISC_COLLISION_MIN_SPIN_RADPS
        // with the assumption that it should stop precessing before control transitions to unreal
        if(abs(throw_container->collision_input.ang_vel_radps[2]) > 3.0)
        {
          const float new_pitch_moment = (pitch_accel_corrected * Iy);
          const float new_roll_moment =  (roll_accel_corrected * Ix);

          //float Wp = -(new_roll_moment)  / (Iz * throw_container->current_disc_state.disc_rotation_vel);
         // float Wq =  (new_pitch_moment) / (Iz * throw_container->current_disc_state.disc_rotation_vel);

          float Wp = -(new_roll_moment)  / (Iz * throw_container->collision_input.ang_vel_radps[2]);
          float Wq =  (new_pitch_moment) / (Iz * throw_container->collision_input.ang_vel_radps[2]);
          // compute resulting angular torque from applied pitching moment vel
          //float Wq_d  = (Wq - throw_container->current_disc_state.disc_rolling_vel)  / MAX(throw_container->collision_input.delta_time_s, CLOSE_TO_ZERO);
          //float Wp_d  = (Wp - throw_container->current_disc_state.disc_pitching_vel) / MAX(throw_container->collision_input.delta_time_s, CLOSE_TO_ZERO);
          float Wq_d = 0.0;
          float Wp_d = 0.0;
          if(throw_container->collision_input.delta_time_s > CLOSE_TO_ZERO)
          {
            Wq_d = (Wq - throw_container->collision_input.ang_vel_radps[0])  / throw_container->collision_input.delta_time_s;
            Wp_d = (Wp - throw_container->collision_input.ang_vel_radps[1])  / throw_container->collision_input.delta_time_s;
          }
          
          float gyro_torque_y = Wp_d * Iy;
          float gyro_torque_x = Wq_d * Ix;

          roll_accel_corrected  += (gyro_torque_x / Ix);
          pitch_accel_corrected += (gyro_torque_y / Iy);
        }
        ////////// end duped precession

        const float roll_collision_torque_Nm  = roll_accel_corrected  * Ix;
        const float pitch_collision_torque_Nm = pitch_accel_corrected * Ix;

        // apply torques directly!
        /*if(!first_frame)
        {
          throw_container->current_disc_state.forces_state.collision_torque_xyz[0] = roll_collision_torque_Nm;
          throw_container->current_disc_state.forces_state.collision_torque_xyz[1] = pitch_collision_torque_Nm;
        }*/

        // compute delta vel instead and tack it on!
        const float roll_vel_delta  = roll_accel_corrected  * throw_container->collision_input.delta_time_s;
        const float pitch_vel_delta = pitch_accel_corrected * throw_container->collision_input.delta_time_s;

        // Only the reaction torque and gyro precession from the unreal input torque
        // can add this to the ang vel from unreal theoretically to get a better approximation
        const float roll_vel_delta_minus_unreal  = (roll_accel_corrected  - roll_X_collision_ang_accel)  * throw_container->collision_input.delta_time_s;
        const float pitch_vel_delta_minus_unreal = (pitch_accel_corrected - pitch_Y_collision_ang_accel) * throw_container->collision_input.delta_time_s;

        //throw_container->current_disc_state.disc_rolling_vel  += roll_vel_delta;
        //throw_container->current_disc_state.disc_pitching_vel += pitch_vel_delta;

        // Update disc normal to the nonsense that unreal gives us
        // We'll tack on the propagation from precession and paddle aero effects below?
        if(first_frame)
        {
          throw_container->current_disc_state.disc_orient_z_vect[0]  = throw_container->collision_input.disc_rotation[0];
          throw_container->current_disc_state.disc_orient_z_vect[1]  = throw_container->collision_input.disc_rotation[1];
          throw_container->current_disc_state.disc_orient_z_vect[2]  = throw_container->collision_input.disc_rotation[2];
        }

        // Copied from Dpropagate
        // only for the vel component applied during the collision (we don't want to dupe the normal propagation)
        // Determine the change to the Z unit vector due to the roll and pitch changes (rotate in that order!)
        // Should we be updating the X and Y unit vectors here as well?? They are not orthonormal after this if not
        const float roll_propagation_rad  = 
          roll_vel_delta  * throw_container->collision_input.delta_time_s;
        const float pitch_propagation_rad = 
          pitch_vel_delta * throw_container->collision_input.delta_time_s;

        // compute the change in rotation we need to apply to the Z unit vector by using a rotation matrix
        // Note this is column major, so it is really the transpose of what you see here.
        Eigen::Matrix3d Rx; 
        Rx <<   1, 0, 0,
                0, cos(roll_propagation_rad), -sin(roll_propagation_rad),
                0, sin(roll_propagation_rad),  cos(roll_propagation_rad);
        // per the note above, we need to transpose this to get column major (which makes the most sense with how this is written out! bah Eigen!)
         Rx = Rx.transpose();
            // Note this is column major, so it is really the transpose of what you see here.
        Eigen::Matrix3d Ry; 
        Ry <<   cos(pitch_propagation_rad), 0, sin(pitch_propagation_rad),
                0, 1, 0,
                -sin(pitch_propagation_rad), 0, cos(pitch_propagation_rad);
        // per the note above, we need to transpose this to get column major (which makes the most sense with how this is written out! bah Eigen!)
        Ry = Ry.transpose();

        // formulate the rotation matrix to bring this back to the world frame next
        // Rememnber this is column-major (ok, so I just wrote this one backward)
        Eigen::Matrix3d Rdw; 
        Rdw << throw_container->current_disc_state.disc_orient_x_vect[0], throw_container->current_disc_state.disc_orient_y_vect[0], throw_container->current_disc_state.disc_orient_z_vect[0],
               throw_container->current_disc_state.disc_orient_x_vect[1], throw_container->current_disc_state.disc_orient_y_vect[1], throw_container->current_disc_state.disc_orient_z_vect[1],
               throw_container->current_disc_state.disc_orient_x_vect[2], throw_container->current_disc_state.disc_orient_y_vect[2], throw_container->current_disc_state.disc_orient_z_vect[2];

        // it is perhaps simpler to just compute the z unit vector change for the two axes?
        // this almost works around the roll -> pitch sequential rotation losses?
        Eigen::Vector3d base_z = {0,0,1};

        Eigen::Vector3d z_unit_delta = {0,0,0};
        z_unit_delta += (Rx * base_z - base_z);
        z_unit_delta += (Ry * base_z - base_z);

        // NOT USING THIS YET!
        z_unit_delta = Rdw * z_unit_delta;
        // Update the vector
        // doing an update here seems to cause a lot of clipping.... so I guess we can leave it up to the sim dt propagation in Dpropagate
        // and simply update the ang velocities here? hmm
/*        throw_container->current_disc_state.disc_orient_z_vect += z_unit_delta;

        // Now let's re-compute the unit X and Y vectors (like we do in Daero) to ensure that they are still
        // orthonormal to the disx Z normal
        // re-normalize just in case
        d_state.disc_orient_z_vect /= d_state.disc_orient_z_vect.norm();
        d_state.disc_orient_y_vect = d_forces.disc_velocity_unit_vector.cross(d_state.disc_orient_z_vect);
        d_state.disc_orient_y_vect /= d_state.disc_orient_y_vect.norm();
        d_state.disc_orient_x_vect = d_state.disc_orient_y_vect.cross(d_state.disc_orient_z_vect);
        d_state.disc_orient_x_vect /= d_state.disc_orient_x_vect.norm();*/
          
        // finally, we bound the acceleration in case it is nonsense as a precaution
        
        //const float max_accel_rollpitch = 50.0; // rad/s^2
        //BOUND_VARIABLE(roll_accel_corrected,  -max_accel_rollpitch, max_accel_rollpitch);
        //BOUND_VARIABLE(pitch_accel_corrected, -max_accel_rollpitch, max_accel_rollpitch);

        //throw_container->current_disc_state.forces_state.collision_torque_xyz[0] = roll_accel_corrected  * Ix;
        //throw_container->current_disc_state.forces_state.collision_torque_xyz[1] = pitch_accel_corrected * Iy;

        // compute new ang vels, and overwrite, leave it up to Dpropagate to update the orientations
        //throw_container->current_disc_state.disc_rolling_vel  += roll_accel_corrected  * throw_container->collision_input.delta_time_s;
        //throw_container->current_disc_state.disc_pitching_vel += pitch_accel_corrected * throw_container->collision_input.delta_time_s;

        //throw_container->current_disc_state.forces_state.collision_torque_xyz[0] = roll_accel_corrected  * Ix;
        //throw_container->current_disc_state.forces_state.collision_torque_xyz[1] = pitch_accel_corrected * Iy;

        // limit ang torque applied
        //BOUND_VARIABLE(throw_container->collision_input.ang_torque_from_impulses_Nm[0],  -max_accel_rollpitch*Ix, max_accel_rollpitch*Ix);
        //BOUND_VARIABLE(throw_container->collision_input.ang_torque_from_impulses_Nm[1],  -max_accel_rollpitch*Iy, max_accel_rollpitch*Iy);

        //throw_container->current_disc_state.forces_state.collision_torque_xyz[0] = throw_container->collision_input.ang_torque_from_impulses_Nm[0];
        //throw_container->current_disc_state.forces_state.collision_torque_xyz[1] = throw_container->collision_input.ang_torque_from_impulses_Nm[1];
        //throw_container->current_disc_state.forces_state.collision_torque_xyz[2] = throw_container->collision_input.ang_torque_from_impulses_Nm[2];


        //BOUND_VARIABLE(throw_container->collision_input.ang_vel_radps[0], -2, 2);
        //throw_container->current_disc_state.forces_state.collision_torque_xyz[0] = throw_container->collision_input.ang_torque_from_delta_vel_Nm[0];
        //throw_container->current_disc_state.forces_state.collision_torque_xyz[1] = throw_container->collision_input.ang_torque_from_delta_vel_Nm[1];
        //throw_container->current_disc_state.forces_state.collision_torque_xyz[2] = throw_container->collision_input.ang_torque_from_delta_vel_Nm[2];

        if(first_frame)
        {
          throw_container->current_disc_state.disc_rolling_vel  = throw_container->collision_input.ang_vel_radps[0];
          throw_container->current_disc_state.disc_pitching_vel = throw_container->collision_input.ang_vel_radps[1];
          throw_container->current_disc_state.disc_rotation_vel = throw_container->collision_input.ang_vel_radps[2];
        }

        throw_container->current_disc_state.disc_rolling_vel  += roll_vel_delta_minus_unreal;
        throw_container->current_disc_state.disc_pitching_vel += pitch_vel_delta_minus_unreal;
        //throw_container->current_disc_state.disc_rotation_vel;
      }
      else
      {

        //throw_container->current_disc_state.disc_location = throw_container->collision_input.lin_pos_m;
        //throw_container->current_disc_state.disc_velocity = throw_container->collision_input.lin_vel_mps;

        //throw_container->current_disc_state.disc_orient_z_vect[0]  = throw_container->collision_input.disc_rotation[0];
        //throw_container->current_disc_state.disc_orient_z_vect[1]  = throw_container->collision_input.disc_rotation[1];
        //throw_container->current_disc_state.disc_orient_z_vect[2]  = throw_container->collision_input.disc_rotation[2];

        // Better idea since it prevents propagation from JUST collisions, however, harder to sync timing wise
        throw_container->current_disc_state.forces_state.collision_force[0]      = throw_container->collision_input.lin_force_from_delta_vel_N[0];
        throw_container->current_disc_state.forces_state.collision_force[1]      = throw_container->collision_input.lin_force_from_delta_vel_N[1];
        throw_container->current_disc_state.forces_state.collision_force[2]      = throw_container->collision_input.lin_force_from_delta_vel_N[2];

        throw_container->current_disc_state.forces_state.collision_torque_xyz[0] = throw_container->collision_input.ang_torque_from_delta_vel_Nm[0];
        throw_container->current_disc_state.forces_state.collision_torque_xyz[1] = throw_container->collision_input.ang_torque_from_delta_vel_Nm[1];
        throw_container->current_disc_state.forces_state.collision_torque_xyz[2] = throw_container->collision_input.ang_torque_from_delta_vel_Nm[2];
      }

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
  static bool gyromode = false;
  void step_simulation(Throw_Container *throw_container, const float dt)
  {
    // consume incoming collisions (do this first in case we need to override states)
    consume_Dcollision(throw_container, dt);
    // determine Aero forces
    step_Daero(throw_container, dt);
    // step_Dcollision (throw_container, dt);
    step_Dgyro(throw_container, dt, gyromode);

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
    const Eigen::Vector3d thrown_disc_orientation, 
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


    const double thrown_disc_rotation = 0;

    //create the starting d state
    throw_container->current_disc_state.disc_location = thrown_disc_position;
    throw_container->current_disc_state.disc_velocity = thrown_disc_velocity;
    throw_container->current_disc_state.disc_orient_z_vect = thrown_disc_orientation;
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

    static uint32_t disc2throw = 1;
    if(1)
    {
      // generate a random disc mold index from all possible sets
      if(1)
      {
        const float index = (float)rand() / (float)RAND_MAX;
        disc_mold = floor(index * disc_object_array.size());
      }

      // OR just override explicitly
      //disc_mold = find_disc_mold_index_by_name("Mako3");

      // OR just proceed through all of them in order
      // (skipping the brick)
      if(0)
      {
        disc_mold++;
        if(disc_mold > disc_object_array.size() - 1)
        {
          disc_mold = 1;
        }
      }

      // OR throw a cycling set for test purposes
      // REALLY handy for tuning, try throwing a:
      // - straight midrange or putter with not much camber (e.g. Mako3) 
      // - understable or stable driver with camber and a thick rim (e.g. destroyer)
      // - stable fairway driver with a thick lower rim camber (e.g. TeeBird or Wraith)
      // If you can get all these flying OK, you've got a decent tuning!

      //disc_mold = find_disc_mold_index_by_name("Firebird");
      gyromode =  !gyromode;

      if(0)
      {
        switch(disc2throw)
        {
          case 1:
            disc_mold = find_disc_mold_index_by_name("Roadrunner");
            //disc2throw = 2;
            break;
          case 2:
            disc_mold = find_disc_mold_index_by_name("Northman");
            //disc2throw = 1;// just do the first 2
            break;
          case 3:
            disc_mold = find_disc_mold_index_by_name("Shryke");
            //disc2throw = 4;
            break;
          case 4:
            disc_mold = find_disc_mold_index_by_name("Swan");
            disc2throw = 5;
            break;
          case 5:
            disc_mold = find_disc_mold_index_by_name("Shryke");
            disc2throw = 6;
            break;
          case 6:
            disc_mold = find_disc_mold_index_by_name("Valkyrie");
            disc2throw = 7;
            break;
          default:
            disc_mold = find_disc_mold_index_by_name("Wraith");
            disc2throw = 1;
            break;
        }
      }    
    }

    // override for Loft Test Throws
    if(0)
    {
      //static int test_throw = 0;
      throw_container->current_disc_state.disc_location[1] = 0.0;
      throw_container->current_disc_state.disc_location[0] = 1.0;

      static int throw_set = 0;
      Eigen::Vector3d hps = {0,0,0};
      switch(throw_set)
      {
        default:
        case 0:
          // regular flat throw
          throw_container->current_disc_state.disc_velocity = {60.0/3.6, 0, 0};
          throw_container->current_disc_state.disc_rotation_vel = 70.0;
          hps = {DEG_TO_RAD(-90.0), DEG_TO_RAD(0.0), DEG_TO_RAD(0)};
          //throw_set = 1;
          break;
        case 1: 
          {       
          // roadrunner roller!
          const double heading = DEG_TO_RAD(-20.0);
          const double speed = 80.0/3.6;
          const double east = -speed * sin(heading);
          const double north = speed * cos(heading);
          // not sure why these north and east assignments don't work here
          throw_container->current_disc_state.disc_velocity = {80.0/3.6, 0, 0};
          throw_container->current_disc_state.disc_rotation_vel = -50.0;
          hps = {DEG_TO_RAD(50), DEG_TO_RAD(20), DEG_TO_RAD(0)};
          throw_set = 2;
          }
          break;
        case 2:
          {
          // SKIP TIME BROSEPH
          const double heading = DEG_TO_RAD(10);
          const double speed = 80.0/3.6;
          const double east = -speed * sin(heading);
          const double north = speed * cos(heading);
          throw_container->current_disc_state.disc_velocity = {80.0/3.6, 0, 0};
          throw_container->current_disc_state.disc_rotation_vel = -75.0;
          hps = {DEG_TO_RAD(-5), DEG_TO_RAD(-15), DEG_TO_RAD(0)};
          throw_set = 0;
          }
          break;
      }

      const double new_x_component = sin (-hps[1]) * cos (hps[0]);
      const double new_y_component = sin (hps[0])  * cos (hps[1]);
      const double new_z_component = cos (hps[1])  * cos (hps[0]);

      Eigen::Vector3d new_thrown_disc_orientation = {new_x_component, new_y_component, new_z_component};

      throw_container->current_disc_state.disc_orient_z_vect = new_thrown_disc_orientation;
    }

    throw_container->disc_object = disc_object_array[disc_mold];

    std::cout << std::endl << std::endl << "Throwing a " << throw_container->disc_object.manufacturer << " " << throw_container->disc_object.mold_name << "(" << disc_object_array.size() << " available disc molds)" << std::endl;

    throw_container->disc_state_array.clear();


/////////// used to populate disc states
    step_simulation(throw_container, SIM_DT_S);
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
