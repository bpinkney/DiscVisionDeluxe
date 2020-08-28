#pragma once
#include <Eigen/Dense>
#include <vector>

#define d_statistics    active_throw.disc_statistics
#define d_velocity      active_throw.current_disc_state.disc_velocity
#define d_location      active_throw.current_disc_state.disc_location
#define d_orientation   active_throw.current_disc_state.disc_orientation
#define d_state         active_throw.current_disc_state
#define d_array         active_throw.disc_state_array
#define p_state         active_throw.previous_disc_state
#define d_forces        active_throw.current_disc_state.forces_state
#define d_object        active_throw.disc_object

#define RAD_360   6.28318531
#define RAD_180   3.14159265
#define RAD_90    1.57079633


#define RHO       1.225
#define PI_X_AR   3.99


#define AVIAR     DANGLE_SS




namespace DfisX
{

const bool basic_console_logging = true;
const bool verbose_console_logging = false;
const double step_time_global = 0.01;

enum Disc_Mold_Enum
{
  DANGLE_SS,
  DANGLE,
  DANGLE_OS
  
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

void test(Disc_Mold_Enum disc_mold_enum,Eigen::Vector3d thrown_disc_position,Eigen::Vector3d thrown_disc_velocity,double thrown_disc_roll,double thrown_disc_pitch,double thrown_disc_radians_per_second,double thrown_disc_wobble);
void simulate_throw();


struct Forces_State
  //simulation step variables
  //not needed for display
  {
      Eigen::Vector3d current_wind;    //in meters per second as a vector

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
      ////double step_time ???needed????                          add me



      /////////////aero holders

      Eigen::Vector3d disc_x_unit_vector;
      Eigen::Vector3d disc_y_unit_vector;
      Eigen::Vector3d disc_lift_unit_vector;
      Eigen::Vector3d disc_velocity_unit_vector;
      Eigen::Vector3d lift_force_vector;
      Eigen::Vector3d drag_force_vector;

      double aoar;
      double velocity_magnitude;
      double v2;
      double pav2by2;

      double coefficient_curve;
      double stall_curve;

      double lift_force_magnitude;
      double drag_force_magnitude;

      double realized_lift_coefficient;
      double realized_drag_coefficient;
      double induced_drag_coefficient;
      double realized_pitching_moment_coefficient;
      double stall_induced_drag;

      double lift_induced_pitching_moment;  //needs to go into struct




  };

///////////////x north
///////////////y east
struct Disc_State
//the location, orientation, and velocity of a disc
//everything needed for display purposes
{
  Eigen::Vector3d disc_location;    //in meters
  Eigen::Vector3d disc_velocity;    //in meters per second
  Eigen::Vector3d disc_orientation;  //orientation vector is the normal of the plane inscribed by the disc
  double disc_rotation;      //in radians
  Sim_State sim_state; 
  //describes the current state of the disc

  Forces_State forces_state;
  
};




struct Global_Variables

{
  std::string save_path;
  Eigen::Vector3d global_wind_vel;    //in meters per second as a vector
  bool matlab_export;
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

//const Disc_Object disc_aviar {"Aviar",0.18,0.5,0.015,-0.012,  0.12,0.175,0.210};
//const Disc_Object disc_aviar {"Aviar",0.15,0.44,0.055,-0.0010,  0.010,0.175,0.210};

struct Disc_Statistics

{
Eigen::Vector3d disc_start_location;
Eigen::Vector3d disc_finish_location;
Eigen::Vector3d disc_landed_location;
double disc_cumulative_roll;

};

struct Throw_Container
//contains all of the info needed to simulate and display a disc
{

  Disc_State current_disc_state;
  Disc_State previous_disc_state;
  Disc_Object disc_object;
  Disc_Statistics disc_statistics;
  std::vector <Disc_State> disc_state_array;
};




void               step_simulation (Throw_Container &throw_container, float step_time);

void               new_throw (Disc_Mold_Enum disc_mold_enum,Eigen::Vector3d thrown_disc_position,Eigen::Vector3d thrown_disc_velocity, double thrown_disc_roll, double thrown_disc_pitch, double thrown_disc_radians_per_second, double thrown_disc_wobble);

void               finish_throw        (Throw_Container &throw_container);

void               load_disc_parameters();
void               init                ();
void               set_save_path       (std::string save_path);
void               set_global_wind_vel (Eigen::Vector3d global_wind_vel);
Eigen::Vector3d    get_global_wind_vel ();
void               activate_matlab_export ();




}