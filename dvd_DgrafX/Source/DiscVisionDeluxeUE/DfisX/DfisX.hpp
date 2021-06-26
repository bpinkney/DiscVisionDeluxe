#pragma once
#include <Eigen/Dense>
#include <vector>

#include "disc_layouts.hpp"

#define d_statistics    throw_container->disc_statistics
#define d_velocity      throw_container->current_disc_state.disc_velocity
#define d_location      throw_container->current_disc_state.disc_location
#define d_orientation   throw_container->current_disc_state.disc_orient_z_vect
#define d_state         throw_container->current_disc_state
#define d_array         throw_container->disc_state_array
#define p_state         throw_container->previous_disc_state
#define d_forces        throw_container->current_disc_state.forces_state
#define d_object        throw_container->disc_object

//disc aerodynamic constant
#define PI_X_AR  (3.99)

#define SIM_DT_S (0.001)

namespace DfisX
{
  //SIMULATION STATE
  //this enum roughly describes the state of flight a disc is in
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


  // environmental enums
  enum class Gust_Factor
  {
    ZERO_DEAD_DIDDLY,
    ONE_DULL_DRAFT,
    TWO_CALM_CHINOOK,
    THREE_BRUSQUE_BREEZE,    
    FOUR_ROBUST_GUST,
    FIVE_ZEALOUS_ZEPHYR,
    SIX_GALLED_GALE,
    SEVEN_FURIOUS_FLURRY,
    EIGHT_TERRIBLE_TEMPEST,
    NINE_PSYCHOTIC_CYCLONE,
    TEN_HOMICIDAL_HURRICANE
  };


  struct Collision_Input
  {
    Eigen::Vector3d lin_pos_m;                      // world frame
    Eigen::Vector3d lin_vel_mps;                    // world frame
    Eigen::Vector3d lin_force_from_impulses_N;      // world frame
    Eigen::Vector3d lin_force_from_delta_vel_N;     // world frame
    Eigen::Vector3d disc_rotation;                  // Disc Z normal unit vector (same as state)
    Eigen::Vector3d ang_vel_radps;                  // about local disc axes unit axes (DfisX defines)
    Eigen::Vector3d ang_vel_delta_radps;            // about local disc axes unit axes (DfisX defines)   
    Eigen::Vector3d ang_torque_from_delta_vel_Nm;   // about local disc axes unit axes (DfisX defines)
    Eigen::Vector3d ang_vel_from_impulses_Nm;       // about local disc axes unit axes (DfisX defines)
    Eigen::Vector3d ang_torque_from_impulses_Nm;    // about local disc axes unit axes (DfisX defines)
    
    double delta_time_s;
    uint8_t consumed_input; // Flag to mark whether or not this force has been consumed by the state propagator
  };

  struct Friction_Input
  {
    Eigen::Vector3d lin_force_XYZ;                  // world frame friction force from fluid model
    Eigen::Vector3d ang_torque_XYZ;                 // disc body frame
  };

  // Forces State
  //Aerodynamic simulation step variables
  //  used to calculate the forces/accelerations on a disc in flight
  struct Forces_State
  {
    Eigen::Vector3d gyro_orientation_delta; // orientation change produced by gyroscopic precession

    Eigen::Vector3d net_force;
    //Eigen::Vector3d net_torque;
    double net_torque_x;
    double net_torque_y;
    double net_torque_z;

    Eigen::Vector3d aero_force;
    double gyro_torque_x;
    double gyro_torque_y;
    double aero_torque_x;
    double aero_torque_y;
    double aero_torque_z;

    Eigen::Vector3d collision_force;  // linear collision force (N) in the world frame
    Eigen::Vector3d collision_torque_xyz; // angular collision torque about the XYZ body frame axes (hyzer, pitch, spin) (defined wrt lin vel vector)

    int step_count;

    float gust_time_s;
    Eigen::Vector3d gust_vector_xyz;

    /////////////aero holders

    Eigen::Vector3d disc_lift_unit_vector;
    Eigen::Vector3d disc_velocity_unit_vector;
    Eigen::Vector3d lift_force_vector;
    Eigen::Vector3d drag_force_vector;

    double aoar;
    double velocity_magnitude;
    double v2;

    // define moments and forces here
    double lift_induced_pitching_moment; // sum of pitching moments

    double lin_drag_force_plate_N; // force along disc normal from 'plate' surface form drag
    double lin_drag_force_edge_N;  // force along disc plane  from 'edge' surface form drag
    double lin_drag_force_cavity_edge_N; // force along disc plane from form drag against the back cavity edge
    double lin_drag_force_front_rim_camber_N;  // force along rim camber normal from form drag against lower rim, front camber
    double lin_drag_force_back_rim_camber_N;   // force along rim camber normal from form drag against lower rim, back camber
    double lin_drag_force_front_dome_camber_N;  // force along dome normal from form drag against lower rim, front camber
    double lin_drag_force_back_dome_camber_N;   // force along dome normal from form drag against lower rim, back camber
    double lin_drag_force_skin_N;  // force along -ve air vector from parasitic skin drag

    double lift_force_cavity_edge_N; // lift generated along the disc normal by the slow-down of air hitting the back of the cavity. higher pressure below -> lift
    double lift_force_camber_N;      // lift generated along the disc normal by the speed up of air over the 'wing-like' disc camber shape. lower pressure above -> lift
    
    double rot_torque_plate_offset_Nm;       // Torque generated form the off-centre force centre of 'lin_drag_force_plate_N'
    double rot_torque_rim_camber_offset_Nm;  // Torque generated form the off-centre force centre of 'lin_drag_force_rim_camber_N'
    double rot_torque_cavity_edge_offset_Nm; // Torque generated form the off-centre force centre of 'lift_force_cavity_edge_N'
    double rot_torque_camber_offset_Nm;      // Torque generated form the off-centre force centre of 'lift_force_camber_N'

    double rot_drag_torque_x_Nm; // skin drag for Z (spin)
    double rot_drag_torque_y_Nm; // 'paddle' form drag for X (pitch)
    double rot_drag_torque_z_Nm; // 'paddle' form drag for Y (roll)
  };


  struct Disc_Env
  {
    Eigen::Vector3d wind_vector_xyz; // m/s static wind (may be updated by Unreal if we move into ground effects, or behind trees later)
    Gust_Factor gust_factor;         // gust factor enum
    double air_density;              // kg/m^3, ISA sea-level nominal 1.225
  };

  //Describes the state of a disc at one point in time
  //
  //   everything needed for display purposes and then some
  struct Disc_State
  {
    Eigen::Vector3d disc_location;      //in meters
    Eigen::Vector3d disc_velocity;      //in meters per second
    Eigen::Vector3d disc_acceleration;  //in meters per second squared (this is just here for logging for now)
    Eigen::Vector3d disc_orient_z_vect; // projected from the top of the disc normal                  (orthogonal to disc_orient_x_vect and disc_orient_y_vect)
    Eigen::Vector3d disc_orient_x_vect; // projected along the disc plane in the direction of the airspeed vector (orthogonal to disc_orient_z_vect and disc_orient_y_vect)
    Eigen::Vector3d disc_orient_y_vect; // projected along the disc plane to the right of disc_orient_x_vect (orthogonal to disc_orient_z_vect and disc_orient_x_vect)
    double disc_pitching_vel;           //radians_per_second (just for pitching moment), for now we'll try to track this despite the frame changing slightly every time[k]
    double disc_pitching_accel;         //radians_per_second squared (just for pitching moment), for now we'll try to track this despite the frame changing slightly every time[k]
    double disc_rolling_vel;            //radians_per_second (just for rolling moment), for now we'll try to track this despite the frame changing slightly every time[k]
    double disc_rolling_accel;          //radians_per_second squared (just for rolling moment), for now we'll try to track this despite the frame changing slightly every time[k]
    double disc_rotation;               //in radians
    double disc_rotation_vel;           //radians_per_second (just for spin)
    double disc_rotation_accel;         //radians_per_second squared (just for spin)
    Sim_State sim_state;                //describes the current state of the disc (just for spin)
    Forces_State forces_state;          //describes the aerodynamic state of a disc in flight including forces,torques,moments, and coefficients  
  };



  //Holds variables need through all DfisX related files that are set during runtime
  struct DfisX_Config
  {
    std::string save_path;
    Eigen::Vector3d global_wind_vel;    //in meters per second as a vector
    bool matlab_export;                 //used to export saved results to a matlab visualizer
    std::string install_path;
  };



  //Disc Model
  //contains the name and the physics/aerodynamic properties of a disc mold
  struct Disc_Model
  {
    char mold_name[32];     // text for disc name, e.g. Buzzz Big Z
    char manufacturer[32];  // e.g. innova (could do this with an enum later)
    char disc_type[32];     // e.g. putter, midrange, distance driver, etc. (could do this with an enum later)
    char stability[32];     // e.g. stable, overstable, understable, etc. (could do this with an enum later)
    char rim_camber[32];    // e.g. Concave, Convex, Flat (could do this with an enum later)
    float mass;             // (kg), this is just a default can be changed later as part of user config
    float radius;           // (m)
    float rim_width;        // width of rim, from edge of cavity, to edge of disc (m)
    float thickness;        // total thickness of disc, from bottom of rim, to top of camber (m)
    float rim_depth;        // height of cavity, measured at the edge of the cavity (m)
    float rim_camber_height;// height of lower rim camber
    float dome_height;      // height of upper dome camber
  };
  

  //Disc Statistics
  //used to track various stats about a discs flight, such as time aloft and distance travelled
  struct Disc_Statistics
  {
    Eigen::Vector3d disc_start_location;
    Eigen::Vector3d disc_finish_location;
    Eigen::Vector3d disc_landed_location;
    double disc_cumulative_roll;
  };


  // generic debug fields we can change from the UI
  struct Disc_Debug
  {
    double debug0;
    double debug1;
    double debug2;
    double debug3;
    double debug4;
    double debug5;
  };

  //Throw Container
  //contains all of the info needed to simulate and display a disc
  struct Throw_Container
  {
    Disc_State current_disc_state;              // current state of disc
    Disc_State previous_disc_state;             // state of disc last step
    Disc_Model disc_object;                     // aero properties of disc being simulated
    Disc_Statistics disc_statistics;            // used to track stats of a flight
    Disc_Env disc_environment;                  // environmental states and parameters
    Collision_Input collision_input;            // Incoming forces/torques from the Unreal engine collisions
    Friction_Input  friction_input;             // Incoming extra friction as a fucntion of ground proximity model and material reported by Unreal
    std::vector <Disc_State> disc_state_array;  // array of disc states: used to hold the flight data of a simulated throw
    Disc_Debug debug;
  };


  void consume_Dcollision(Throw_Container *throw_container, const float dt);

  bool                is_finished_simulating(Throw_Container *throw_container);

  std::vector <Disc_State> get_disc_state_array(Throw_Container *throw_container);

  //Simulates the current throw to completion
  void                simulate_throw(Throw_Container *throw_container, const float dt);

  //Steps the aero simulation forward one step
  void                step_simulation(Throw_Container *throw_container, const float dt);

  //Instantiates a new throw, ready to be stepped or simulated
  void                new_throw(
    Throw_Container *throw_container, 
    const DiscIndex disc_index, // from dvd_DvisEst
    const Eigen::Vector3d thrown_disc_position,
    const Eigen::Vector3d thrown_disc_velocity, 
    const Eigen::Vector3d thrown_disc_orientation, 
    const double thrown_disc_radians_per_second, 
    const double thrown_disc_wobble);



  //Finishes a throw
  //This includes things such as saving/serializing the throw and performing various stat calculations
  void                finish_throw        (Throw_Container *throw_container, const float dt);
  Disc_State get_disc_state(Throw_Container *throw_container);

  void                activate_matlab_export ();

}