#include "DfisX.hpp"
#include "Daero.hpp"
#include "dvd_maths.hpp"

#include <iostream> 
#include <math.h>

#if defined(WIN32) || defined(_WIN32) || defined(__WIN32) && !defined(__CYGWIN__)

#define ALLOW_AERO_UNREAL_DEBUG
//#include "DebugWidget.h"
#endif


/*
||||||||||||Daero|||||||||||||||||
Handles the aerodynamic forces of disc simulation.
Also gravity.
*/

// Model Constants for all discs
// Form Drag Base coefficients
#define Cm_BASE  (0.08) // base z-axis rotational parasitic skin drag coeff
// https://www.engineeringtoolbox.com/drag-coefficient-d_627.html
#define Cd_PLATE (1.17) // base form drag coeff for 'plate' section of disc
#define Cd_EDGE  (1.10) // base form drag coeff for the 'edge' approximated section of disc
#define Cd_SKIN  (0.01) // base linear parasitic skin drag coeff

// Lift Base coefficients
#define Cl_BASE  (1.0)  // base lift cofficient for all bernoulli lift effects
#define CAVITY_EDGE_LIFT_FACTOR (0.00035302903145605 * 0.4) // lift factor as a fucntion of effective exposed cavity edge area inverse
#define CAVITY_EDGE_NORM_ROT_SPEED (70.0) // rad/s (if this is non-zero, attenuate the cavity lift as a function of spin speed)

// how much deviation we expect from a 'true rectangle' for the edge form drag model
// (equivalent to changing the Cd_EDGE really, but more correct this way)
#define A_EDGE_EFFECTIVE_GAIN (0.25)

// Pitching moment arms as a percentage of total diameter
#define PITCHING_MOMENT_FORM_DRAG_PLATE_OFFSET (0.05) // % of diameter toward the front of the disc for plate drag force centre
#define PITCHING_MOMENT_CAVITY_LIFT_OFFSET     (0.1) // % of diameter toward the back of the disc for cavity lift force centre
#define PITCHING_MOMENT_CAMBER_LIFT_OFFSET     (0.08) // % of diameter toward the front of the disc for camber lift force centre
// disable the lower rim camber model for now (re-evaluate later)
#define RIM_CAMBER_EXPOSURE (0.0) // % of lower rim camber exposed to the airflow vs a rim_width * diameter rectangle

// add some runtime tuning hook-ups
std::string gv_aero_label_debug0  = "CAVITY_EDGE_LIFT_FACTOR";
double gv_aero_debug0             = (CAVITY_EDGE_LIFT_FACTOR);

std::string gv_aero_label_debug1  = "CAVITY_EDGE_NORM_ROT_SPEED";
double gv_aero_debug1             = (CAVITY_EDGE_NORM_ROT_SPEED);

std::string gv_aero_label_debug2  = "A_EDGE_EFFECTIVE_GAIN";
double gv_aero_debug2             = (A_EDGE_EFFECTIVE_GAIN);

std::string gv_aero_label_debug3  = "PITCHING_MOMENT_FORM_DRAG_PLATE_OFFSET";
double gv_aero_debug3             = (PITCHING_MOMENT_FORM_DRAG_PLATE_OFFSET);

std::string gv_aero_label_debug4  = "PITCHING_MOMENT_CAVITY_LIFT_OFFSET";
double gv_aero_debug4             = (PITCHING_MOMENT_CAVITY_LIFT_OFFSET);

std::string gv_aero_label_debug5  = "PITCHING_MOMENT_CAMBER_LIFT_OFFSET";
double gv_aero_debug5             = (PITCHING_MOMENT_CAMBER_LIFT_OFFSET);

std::string gv_aero_label_debug6  = "";
double gv_aero_debug6             = (0.0);

namespace DfisX
{
  ///for display purposes     see Sim_State
  // determines the division between
  // SIM_STATE_FLYING_TURN  -  TURN_CONST   -   SIM_STATE_FLYING   -  FADE_CONST  -  SIM_STATE_FLYING_FADE
  const double HS_TURN_CONST = -0.05;
  const double TURN_CONST =  0.05;
  const double FADE_CONST =  0.15;

  void             make_unit_vector         (Eigen::Vector3d &vector_to_unitize)
  {
      vector_to_unitize /= vector_to_unitize.norm();
  }

  Eigen::Vector3d  get_unit_vector          (Eigen::Vector3d vector_to_unitize)
  {
      return vector_to_unitize /= vector_to_unitize.norm();
  }

  double           angle_between_vectors    (Eigen::Vector3d a, Eigen::Vector3d b) 
  {
      double angle = 0.0;
      angle = std::atan2(a.cross(b).norm(), a.dot(b));
      return angle;
  }


  // GUST TEST
  // Gaussian Noise Generator
  // generate numbers with mean 0 and standard deviation 1. 
  // (To adjust to some other distribution, multiply by the standard deviation and add the mean.)
  // ~0.65s for 10000000 rands on X86
  float gaussrand()
  {
    // not sure if these are a problem for multi-throw, don't think so
    static float V1, V2, S;
    static int phase = 0;
    float X;

    if(phase == 0) {
      do {
        float U1 = (float)rand() / (float)RAND_MAX;
        float U2 = (float)rand() / (float)RAND_MAX;

        V1 = 2 * U1 - 1;
        V2 = 2 * U2 - 1;
        S = V1 * V1 + V2 * V2;
        } while(S >= 1 || S == 0);

      X = V1 * sqrt(-2 * log(S) / S);
    } else
      X = V2 * sqrt(-2 * log(S) / S);

    phase = 1 - phase;

    return X;
  }


  void Daero_compute_gusts(Throw_Container *throw_container)
  {
    // apply static var filters (this will need to change for multidisc)
    // approximate a 1st order butterworth filrter with 0.2Hz cutoff, and 200Hz sampling
    const float N = 1.0 / (0.2/(200/2)) * 0.3;
    // do it with filtered white noise instead
    // we'll do some noise with a standard deviation of 0.3
    // and then bound it to +-1.0
    const double gust_stddev = 0.3;
    const double scaling_factor = N/10.0;

    double raw_gust_noise[3] = 
    {
      gaussrand() * gust_stddev,
      gaussrand() * gust_stddev,
      gaussrand() * gust_stddev * 0.25
    };

    // Bound to +-1.0 absolute
    BOUND_VARIABLE(raw_gust_noise[0], -1.0, 1.0);
    BOUND_VARIABLE(raw_gust_noise[1], -1.0, 1.0);
    BOUND_VARIABLE(raw_gust_noise[2], -1.0, 1.0);

    // Amplify based on gust enum directly for now
    double gust_amplitude = ((double)(throw_container->disc_environment).gust_factor);
    gust_amplitude *= gust_amplitude; // square it

    raw_gust_noise[0] *= gust_amplitude;
    raw_gust_noise[1] *= gust_amplitude;
    raw_gust_noise[2] *= gust_amplitude;

    LP_FILT(d_forces.gust_vector_xyz[0], raw_gust_noise[0], N);
    LP_FILT(d_forces.gust_vector_xyz[1], raw_gust_noise[1], N);
    LP_FILT(d_forces.gust_vector_xyz[2], raw_gust_noise[2], N);
  }

  //main file function
  //this takes a throw container reference and a step time in seconds and performs the aerdynamic force and torque calculations
  //step_daero saves these calculations into the throw container
  void step_Daero(Throw_Container *throw_container, const float dt)
  {
    /* ripped from dfisx.py, naming scheme isnt accurate yet
    #####Unit Vectors
    #vel_unit:  unit vector of total disc velocity
    #lift_unit: unit vector 90 degrees from vel_unit in line with discs 'up' vector
    
    #disc_normal_unit:  unit vector of disc which points 'up' relative to the disc
    #disc_unit_y:  unit vector of velocity vector projected onto discs planes (angle between this and vel_unit is angle of attack)
    #disc_unit_x:  unit vector of disc which points perpendicular to direction of travel but lays on the discs plane
    
    #rotation direction goes from disc_unit_x to disc_unit_y
    """
    */

    // add LP-filtered white-noise gusts
    Daero_compute_gusts(throw_container);

    Eigen::Vector3d disc_air_velocity_vector = d_velocity - throw_container->disc_environment.wind_vector_xyz - d_forces.gust_vector_xyz;

    d_forces.disc_velocity_unit_vector = get_unit_vector(disc_air_velocity_vector);
    make_unit_vector(d_orientation);


  ////////////////////////////////////////////////////////////Div//By//Zero//Protection///////////////////////////////////////////////////////////////
    //division by zero protection......maybe not necessary
    if (!d_orientation.isApprox(d_forces.disc_velocity_unit_vector))
    {
      d_forces.disc_x_unit_vector = d_forces.disc_velocity_unit_vector.cross(d_orientation);
      make_unit_vector (d_forces.disc_x_unit_vector);
      d_forces.disc_y_unit_vector = d_forces.disc_x_unit_vector.cross(d_orientation);
      make_unit_vector (d_forces.disc_y_unit_vector);

      d_forces.disc_lift_unit_vector = d_forces.disc_x_unit_vector.cross (d_forces.disc_velocity_unit_vector);
    }
    ///divide by zero case (disc is travelling perdicularily through the air)
    else
    {
      d_forces.disc_x_unit_vector =    Eigen::Vector3d    (0,0,0);
      d_forces.disc_y_unit_vector =    Eigen::Vector3d    (0,0,0);
      d_forces.disc_lift_unit_vector = Eigen::Vector3d    (0,0,0);
    //if (basic_console_logging) std::cout << "This would produce an error if there was no divide by zero protection in Daero unit vector creation process!!!!!!!!!!!!!!!!!";
    }
  ////////////////////////////////////////////////////////////Div//By//Zero//Protection/////////////////////////////////////////////////////////


    //////////////////////////////////////////Multiuse Variables/////////////////////////////////
    //#AoAr angle of attack (radians)
    //aoar = vel_unit.angle(disc_normal_unit)-np.deg2rad(90)
    d_forces.aoar = angle_between_vectors (d_forces.disc_velocity_unit_vector, d_orientation) - M_PI_2;
    
    //#velocity squared
    //V2 = (vel.magnitude()) ** 2
    d_forces.velocity_magnitude = disc_air_velocity_vector.norm();
    d_forces.v2 =                 d_forces.velocity_magnitude * d_forces.velocity_magnitude;

    //////////////////////////////////////////Multiuse Variables/////////////////////////////////

    /////////////Sim_State calculations for display purposes////////////////////////////////////////
    if      (d_forces.aoar < HS_TURN_CONST)   d_state.sim_state = SIM_STATE_FLYING_HIGH_SPEED_TURN;
    else if (d_forces.aoar < TURN_CONST)      d_state.sim_state = SIM_STATE_FLYING_TURN;
    else if (d_forces.aoar < FADE_CONST)      d_state.sim_state = SIM_STATE_FLYING;
    else                                      d_state.sim_state = SIM_STATE_FLYING_FADE;

    // parasidic drag torque calculations:
    
    // Inertia of a thin disc:
    // Iz =      1/2 * m * r^2
    // Ix = Iy = 1/4 * m * r^2

    // torque = accel * I
    const float r2 = (d_object.radius * d_object.radius);
    const float r5 = (d_object.radius * d_object.radius * d_object.radius * d_object.radius * d_object.radius);
    
    ////// ** ** Start Rotational Drag Model ** ** //////

      // from 'drag of rotating disc pitching'
      // k = 0.13412
      // Td = 2.0 * k * Cwdxy * r^5 * rho * w^2

      //// Form Drag from the disc Rotation about
      //// The forward veloicty vector (-ve disc Y axis)
      //// and the right orthogonal vector (disc X axis)
      //// This is this like a paddle boat paddle (disc rotates and smashes into nearby air)
      //// Uses the same drag coeff as our standard 'plate form drag'

      // 0.13412 is an constant for taking the limit for this drag force
      // see dvd_DfisX_drag_of_rotating_disc_pitching.m
      const double xy_rot_form_drag_limit = 0.13412;
      d_forces.rot_drag_torque_x_Nm = 
        -signum(d_state.disc_pitching_vel) *
        2.0 * xy_rot_form_drag_limit *
        Cd_PLATE *
        r5 *
        throw_container->disc_environment.air_density *
        (d_state.disc_pitching_vel*d_state.disc_pitching_vel);

      d_forces.rot_drag_torque_y_Nm = 
        -signum(d_state.disc_rolling_vel) *
        2.0 * xy_rot_form_drag_limit *
        Cd_PLATE *
        r5 *
        throw_container->disc_environment.air_density *
        (d_state.disc_rolling_vel*d_state.disc_rolling_vel);

      //// Parasitic Skin Drag from the disc Rotation about the normal (disc Z axis)
      //// Think of this like small eddys and friction from the plastic surface of the disc

      // rotational Reynolds number = Re = omega * r^2 / linear_v
      // where (I think) linear_v is along the rotational plane (not sure)
      // for now, we can just take the total lin vel magnitude...

      // (I think?) The rotational reynolds number is a function only of the linear aispeed
      // across the disc surface (orthogonal to the disc normal)
      // So we can compute the dot product between the disc normal unit vector, and the airspeed vector
      // to compute the effective angle between the airspeed vector and the disc normal
      // then, we can use this angle to attenuate the magnitude of the airspeed vector to produce 
      // the magnitude of airflow in the disc plane only
      const float ang_disc_normal_to_airspeed = angle_between_vectors(disc_air_velocity_vector, d_state.disc_orientation);
      const float airspeed_vel_mag_disc_plane = sin(ang_disc_normal_to_airspeed) * d_forces.velocity_magnitude;
      const float Re_rot = (std::fabs(d_state.disc_rotation_vel) * r2) / MAX(airspeed_vel_mag_disc_plane, CLOSE_TO_ZERO);

      // https://www.sciencedirect.com/topics/engineering/rotating-disc
      // approximate surrounded by laminar   Cm = 3.87Re^(-1/2)
      // approximate surrounded by turbulent Cm = 0.146Re^(-1/5)
      // NO IDEA, let's just tune this with the laminar formula
      float Cm = 0.0;
      if(airspeed_vel_mag_disc_plane > CLOSE_TO_ZERO)
      {
        Cm = Cm_BASE * (1.0 / MAX(std::sqrt(Re_rot), CLOSE_TO_ZERO));
      }

      // parasidic drag torque = Tq = 0.5 * rho * omega^2 * r^5 * Cm
      // where omega is the angular vel in m/s
      // and 'r' is the radius in m
      d_forces.rot_drag_torque_z_Nm =
        -signum(d_state.disc_rotation_vel) *
        0.5 * 
        throw_container->disc_environment.air_density * 
        (d_state.disc_rotation_vel * d_state.disc_rotation_vel) * 
        r5 * 
        Cm;
    ////// ** ** End Rotational Drag Model ** ** //////

    ////// ** ** Start Linear Form Drag Model ** ** //////
      // effective edge heighr for our simplified 'edge' and 'plate' model approximation
      // https://www.engineeringtoolbox.com/drag-coefficient-d_627.html
      const double A_plate  = r2 * M_PI;
      const double A_edge   = d_object.radius * 2 * d_object.edge_height * gv_aero_debug2; // approximated as a rectangle
      const double rhov2o2  = throw_container->disc_environment.air_density * d_forces.v2 * 0.5;

      // Disc 'Form' Drag
      // what we need here is the projection of the airspeed induced force against the surfaces of the disc
      // we are considering two surfaces right now:
      // 1. disc edge, this will be minimal on drivers, but not on putters. This effective area should later be a disc param
      // 2. disc plate, this is the flat surface on the top or bottom of the disc. The Cd*A is probably higher on the bottom due to the lips,
      // but we'll consider it symmetric for the moment

      // For the EDGE:
      // The incident force of the air on the edge will be felt along the plane of the disc.
      // e.g. a nose-down disc with a horz airflow will produce lift, and get pushed backward along the air force vector
      // and a nose-up disc with a horz airflow will produce negative lift, and get pushed backward along the air force vector
      // ^                                /\<------ V
      //  \                              / /
      //  \\\                           / /
      //   \ \                         ///   
      //    \ \                        /
      //     \/<----- V               v

      // For the PLATE:
      // The incident force of the air on the plate should always project (positively or negatively) along 
      // the disc nomrla plane. You can think of this like a normal force for an applied force on the ground
      // (it is always orthogonal to the incident surface)

      // The magnitude of these applied forces is a function of how much of this surface is incident with the airflow.
      // So if the disc is completely flat in a horizontal airflow, there is NO PLATE DRAG FORCE
      // and similarly, a flat disc in vertical free-fall would have NO EDGE DRAG FORCE
      // this is only considering form drag, and NOT parasitic surface drag

      // Fd_edge
      d_forces.lin_drag_force_edge_N  = rhov2o2 * Cd_EDGE  * A_edge  * abs(cos(d_forces.aoar));
      // Fd_plate
      d_forces.lin_drag_force_plate_N = rhov2o2 * Cd_PLATE * A_plate * sin(d_forces.aoar);

      // Parasitic Skin Drag
      // This is not included in the two components of form drag listed above, but will change with AOA
      // It is akin to 'air friction', and is addressed for the rotational case in 'aero_torque_z' above
      // For the linear airflow interaction, 
      // we assume that maximum skin drag occurs when the most surface area is exposed to an airflow
      // this is very probably when the disc is flat
      // The effective area used for this drag term may be related to the boundary layer, and how
      // long laminar airflows stay attached to the disc.
      // For now, we'll just approximate it as a fixed valuefor any AOA (to be revisited later if required)
      // Skin drag is considered to be in the direction of the airflow vector
      // Fd_skin
      d_forces.lin_drag_force_skin_N = rhov2o2 * Cd_SKIN * (A_edge + A_plate);

      // now we can determine the unit vector to apply the edge drag force in
      // get orth vector to airspeed and disc norm
      // then use this vector to get the projected edge force vector along the disc plane
      Eigen::Vector3d edge_force_vector = d_orientation.cross(d_orientation.cross(d_forces.disc_velocity_unit_vector));
      make_unit_vector(edge_force_vector);

      // from the model validation and comparison with wind tunnel data in
      // "dvd_DfisX_form_drag_and_stall_drag_comparison.m"
      // 1. Effective drag in increase from air hitting the back of the disc inner lip
      // this was shown to be aroun 0.5 by comparison with the wind tunnel models
      const double lip_exposed_surface_factor = 0.5;
      
      // get effective area exposed by inner lip
      const double A_eff_lip = 
        (d_object.radius * 2 - d_object.rim_width * 2) *
        d_object.rim_depth * lip_exposed_surface_factor;

      const double A_eff_lip_at_aoa = A_eff_lip * cos(d_forces.aoar);

      // Fd_lip
      // along edge_force_vector
      // TODO: make this better! effective range is [-15deg, 70deg] AOA, peak at 20 AOA
      // attenuated with AOA as a sinusoid
      d_forces.lin_drag_force_cavity_edge_N = 
        rhov2o2 * Cd_EDGE * A_eff_lip_at_aoa * cos(d_forces.aoar - DEG_TO_RAD(20)) * 
        (d_forces.aoar <= DEG_TO_RAD(70) && d_forces.aoar >= DEG_TO_RAD(-15) ? 1.0 : 0.0);

    ////// ** ** End Linear Form Drag Model ** ** //////

    ////// ** ** Start Linear Lift Model ** ** //////

      // from the model validation and comparison with wind tunnel data in
      // "dvd_DfisX_form_drag_and_stall_drag_comparison.m"
      // 2. Lift from decreased aispeed below the disc due to the inner lip 'edge' form drag noted above
      // This is a Bernoulli lift effect, since the slowed air below the disc
      // results in an increase in pressure below, resulting in lift
      // define the range for Bernoulli effects from the inner lip
      double lift_factor = 0;
      if(A_eff_lip > 0)
      {
        lift_factor = (1.0 / A_eff_lip) * gv_aero_debug0;//CAVITY_EDGE_LIFT_FACTOR;
        if(gv_aero_debug1 > 0)
        {
          lift_factor *= abs(d_state.disc_rotation_vel) / gv_aero_debug1;
        }
      }
      
      // same angular range as above!
      d_forces.lift_force_cavity_edge_N = 
        rhov2o2 * Cl_BASE * A_plate * lift_factor * cos(d_forces.aoar - DEG_TO_RAD(20)) * 
        (d_forces.aoar <= DEG_TO_RAD(70) && d_forces.aoar >= DEG_TO_RAD(-15) ? 1.0 : 0.0);

      // Only attenuate this lift for nose-down, i.e. negative AOAs
      // TODO: Is this right? who knows 
      
      // 3. Lift from increased top arc-length, and corresponding increasing in 'above disc' airspeed
      // This is the classic 'wing' Bernoulli effect, where the extra distance covered by the airstream
      // above the disc, results in a higher airspeed, and a lower pressure than below, resulting in lift
      // height above edge for camber dome peak

      // intuit camber height from the total thickness, and edge height delta
      const double camber_height = d_object.thickness - d_object.edge_height;

      // treat this like a triangle approx
      const double camber_rect_arc_length = sqrt(d_object.radius * d_object.radius + camber_height * camber_height) * 2.0;
      const double camber_arc_to_diameter_ratio = camber_rect_arc_length / (d_object.radius * 2.0) - 1.0;
      const double camber_rect_arc_length_ref_scale = 1.0 / (sqrt(d_object.radius * d_object.radius + 0.02 * 0.02) * 2.0 / (d_object.radius * 2.0) - 1.0);

      // define the stall range for this effect
      //Fl_arc      
      // TODO: make this better! effective range is [-30deg, 50deg] AOA, peak at 0 AOA
      // attenuated with AOA as a sinusoid
      // (camber_arc_to_diameter_ratio - 1.0) normalized to a camber_height of 2cm
      // where camber_arc_to_diameter_ratio == 1 means no added lift
      d_forces.lift_force_camber_N = 
        rhov2o2 * A_plate * Cl_BASE * (camber_arc_to_diameter_ratio * camber_rect_arc_length_ref_scale) * cos(d_forces.aoar) *
        (d_forces.aoar <= DEG_TO_RAD(50) && d_forces.aoar >= DEG_TO_RAD(-30) ? 1.0 : 0.0);

    ////// ** ** End Linear Lift Model ** ** //////

    ////// ** ** Start Pitching Moment Model ** ** //////

      //// Use form drag terms to apply some of the resulting pitching moment
      // for now, we just assume that the centre of application for 'plate drag' is 15% forward from the 
      // disc centre. This actually changes with thickness, and other factors, but this approximation is OK
      // for now.
      const double plate_moment_arm_length = gv_aero_debug3 * d_object.radius * 2;

      // The paper seems to imply that the camber (see below) causes extra torque due to the plate drag
      // We could add this to the torque term as a function of 'amplified' torque here
      // using the camber arc length as an extended effective A_plate
      // we may want to add this to the 'd_forces.lin_drag_force_plate_N' later as well, not too sure yet
      // Copied from below:
      // treat this like the pulled out edges of a rectangle for now (seems OK)
      // attenuate this factor with AOA since the entire 'plate' is exposed at some point...?
      const double Fd_plate_pitching_factor = 
        MAX(1.0, (camber_rect_arc_length / (d_object.radius * 2)) * cos(d_forces.aoar));

      // AOA is about the 'X' axis to the right, positive wrt Fd_plate sign, arm is toward the leading end
      //Fd_plate_induced_moment_Nm 
      // TODO: make this better! effective range is [-45, 45deg] AOA, peak at 0 AOA
      // We're only going to limit this for the applied torque for now
      // sign is already handled by the "lin_drag_force_plate_N"
      d_forces.rot_torque_plate_offset_Nm = 
        plate_moment_arm_length * d_forces.lin_drag_force_plate_N * Fd_plate_pitching_factor * abs(sin(d_forces.aoar)) *
        (d_forces.aoar <= DEG_TO_RAD(45) && d_forces.aoar >= DEG_TO_RAD(-45) ? 1.0 : 0.0);

      // HOWEVER: thise nose-down effect here seems too strong.
      // that is making me thing that this is actually caused by the "lower surface of rim camber"
      // on the underside of the rim
      // note how a form drag force applied there would not affect the same force at the back of the disc.
      // I would then propose that the strength of this torque is a function of AOA, and the (maybe?) 
      // normal of the rim camber
      // we'll assume 'camber_height' is symmetric, and apply a torque here as a function of the
      // rim width.
      // optimal angle would be rim_camber_norm_angle = atan2(camber_height, radius) if we assume that is a straight plane
      // then the 'centre' of this effect should be at cos(AOA + rim_camber_norm_angle)
      const double rim_camber_norm_angle = atan2(camber_height, d_object.radius);
      // effect is maxed at cos(aoa + rim_camber_norm_angle - pi/2)

      const double rim_camber_incidence_angle = d_forces.aoar + rim_camber_norm_angle - M_PI_2;
      const double effective_rim_camber_area = d_object.rim_width * d_object.radius * 2 * RIM_CAMBER_EXPOSURE;

      // if the angle is too far nose-down, this is no longer a factor
      // TODO: make this better! effective range is [-15deg, +45deg] AOA, peak at 0 AOA
      d_forces.lin_drag_force_rim_camber_N = 
        rhov2o2 * Cd_EDGE  * effective_rim_camber_area * cos(rim_camber_incidence_angle) *
        (d_forces.aoar <= DEG_TO_RAD(45) && d_forces.aoar >= DEG_TO_RAD(-15) ? 1.0 : 0.0);        

      // assume the force is applied halfway along thr rim width
      const double rim_camber_moment_arm_length = d_object.radius * 2 - d_object.rim_width * 0.5;

      d_forces.rot_torque_rim_camber_offset_Nm = rim_camber_moment_arm_length * d_forces.lin_drag_force_rim_camber_N;            

      //// Use bernoulli lift terms to apply some of the resulting pitching moment
      // from the paper and matlab: 
      // the centre offset for the 'Fl_lip' seems to be around 0.1*diameter offset to the back
      // AOA is about the 'X' axis to the right, negative wrt Fl_lip sign, arm is toward the trailing end
      const double Fl_lip_moment_arm_length = gv_aero_debug4 * d_object.radius * 2;
      d_forces.rot_torque_cavity_edge_offset_Nm = -Fl_lip_moment_arm_length * d_forces.lift_force_cavity_edge_N;

      // after contending with the complication of 'Fd_plate_pitching_factor' in the paper results
      // it looks like there is about a 0.1*diameter moment arm left over for the bernoulli lift effects due to the camber
      // this probably changes with AOA, but we'll just make it static for now
      const double Fl_arc_moment_arm_length = gv_aero_debug5 * d_object.radius * 2;

      // We observe that the camber (below) causes extra torque due to the plate drag
      // AOA is about the 'X' axis to the right, so this is positive wrt Fl_arc sign, arm is toward the leading end
      d_forces.rot_torque_camber_offset_Nm = Fl_arc_moment_arm_length * d_forces.lift_force_camber_N;

    ////// ** ** End Pitching Moment Model ** ** //////

    // sum all forces and torques for aero effects
    d_forces.aero_torque_x = d_forces.rot_drag_torque_x_Nm;
    d_forces.aero_torque_y = d_forces.rot_drag_torque_y_Nm;
    d_forces.aero_torque_z = d_forces.rot_drag_torque_z_Nm;

    d_forces.drag_force_vector *= 0;
    d_forces.drag_force_vector += d_forces.lin_drag_force_edge_N        * edge_force_vector;
    d_forces.drag_force_vector += d_forces.lin_drag_force_plate_N       * d_orientation;
    d_forces.drag_force_vector += d_forces.lin_drag_force_skin_N        * -d_forces.disc_velocity_unit_vector;
    d_forces.drag_force_vector += d_forces.lin_drag_force_cavity_edge_N * edge_force_vector;
    d_forces.drag_force_vector += d_forces.lin_drag_force_rim_camber_N  * d_orientation;

    d_forces.lift_force_vector *= 0;
    d_forces.lift_force_vector += d_forces.lift_force_cavity_edge_N     * d_orientation;
    d_forces.lift_force_vector += d_forces.lift_force_camber_N          * d_orientation;
    // assume d_forces.lin_drag_force_rim_camber_N is all in the disc normal for simplification
    d_forces.lift_induced_pitching_moment *= 0;
    d_forces.lift_induced_pitching_moment += d_forces.rot_torque_rim_camber_offset_Nm;
    d_forces.lift_induced_pitching_moment += d_forces.rot_torque_plate_offset_Nm;
    d_forces.lift_induced_pitching_moment += d_forces.rot_torque_cavity_edge_offset_Nm;
    d_forces.lift_induced_pitching_moment += d_forces.rot_torque_camber_offset_Nm;

    d_forces.aero_force = d_forces.lift_force_vector + d_forces.drag_force_vector;

/*    std::stringstream ss;
    ss << "AOA = " << std::to_string(RAD_TO_DEG(d_forces.aoar));
    ss << ", rim_camber = " << std::to_string(d_forces.rot_torque_rim_camber_offset_Nm);
    ss << ", plate_offset = " << std::to_string(d_forces.rot_torque_plate_offset_Nm);
    ss << ", cavity_edge = " << std::to_string(d_forces.rot_torque_cavity_edge_offset_Nm);
    ss << ", camber_offset = " << std::to_string(d_forces.rot_torque_camber_offset_Nm);
    ss << ", sum = " << std::to_string(d_forces.lift_induced_pitching_moment);
    ss << ", lift_force_cavity_edge = " << std::to_string(d_forces.lift_force_cavity_edge_N / 0.175);
    ss << ", lift_force_camber = " << std::to_string(d_forces.lift_force_camber_N / 0.175);
    ss << ", pos_x = " << std::to_string(d_state.disc_location[0]);
    ss << ", pos_y = " << std::to_string(d_state.disc_location[1]);
    ss << ", pos_z = " << std::to_string(d_state.disc_location[2]);
    std::cout << ss.str() << std::endl;*/

/*    std::stringstream ss;
    ss << "" << std::to_string(RAD_TO_DEG(d_forces.aoar));
    ss << ", " << std::to_string(d_forces.rot_torque_rim_camber_offset_Nm);
    ss << ", " << std::to_string(d_forces.rot_torque_plate_offset_Nm);
    ss << ", " << std::to_string(d_forces.rot_torque_cavity_edge_offset_Nm);
    ss << ", " << std::to_string(d_forces.rot_torque_camber_offset_Nm);
    ss << ", " << std::to_string(d_forces.lift_induced_pitching_moment);
    std::cout << ss.str() << std::endl;*/

    std::stringstream ss;
    ss << "AOA = " << std::to_string(RAD_TO_DEG(d_forces.aoar));
    //ss << ", rim_camber = " << std::to_string(d_forces.rot_torque_rim_camber_offset_Nm);
    //ss << ", plate_offset = " << std::to_string(d_forces.rot_torque_plate_offset_Nm);
    ss << ", cavity_torque = " << std::to_string(d_forces.rot_torque_cavity_edge_offset_Nm);
    ss << ", camber_torque = " << std::to_string(d_forces.rot_torque_camber_offset_Nm);
    ss << ", torque_sum = " << std::to_string(d_forces.lift_induced_pitching_moment);
    ss << ", net_vert_accel = " << std::to_string(d_forces.aero_force[2]/ d_object.mass - GRAV);
    //ss << ", lift_accel_cavity_edge = " << std::to_string(d_forces.lift_force_cavity_edge_N/ 0.175);
    //ss << ", lift_accel_camber = " << std::to_string(d_forces.lift_force_camber_N / 0.175);
    //ss << ", pos_y = " << std::to_string(d_state.disc_location[1]);
    ss << ", vel_z = " << std::to_string(d_state.disc_velocity[2]);
    ss << ", pos_z = " << std::to_string(d_state.disc_location[2]);
    std::cout << ss.str() << std::endl;
  }
}