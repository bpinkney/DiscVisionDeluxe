#include "DfisX.hpp"
#include "Daero.hpp"
#include "dvd_maths.hpp"

#include <iostream> 
#include <math.h>
/*




||||||||||||Daero|||||||||||||||||
Handles the aerodynamic forces of disc simulation.
Also gravity.

*/
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

    Eigen::Vector3d disc_air_velocity_vector = d_velocity - throw_container->disc_environment.wind_vector_xyz;

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

    //#0.5 * pressure * area * velocity^2
    //pav2by2 = p * a * V2 / 2
    d_forces.pav2by2 = throw_container->disc_environment.air_density * d_object.area * d_forces.v2 * 0.5;
    //////////////////////////////////////////Multiuse Variables/////////////////////////////////


    /////////////Calculating the realized flight coefficients////////////////////////////////////
    //normal flight conditions
    if (d_forces.aoar > -0.52 && d_forces.aoar > -0.52)
    {
      d_forces.coefficient_curve =  0.5 * std::sin(6*d_forces.aoar) + std::sin(2*d_forces.aoar);
      d_forces.realized_lift_coefficient =            d_object.lift_coefficient_base +     d_object.lift_coefficient_per_radian * d_forces.coefficient_curve;
      d_forces.realized_pitching_moment_coefficient = d_object.pitching_moment_base  +     d_object.pitching_moment_per_radian  * d_forces.coefficient_curve;
      d_forces.stall_induced_drag = 0.0;

    }
    //stall conditions
    else
    {
      d_forces.stall_curve =         std::sin(2*d_forces.aoar);
      d_forces.realized_lift_coefficient =            d_object.lift_coefficient_base + d_object.lift_coefficient_per_radian * d_forces.stall_curve;
      d_forces.realized_pitching_moment_coefficient = d_object.pitching_moment_base  + d_object.pitching_moment_per_radian  * d_forces.stall_curve;
      d_forces.stall_induced_drag = -std::cos(2*d_forces.aoar)+0.55;
    }
    /////////////Calculating the realized flight coefficients////////////////////////////////////


    /////////////Sim_State calculations for display purposes////////////////////////////////////////
    if      (d_forces.aoar < HS_TURN_CONST)   d_state.sim_state = SIM_STATE_FLYING_HIGH_SPEED_TURN;
    else if (d_forces.aoar < TURN_CONST)      d_state.sim_state = SIM_STATE_FLYING_TURN;
    else if (d_forces.aoar < FADE_CONST)      d_state.sim_state = SIM_STATE_FLYING;
    else                                      d_state.sim_state = SIM_STATE_FLYING_FADE;
  /*
        if      (d_forces.aoar < 0)                                                d_state.sim_state = SIM_STATE_FLYING_HIGH_SPEED_TURN;
      else if (d_forces.realized_pitching_moment_coefficient <= TURN_CONST)      d_state.sim_state = SIM_STATE_FLYING_TURN;
      else if (d_forces.realized_pitching_moment_coefficient >  TURN_CONST && 
               d_forces.realized_pitching_moment_coefficient <  FADE_CONST)      d_state.sim_state = SIM_STATE_FLYING;
      else if (d_forces.realized_pitching_moment_coefficient >= FADE_CONST)      d_state.sim_state = SIM_STATE_FLYING_FADE;
      */
      /////////////Sim_State calculations for display purposes////////////////////////////////////////

  /*
  induced drag Cdi = Cl**2 / pi AR

  AR = 1.27 for a circular disc
  pi * AR = PI_X_AR = 3.99
   */

    // parasidic drag torque calculations:
    
    // Inertia of a thin disc:
    // Iz =      1/2 * m * r^2
    // Ix = Iy = 1/4 * m * r^2

    // torque = accel * I
    const bool use_updated_rotational_drag_model = true;
    const float r2 = (d_object.radius * d_object.radius);
    const float r5 = (d_object.radius * d_object.radius * d_object.radius * d_object.radius * d_object.radius);
    if(use_updated_rotational_drag_model)
    {
      // rotational Reynolds number = Re = omega * r^2 / linear_v
      // where (I think) linear_v is along the rotational plane (not sure)
      // for now, we can just take the total lin vel magnitude...
      Eigen::Vector3d v = disc_air_velocity_vector;
      const float airspeed_vel_mag = std::sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);

      // (I think?) The rotational reynolds number is a function only of the linear aispeed
      // across the disc surface (orthogonal to the disc normal)
      // So we can compute the dot product between the disc normal unit vector, and the airspeed vector
      // to compute the effective angle between the airspeed vector and the disc normal
      // then, we can use this angle to attenuate the magnitude of the airspeed vector to produce 
      // the magnitude of airflow in the disc plane only
      const float ang_disc_normal_to_airspeed = angle_between_vectors(v, d_state.disc_orientation);
      const float airspeed_vel_mag_disc_plane = sin(ang_disc_normal_to_airspeed) * airspeed_vel_mag;
      const float Re_rot = (std::fabs(d_state.disc_rotation_vel) * r2) / MAX(airspeed_vel_mag_disc_plane, CLOSE_TO_ZERO);

      // https://www.sciencedirect.com/topics/engineering/rotating-disc
      // approximate surrounded by laminar   Cm = 3.87Re^(-1/2)
      // approximate surrounded by turbulent Cm = 0.146Re^(-1/5)
      // NO IDEA, let's just tune this with the laminar formula
      float Cm = 0.0;
      if(airspeed_vel_mag_disc_plane > CLOSE_TO_ZERO)
      {
        const float Cm_base = 0.1;
        Cm = Cm_base * (1.0 / MAX(std::sqrt(Re_rot), CLOSE_TO_ZERO));
      }

      // parasidic drag torque = Tq = 0.5 * rho * omega^2 * r^5 * Cm
      // where omega is the angular vel in m/s
      // and 'r' is the radius in m
      d_forces.aero_torque_z =
        -signum(d_state.disc_rotation_vel) *
        0.5 * 
        throw_container->disc_environment.air_density * 
        (d_state.disc_rotation_vel * d_state.disc_rotation_vel) * 
        r5 * 
        Cm;
    }
    else
    {
      const double hacky_spin_drag_rate = 5.0;
      const float Iz = 0.5 * d_object.mass * r2;
      d_forces.aero_torque_z = -signum(d_state.disc_rotation_vel) * hacky_spin_drag_rate * Iz;
    }

    //std::cout << std::to_string(d_forces.step_count) << ": RotParaDrag Torque = " << std::to_string(d_forces.aero_torque_z) << 
    //  " Nm, SPIN = " << std::to_string(d_state.disc_rotation_vel) << "rad/s" << std::endl;


    const bool use_pitching_drag_model = true;

    d_forces.aero_torque_x = 0;
    if(use_pitching_drag_model)
    {
      // from 'drag of rotating disc pitching'
      // k = 0.13412
      // Td = 2.0 * k * Cwdxy * r^5 * rho * w^2

      // https://www.engineeringtoolbox.com/drag-coefficient-d_627.html
      const double Cwdxy = 1.17;

      const double Td = 
        -signum(d_state.disc_pitching_vel) *
        2.0 * 0.13412 *
        Cwdxy *
        r5 *
        throw_container->disc_environment.air_density *
        (d_state.disc_pitching_vel*d_state.disc_pitching_vel);

      d_forces.aero_torque_x = Td;

      //std::cout << std::to_string(d_forces.step_count) << ": Rot Drag XY Torque = " << out.str() << " Nm vs gyro induced torque = " << std::to_string(d_forces.gyro_torque_x) << std::endl;
    }

    d_forces.induced_drag_coefficient  = d_forces.realized_lift_coefficient * d_forces.realized_lift_coefficient / PI_X_AR;
    d_forces.realized_drag_coefficient = d_object.drag_coefficient + d_forces.induced_drag_coefficient + d_forces.stall_induced_drag;

    d_forces.lift_induced_pitching_moment = d_forces.pav2by2 * d_forces.realized_pitching_moment_coefficient * d_object.diameter;
    d_forces.lift_force_magnitude         = d_forces.pav2by2 * d_forces.realized_lift_coefficient;
    d_forces.drag_force_magnitude         = d_forces.pav2by2 * d_forces.realized_drag_coefficient;

    d_forces.lift_force_vector =  d_forces.lift_force_magnitude * d_forces.disc_lift_unit_vector;
    d_forces.drag_force_vector = -d_forces.drag_force_magnitude * d_forces.disc_velocity_unit_vector;

    d_forces.aero_force = d_forces.lift_force_vector + d_forces.drag_force_vector;    
  }
}