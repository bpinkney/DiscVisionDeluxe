// Fill out your copyright notice in the Description page of Project Settings.


#include "DiscThrow.h"
#include "FollowFlight.h"
#include "dvd_maths.hpp"
#include "Math/Vector.h"
#include "disc_layouts.hpp"
#include "dvd_maths.hpp"
#include "FlightLog.h"
#include "PhysXIncludes.h"
#include "Kismet/KismetMathLibrary.h"

// include for debug stuff
#include "Daero.hpp"

#include "UI/RangeHUD.h"


/////decent values 500,100,35
#define end_throw_camera_velocity_cutoff 500.0
#define kill_control_velocity_cutoff     100.0
#define end_throw_velocity_cutoff        35.0

#define desired_dfisx_dt                 0.01
  // Sets default values
ADiscThrow::ADiscThrow()
{
   

  PrimaryActorTick.bCanEverTick = true;
  memset(&throw_container, 0, sizeof(DfisX::Throw_Container));
  memset(&initial_release_stats, 0, sizeof(Initial_Release_Stats));
  memset(&flight_cumulative_stats, 0, sizeof(Flight_Cumulative_Stats));

}

// Called when the game starts or when spawned
void ADiscThrow::BeginPlay()
{
  Super::BeginPlay();

  
  if (GEngine)
  {
  ARangeHUD* RangeHUD = Cast<ARangeHUD>(GetWorld()->GetFirstPlayerController()->GetHUD());
  if (IsValid(RangeHUD)) 
  {
      RangeHUD->SetLatestDiscThrow(this);
  }
  ptr_disc_character = static_cast<ADiscCharacter*>(this->GetOwner());
  ptr_camera_manager = ptr_disc_character->ptr_camera_manager;
  follow_flight_hue = 000.0;
  }

}

// Called every frame
void ADiscThrow::Tick(const float DeltaTime)
{
  Super::Tick(DeltaTime);

//pause checker as we use global time dilation for pausing
  if (is_throw_simulating&&DeltaTime>0.00001)
  {


//////////////////////////Dfisx substepping//////////////////////////
    if (DeltaTime>desired_dfisx_dt)
    {
      int needed_substeps = ceil(DeltaTime/desired_dfisx_dt);
      double substep_dt = DeltaTime / needed_substeps;
      for (int substep = 0; substep < needed_substeps; substep++) {
        DfisX::step_simulation(&throw_container, substep_dt);
        //GEngine->AddOnScreenDebugMessage(-1, 5.0f, FColor::Green,FString::FromInt(substep)+FString::FromInt(needed_substeps));
      }
    }
    else DfisX::step_simulation(&throw_container, DeltaTime);
/////////////////////////end Dfisx substepping//////////////////////


    
    generate_flight_cumulative_stats();
    
    //converting dfisx disc state into unreal usable forms
    DfisX::Disc_State disc_state = DfisX::get_disc_state(&throw_container);
    float xx = disc_state.disc_location[0]*100;
    float yy = disc_state.disc_location[1]*100;
    float zz = disc_state.disc_location[2]*100;
    FVector disc_position = {xx,yy,zz};

    float ii = disc_state.disc_orient_z_vect[0];
    float jj = disc_state.disc_orient_z_vect[1];
    float kk = disc_state.disc_orient_z_vect[2];

    float pitch = RAD_TO_DEG(-atan2(ii,kk));
    float yaw   = RAD_TO_DEG( atan2(jj,kk));
    float roll  = 0.0;

    FVector disc_direction = FVector (disc_state.disc_velocity[0],disc_state.disc_velocity[1],disc_state.disc_velocity[2]);

    // Check for gimbal lock condition
    // This is a hacky work around, but should do the trick
    // In principle, we would only expect one of 'pitch, roll, yaw' to change sign in any one timestep
    // so reverse signs on 2 or more of these eulers implies we have a 'gimbal lock' solution

    // just hack this in with a local static for now, needs to change later
/*    static float last_pitch = 0;
    static float last_yaw   = 0;

    int reverse_count = 0;
    if(signum(pitch) != signum(last_pitch))
    {
      reverse_count++;
    }
    if(signum(yaw) != signum(last_yaw))
    {
      reverse_count++;
    }

    // swap signs if we're locked, make sure this is not the first run
    if(reverse_count > 0 && last_pitch != 0)
    {
      pitch = -pitch;
      yaw   = -yaw;
      roll  = -roll;
    }

    last_pitch = pitch;
    last_yaw   = yaw;*/
    



    
    float ll = disc_state.disc_velocity[0]*100.0;
    float mm = disc_state.disc_velocity[1]*100.0;
    float nn = disc_state.disc_velocity[2]*100.0;

    FVector disc_velocity = {ll,mm,nn};
  
    //float disc_spin = -disc_state.disc_rotation/10;
    //FVector ang_velocity = FVector (0,0,-disc_state.disc_rotation_vel);
    
    // Convert angular rates into the world frame before passing back to unreal
    // (They have no concept of our locally defined 'velocity vector' frame)
    // Note this is column major, so it is really the transpose of what you see here.
    Eigen::Matrix3d Rdw; 
    Rdw << disc_state.disc_orient_x_vect[0], disc_state.disc_orient_y_vect[0], disc_state.disc_orient_z_vect[0],
           disc_state.disc_orient_x_vect[1], disc_state.disc_orient_y_vect[1], disc_state.disc_orient_z_vect[1],
           disc_state.disc_orient_x_vect[2], disc_state.disc_orient_y_vect[2], disc_state.disc_orient_z_vect[2];

    // convert to quaternion
    // define as a simple float, row-major matrix so we can use R2eul
    // Since unreal defines the LOCAL Z with the wrong sign, we flip the 'disc_orient_z_vect' normal here
    float MAT3X3(Rdw_float) = 
    {
      disc_state.disc_orient_x_vect[0], disc_state.disc_orient_x_vect[1], disc_state.disc_orient_x_vect[2],
      disc_state.disc_orient_y_vect[0], disc_state.disc_orient_y_vect[1], disc_state.disc_orient_y_vect[2],
      disc_state.disc_orient_z_vect[0], disc_state.disc_orient_z_vect[1], disc_state.disc_orient_z_vect[2]
    };

    float MAT3X3(Rwd_float) = 
    {
      disc_state.disc_orient_x_vect[0], disc_state.disc_orient_y_vect[0], disc_state.disc_orient_z_vect[0],
      disc_state.disc_orient_x_vect[1], disc_state.disc_orient_y_vect[1], disc_state.disc_orient_z_vect[1],
      disc_state.disc_orient_x_vect[2], disc_state.disc_orient_y_vect[2], disc_state.disc_orient_z_vect[2]
    };

    float VEC3(eulers_zyx);
    //Rxyz2eulxyz(Rdw_float, eulers_xyz);
    Rzyx2eulxyz(Rwd_float, eulers_zyx);

    // take negative rotations (except for Z?)
    float eul_zyx_deg[3] = {RAD_TO_DEG(eulers_zyx[0]), RAD_TO_DEG(eulers_zyx[1]), RAD_TO_DEG(eulers_zyx[2])};

/*    GEngine->AddOnScreenDebugMessage(-1, 30.f, FColor::Red,(FString::SanitizeFloat(eul_zyx_deg[0])));
    GEngine->AddOnScreenDebugMessage(-1, 30.f, FColor::Red,(FString::SanitizeFloat(eul_zyx_deg[1])));
    GEngine->AddOnScreenDebugMessage(-1, 30.f, FColor::Red,(FString::SanitizeFloat(eul_zyx_deg[2])));  
    GEngine->AddOnScreenDebugMessage(-1, 30.f, FColor::Yellow,(FString::SanitizeFloat(roll)));
    GEngine->AddOnScreenDebugMessage(-1, 30.f, FColor::Yellow,(FString::SanitizeFloat(pitch)));
    GEngine->AddOnScreenDebugMessage(-1, 30.f, FColor::Yellow,(FString::SanitizeFloat(yaw)));
    GEngine->AddOnScreenDebugMessage(-1, 30.f, FColor::Green, FString(" "));*/

    // Now we should be able to rotate the XYZ 'velocity disc frame' ang rates into the world frame
    // NEGATIVE Z due to unreal's weird frame!
    Eigen::Vector3d local_ang_vel_radps = {disc_state.disc_rolling_vel, disc_state.disc_pitching_vel, -disc_state.disc_rotation_vel};//{disc_state.disc_pitching_vel, disc_state.disc_rolling_vel, -disc_state.disc_rotation_vel};
    Eigen::Vector3d world_ang_vel_radps = Rdw * local_ang_vel_radps;

    // From the FVector defs in Unreal, we presume this to be rotational rates about the XYZ world axes
    FVector ang_velocity = FVector 
      (
        world_ang_vel_radps[0],
        world_ang_vel_radps[1],
        world_ang_vel_radps[2]
      );

    //{-eul_xyz_deg[1], -eul_xyz_deg[2], -eul_xyz_deg[0]};
    // FRotator defined as [Pitch, Yaw, Roll] for who knows what reason
    // Is this the order of rotations??? THESE ARE EULER ANGLES UNREAL, IT MATTERS
    // IT seems like the rotation order is XYZ (roll, pitch, yaw)
    // Consquently, the yaw position is not meaningful, and should just be zero
    // We can populate this with 'd_state.disc_rotation' is we want to see it spin
    // remember that this is not well defined wrt out airspeed vector unit frame
    //Eigen::Vector3d spin_pos_body = {0.0, 0.0, disc_state.disc_rotation};
    // remember that this is a rotation position, so in order to change rotational frames
    // we need ang_pos_vec_world = R * ang_pos_vec * R';
    //Eigen::Matrix<double, 1, 3> spin_pos_world = Rdw * spin_pos_body;
    //spin_pos_world = spin_pos_world * Rdw.transpose();
    //float spin_pos_world_Z = spin_pos_world[2];
    //WRAP_TO_2PI(spin_pos_world_Z);
    //spin_pos_world_Z = RAD_TO_DEG(spin_pos_world_Z);

    //GEngine->AddOnScreenDebugMessage(-1, 30.f, FColor::Red,(FString::SanitizeFloat(spin_pos_body[2])));
    //GEngine->AddOnScreenDebugMessage(-1, 30.f, FColor::Yellow,(FString::SanitizeFloat(spin_pos_world_Z)));
    //GEngine->AddOnScreenDebugMessage(-1, 30.f, FColor::Green, FString(" "));

    // This is another way to compute these eulers, keep this code here for easy checking of errors!
    //FVector disc_rotation_forward_vector = FVector (disc_state.disc_orient_x_vect[0],disc_state.disc_orient_x_vect[1],disc_state.disc_orient_x_vect[2]);
    //FVector disc_rotation_side_vector = FVector (disc_state.disc_orient_y_vect[0],disc_state.disc_orient_y_vect[1],disc_state.disc_orient_y_vect[2]);    
    //float roll_angle = FMath::RadiansToDegrees(acosf(FVector::DotProduct(disc_rotation_side_vector, FVector(0,0,1))));
    //FRotator disc_rotation = disc_rotation_forward_vector.Rotation();
    //disc_rotation.Roll = roll_angle-90;
    //GEngine->AddOnScreenDebugMessage(-1, 30.f, FColor::Yellow,(FString::SanitizeFloat(disc_rotation.Roll)));
    //GEngine->AddOnScreenDebugMessage(-1, 30.f, FColor::Yellow,(FString::SanitizeFloat(disc_rotation.Pitch)));
    //GEngine->AddOnScreenDebugMessage(-1, 30.f, FColor::Yellow,(FString::SanitizeFloat(disc_rotation.Yaw)));*/

    FRotator disc_rotation = {0,0,0};

    // Roll and pitch are negative due to the swapped Z axis!
    disc_rotation.Roll  = -eul_zyx_deg[2];
    disc_rotation.Pitch = -eul_zyx_deg[1];
    disc_rotation.Yaw   = eul_zyx_deg[0]; 

/*      GEngine->AddOnScreenDebugMessage(-1, 30.f, FColor::Red,(FString::SanitizeFloat(-eul_zyx_deg[2])));
      GEngine->AddOnScreenDebugMessage(-1, 30.f, FColor::Red,(FString::SanitizeFloat(-eul_zyx_deg[1])));
      GEngine->AddOnScreenDebugMessage(-1, 30.f, FColor::Red,(FString::SanitizeFloat(eul_zyx_deg[0])));*/
          
    //GEngine->AddOnScreenDebugMessage(-1, 30.f, FColor::Red,(FString(" "))); 

    
    //ptr_disc_projectile->SetDiscPosRot(disc_position,disc_rotation,disc_velocity,disc_spin_rate);

    FVector orientationtestoutput = FVector (ii,jj,kk);
    ///log_string(ang_velocity.ToString());
    log_string(FString(" "));
    log_string(FString("Set states at delta time: "));
    log_string(FString::SanitizeFloat(DeltaTime));
    
    log_string(FString(" "));
    log_string(FString("set disc vel"));
    log_string(disc_velocity.ToString());
    log_string(FString("set ang vel"));
    log_string(ang_velocity.ToString());
    log_string(FString("set disc rot"));
    log_string(disc_rotation.ToString());
    log_string(FString("raw orientation"));
    log_string(orientationtestoutput.ToString());
    log_string(FString(" "));
    ptr_disc_projectile->SetDiscVelRot(disc_velocity, ang_velocity, disc_rotation, -disc_state.disc_rotation);
    //finish converting dfisx disc state into unreal usable forms

    // read back to check
    //FVector ang_vel_readback = ptr_disc_projectile->GetPhysicsAngularVelocityInRadians();
/*    GEngine->AddOnScreenDebugMessage(-1, 30.f, FColor::Red,(FString::SanitizeFloat(local_ang_vel_radps[0])));
    GEngine->AddOnScreenDebugMessage(-1, 30.f, FColor::Red,(FString::SanitizeFloat(local_ang_vel_radps[1])));
    GEngine->AddOnScreenDebugMessage(-1, 30.f, FColor::Red,(FString::SanitizeFloat(local_ang_vel_radps[2])));
    GEngine->AddOnScreenDebugMessage(-1, 30.f, FColor::Green, FString(" "));*/
    /*GEngine->AddOnScreenDebugMessage(-1, 30.f, FColor::Yellow,(FString::SanitizeFloat(world_ang_vel_radps[0])));
    GEngine->AddOnScreenDebugMessage(-1, 30.f, FColor::Yellow,(FString::SanitizeFloat(world_ang_vel_radps[1])));
    GEngine->AddOnScreenDebugMessage(-1, 30.f, FColor::Yellow,(FString::SanitizeFloat(world_ang_vel_radps[2])));
     GEngine->AddOnScreenDebugMessage(-1, 30.f, FColor::Green, FString(" "));
      GEngine->AddOnScreenDebugMessage(-1, 30.f, FColor::Green, FString(" "));*/

    //unused sim states for now: SIM_STATE_STOPPED,SIM_STATE_STARTED,SIM_STATE_SKIPPING,SIM_STATE_TREE_HIT,SIM_STATE_ROLLING,SIM_STATE_SLIDING  transition_to_colour
    
    ptr_follow_flight->log_position();
    ptr_flight_log->log_position(DeltaTime);
    

    /////////////velocity cutoff event state checks/////////////////////////////////
    //
/*
TODO: nest these once good values are determined
*/
    //GEngine->AddOnScreenDebugMessage(-1, 0.5f, FColor::Green,FString::SanitizeFloat(disc_velocity.Size()));
    //GEngine->AddOnScreenDebugMessage(-1, 0.5f, FColor::Red,FString::SanitizeFloat(ang_velocity.Size()));
    //landscape fall through safety check
    if (zz<0) end_throw_simulation();    
    if ((disc_velocity.Size()+ang_velocity.Size()) < end_throw_velocity_cutoff)       end_throw_simulation();
    if (disc_velocity.Size() < kill_control_velocity_cutoff)     ptr_disc_projectile->kill_control();
    if (disc_velocity.Size() < end_throw_camera_velocity_cutoff) ptr_disc_projectile->end_of_throw_camera();

  }
}



void ADiscThrow::GenerateDiscEnv(DfisX::Disc_Env * disc_environment)
{
  // use default for now
  disc_environment->wind_vector_xyz = Eigen::Vector3d(0,0,0); // m/s
  disc_environment->gust_factor = DfisX::Gust_Factor::TEN_HOMICIDAL_HURRICANE;//ZERO_DEAD_DIDDLY;
  disc_environment->air_density = ISA_RHO;
}


void ADiscThrow::new_captured_throw(
  const int captured_disc_index, 
  const FVector captured_position, 
  const FVector captured_velocity, 
  const float captured_world_roll, 
  const float captured_world_pitch, 
  const float captured_spin_speed, 
  const float captured_wobble)
{
  // Screen space > world space
  //
  //This function transforms throw data from a location/direciton relative to the 
  //current player(and hopefully player camera) position and rotation and transforms 
  //it into world space within the engine
  

  /////////////////Rotation
  //
  FRotator character_rotation = GetWorld()->GetFirstPlayerController()->GetControlRotation();
  FRotator disc_rotation = FRotator(RAD_TO_DEG(captured_world_pitch), character_rotation.Yaw, RAD_TO_DEG(captured_world_roll)); ///this is a vector just for the transform function
  FVector thrown_disc_orientation = disc_rotation.RotateVector(FVector(0.0, 0.0, 1.0));


  ///////////////////Position
  //
  FVector character_location = ptr_disc_character->GetActorLocation();
  FVector thrown_disc_position = character_location + FTransform(character_rotation).TransformVector(captured_position);
  

  //////////////////Velocity
  //
  //GEngine->AddOnScreenDebugMessage(-1, 15.0f, FColor::Red, TEXT("Das Velocity!"));
  //GEngine->AddOnScreenDebugMessage(-1, 15.0f, FColor::Red, captured_velocity.ToString());
  FVector thrown_disc_velocity = FTransform(character_rotation).TransformVector(captured_velocity);
  //GEngine->AddOnScreenDebugMessage(-1, 15.0f, FColor::Red, thrown_disc_velocity.ToString());


  new_throw_world_frame(captured_disc_index,thrown_disc_position,thrown_disc_velocity,thrown_disc_orientation,captured_spin_speed,captured_wobble);
}




///used for captured throws
void ADiscThrow::new_throw_world_frame(
  const int disc_index,
  const FVector thrown_disc_position,
  const FVector thrown_disc_velocity, 
  const FVector thrown_disc_orientation, 
  const float thrown_disc_radians_per_second, 
  const float thrown_disc_wobble)
{
  
  spawn_disc_and_follow_flight();

  Eigen::Vector3d v3d_thrown_disc_position = Eigen::Vector3d(thrown_disc_position.X/100.0,thrown_disc_position.Y/100.0,thrown_disc_position.Z/100.0);
  Eigen::Vector3d v3d_thrown_disc_velocity = Eigen::Vector3d(thrown_disc_velocity.X,thrown_disc_velocity.Y,thrown_disc_velocity.Z);
  Eigen::Vector3d v3d_thrown_disc_orientation = Eigen::Vector3d(thrown_disc_orientation.X,thrown_disc_orientation.Y,thrown_disc_orientation.Z);
  
  // get current wind state (we'll need to update this periodically later as the disc changes environments)
  GenerateDiscEnv(&(throw_container.disc_environment));

  DfisX::new_throw(
    &throw_container,
    static_cast<DiscIndex>(disc_index),
    v3d_thrown_disc_position,
    v3d_thrown_disc_velocity,
    v3d_thrown_disc_orientation,
    thrown_disc_radians_per_second,
    thrown_disc_wobble);




////////////////////////print disc name/////////////
  std::stringstream ss;
  ss << "Threw a ";
  ss << throw_container.disc_object.manufacturer << " ";
  ss << throw_container.disc_object.mold_name;

  FString output_text(ss.str().c_str());
  GEngine->AddOnScreenDebugMessage(-1, 5.0f, FColor::Red, output_text); 
////////////////////////end print disc name/////////




////////////////////////////        Initial release Stats          /////////////////////////////////////////////////////////

    initial_release_stats.initial_direction_vector = FVector (throw_container.current_disc_state.disc_velocity[0],throw_container.current_disc_state.disc_velocity[1],throw_container.current_disc_state.disc_velocity[2]);
    initial_release_stats.initial_location_vector = FVector (throw_container.current_disc_state.disc_location[0],throw_container.current_disc_state.disc_location[1],throw_container.current_disc_state.disc_location[2]);
    initial_release_stats.initial_orientation_vector = FVector (throw_container.current_disc_state.disc_orient_z_vect[0],throw_container.current_disc_state.disc_orient_z_vect[1],throw_container.current_disc_state.disc_orient_z_vect[2]);
    initial_release_stats.initial_polarity = copysignf(1.0, throw_container.current_disc_state.disc_rotation_vel);

    initial_release_stats.initial_direction_vector.Normalize();
    initial_release_stats.initial_location_vector.Normalize();
    initial_release_stats.initial_orientation_vector.Normalize();
    //end these arent for hud display

    //initial_speed
    initial_release_stats.initial_speed = throw_container.current_disc_state.disc_velocity.norm();

    //inital_spin_rate
    initial_release_stats.initial_spin_rate = throw_container.current_disc_state.disc_rotation_vel / (M_PI * 2.0) * 60.0;

    //initial_spin_percent
    float theoretical_max_spin_speed = initial_release_stats.initial_speed / throw_container.disc_object.radius;
    initial_release_stats.initial_spin_percent = throw_container.current_disc_state.disc_rotation_vel / theoretical_max_spin_speed * 100.0;

    //initial_direction
    initial_release_stats.initial_direction = 0;

    //initial_loft
    float vertical_speed   = throw_container.current_disc_state.disc_velocity [2];
    float horizontal_speed = sqrt (throw_container.current_disc_state.disc_velocity [0] * throw_container.current_disc_state.disc_velocity [0] +throw_container.current_disc_state.disc_velocity [1] * throw_container.current_disc_state.disc_velocity [1]);
    initial_release_stats.initial_loft = RAD_TO_DEG(atan(vertical_speed/horizontal_speed));

    //initial_hyzer
    //directions as unit vectors
    initial_release_stats.initial_disc_right_vector = initial_release_stats.initial_direction_vector ^ FVector (0,0,1);
    FVector velocity_up_vector = initial_release_stats.initial_disc_right_vector ^ initial_release_stats.initial_direction_vector;

    //projections of orientation onto the direction vectors
    initial_release_stats.initial_disc_right_vector = (initial_release_stats.initial_orientation_vector | initial_release_stats.initial_disc_right_vector) * initial_release_stats.initial_disc_right_vector;
    velocity_up_vector = (initial_release_stats.initial_orientation_vector | velocity_up_vector) * velocity_up_vector;

    initial_release_stats.initial_hyzer = RAD_TO_DEG(atan2(initial_release_stats.initial_disc_right_vector.Size() , velocity_up_vector.Size()));
    /*
     GEngine->AddOnScreenDebugMessage(-1, 30.f, FColor::Green,(FString::SanitizeFloat(initial_release_stats.initial_orientation_vector[1])));
 GEngine->AddOnScreenDebugMessage(-1, 30.f, FColor::Green,(FString::SanitizeFloat(disc_right_vector.Size())));
  GEngine->AddOnScreenDebugMessage(-1, 30.f, FColor::Green,(FString::SanitizeFloat(velocity_up_vector.Size())));
    GEngine->AddOnScreenDebugMessage(-1, 30.f, FColor::Green,(FString::SanitizeFloat(initial_release_stats.initial_hyzer)));
*/
    //initial_nose_up
    initial_release_stats.initial_nose_up = RAD_TO_DEG(throw_container.current_disc_state.forces_state.aoar);

    //initial_wobble
    initial_release_stats.initial_wobble = 0;
    
////////////////////////////////     end Initial release Stats            //////////////////////////////////////////////
}


void ADiscThrow::spawn_disc_and_follow_flight()
{
  is_throw_simulating = true;
  

    // Get the camera transform.
    FVector forward_offset = FVector (0,0,40);///temp offset to prevent from colliding with invisible character model 
    FVector current_location = forward_offset + ptr_disc_character->GetActorLocation();
    
    UWorld* World = GetWorld();
    FActorSpawnParameters SpawnParams;
    SpawnParams.Owner = this;
    SpawnParams.Instigator = ptr_disc_character;

    ptr_disc_projectile = World->SpawnActor<ADiscProjectile>(ProjectileClass, current_location, FRotator(0,0,0), SpawnParams);

    AddTickPrerequisiteActor(ptr_disc_projectile);
    
    const bool turn_off_collision   = false;
    const bool turn_off_follow_cam  = false;

    if(turn_off_collision)
    {
      ptr_disc_projectile->SetActorEnableCollision(false);
    }

    if(!turn_off_follow_cam)
    {
      ptr_camera_manager->focus_on_disc(ptr_disc_projectile);
    }



/////////////////Follow flight spawn and init////////////////////////////////
    SpawnParams.Owner = ptr_disc_projectile;
    ptr_follow_flight = World->SpawnActor<AFollowFlight>(FollowFlightBP, FVector(0,0,0), FRotator(0,0,0), SpawnParams);
    ptr_flight_log    = World->SpawnActor<AFlightLog>(FlightLogBP, FVector(0,0,0), FRotator(0,0,0), SpawnParams);
    ptr_flight_log->ptr_disc_projectile = ptr_disc_projectile;

  //float set_disc_hue = 0.00;
  FColor set_player_colour = FColor::Black;
  //enum_ff_display_shape set_shape = enum_ff_display_shape::Bandsaw;//Spiral;
  //bool set_rainbow = true;
  float set_disc_hue = (float)FMath::RandRange(1,360);
  bool set_rainbow = 0 == FMath::RandRange(0,3); 
  enum_ff_display_shape set_shape = (enum_ff_display_shape)FMath::RandRange(0,9);

  ptr_follow_flight->init (set_disc_hue,set_player_colour,set_shape,set_rainbow);
//////////////////end Follow flight spawn and init////////////////////////////////////
  
}

void ADiscThrow::end_throw_simulation ()
{
  is_throw_simulating = false;
  ptr_follow_flight->unselect();
  ptr_disc_character->throw_was_finished();
  ptr_disc_projectile->DisableComponentsSimulatePhysics();
}

ADiscThrow::Initial_Release_Stats* ADiscThrow::get_initial_release_stats()
  {
    return &initial_release_stats;
  }

ADiscThrow::Flight_Cumulative_Stats* ADiscThrow::get_flight_cumulative_stats()
  {
    return &flight_cumulative_stats;
  }

void ADiscThrow::generate_flight_cumulative_stats()
{
    ARangeHUD* RangeHUD = Cast<ARangeHUD>(GetWorld()->GetFirstPlayerController()->GetHUD());
    if (RangeHUD)
    {
        
        static bool initialized = false;
        if(!initialized)
        {
          RangeHUD->InitializeDAero();
          initialized = true;
        }

        // we need to use this structure since we get memory exceptions otherwise
        // wow, sketchy
        throw_container.debug.debug0 = RangeHUD->GetAero1Input();
        throw_container.debug.debug1 = RangeHUD->GetAero2Input();
        throw_container.debug.debug2 = RangeHUD->GetAero3Input();
        throw_container.debug.debug3 = RangeHUD->GetAero4Input();
        throw_container.debug.debug4 = RangeHUD->GetAero5Input();
        throw_container.debug.debug5 = RangeHUD->GetAero6Input();
    }
    
    
  flight_cumulative_stats.current_distance   = (throw_container.disc_state_array[0].disc_location   -   throw_container.current_disc_state.disc_location).norm();
    flight_cumulative_stats.current_speed      = throw_container.current_disc_state.disc_velocity.norm();
    flight_cumulative_stats.current_spin       = throw_container.current_disc_state.disc_rotation_vel / (M_PI*2.0) * 60.0;
    

    //find the distance from the intial velocity vector 
    FVector n = initial_release_stats.initial_direction_vector;
    n.Normalize();
    FVector a = initial_release_stats.initial_location_vector;
    FVector p = FVector (throw_container.current_disc_state.disc_location[0],throw_container.current_disc_state.disc_location[1],throw_container.current_disc_state.disc_location[2]);
    FVector a_p = a - p;
    FVector tf_vector = a_p - (FVector::DotProduct(a_p,n))*n;

    //find the polarity
    FVector fade_vector = initial_release_stats.initial_disc_right_vector * initial_release_stats.initial_polarity;
    if ((a_p-fade_vector).Size()>(a_p-fade_vector).Size())
      flight_cumulative_stats.current_turnfade   = -tf_vector.Size();
    else
      flight_cumulative_stats.current_turnfade   = tf_vector.Size();

    


    

    flight_cumulative_stats.current_wobble     = 0; ///will add later



}

// disables aero and DfisX immediately after a hit
#define DISABLE_COMPLEX_DISC_COLLISION (false)
// disables aero and DfisX if both of these conditions are met
// disable for now
#define DISABLE_COMPLEX_DISC_COLLISION_MIN_SPEED_MPS (0.3)
#define DISABLE_COMPLEX_DISC_COLLISION_MIN_SPIN_RADPS (1.0)

double           angle_between_vectors    (Eigen::Vector3d a, Eigen::Vector3d b) 
{
    double angle = 0.0;
    angle = std::atan2(a.cross(b).norm(), a.dot(b));
    return angle;
}

static std::string EigenVect3dToString(Eigen::Vector3d vect)
{
    std::stringstream ss;
    ss << vect.transpose();
    return ss.str();
}

void ADiscThrow::on_collision(
  const FVector disc_position,          //world frame
  const FVector disc_rotation, 
  const TArray<FVector> hit_location,   //disc frame? microstepping seems to ruin this, cms
  const TArray<FVector> hit_normal,     //unit direction
  const TArray<FVector> normal_impulse, //looks si, magnitude and direction
  const FVector lin_vel,                //world frame
  const FVector lin_vel_delta,          //world frame
  const FVector ang_vel,                //disc frame
  const FVector world_ang_vel,
  const FVector world_ang_vel_delta,
  const FVector ang_vel_delta,          //disc frame
  const int total_hit_events,
  const float delta_time,               //si
  const TArray<float> hit_friction,
  const TArray<float> hit_restitution)       

{

  // change this unused variable randomly using sed to force unreal to rebuild
  const int changevar = 9565;

  if
  (
    DISABLE_COMPLEX_DISC_COLLISION
    ||
    (
      throw_container.current_disc_state.disc_velocity.norm() < DISABLE_COMPLEX_DISC_COLLISION_MIN_SPEED_MPS
      &&
      abs(throw_container.current_disc_state.disc_rotation_vel) < DISABLE_COMPLEX_DISC_COLLISION_MIN_SPIN_RADPS
    )
  )
  {
    //ptr_disc_projectile->kill_control();    
  }

  // NOTE: Mike says this dt is incorrect, and we actually need to use the one
  // passed into the DfisX step. That means everything here which uses dt (and is useful) should migrate to 'consume_Dcollision'!
  const float dt = delta_time;

  // set throw controller collision input states
  throw_container.collision_input.consumed_input = 255; // mark as invalid first for thread safety?

  throw_container.collision_input.delta_time_s = delta_time;

  Eigen::Vector3d world_ang_vel_radps;
  Eigen::Vector3d world_ang_vel_delta_radps;
  Eigen::Vector3d world_lin_acc;
  Eigen::Vector3d world_ang_acc;

  uint8_t i;
  for(i = 0; i < 3; i++)
  {
    throw_container.collision_input.lin_pos_m[i]                  = disc_position[i]*0.01; //cm to m, world frame
    throw_container.collision_input.lin_vel_mps[i]                = lin_vel[i]*0.01;       //cm to m, world frame
    world_ang_vel_radps[i]                                        = world_ang_vel[i];      // about the XYZ unreal world frame (we think)
    world_ang_vel_delta_radps[i]                                  = world_ang_vel_delta[i];      // about the XYZ unreal world frame (we think)
    throw_container.collision_input.disc_rotation[i]              = disc_rotation[i];

    // Try to re-derive the linear force from the vel delta for comparison and validation
    world_lin_acc[i] = lin_vel_delta[i]*0.01 / dt;
    world_ang_acc[i] = world_ang_vel_delta[i] / dt;
  }

  throw_container.collision_input.lin_force_from_delta_vel_N = world_lin_acc * throw_container.disc_object.mass;

  // try to build a rotation matrix from the local disc unit vectors
  // ideally, this will result in a rotation from the world frame to the local disc frame
  // validation on this should be possible by comparing 
  // (R*world_ang_vel_radps)[2] and disc_ang_vel_radps[2], since the Z axis frames are aligned

  // Note this is column major, so it is really the transpose of what you see here.
  Eigen::Matrix3d Rwd; 
  Rwd << throw_container.current_disc_state.disc_orient_x_vect[0], throw_container.current_disc_state.disc_orient_y_vect[0], throw_container.current_disc_state.disc_orient_z_vect[0],
         throw_container.current_disc_state.disc_orient_x_vect[1], throw_container.current_disc_state.disc_orient_y_vect[1], throw_container.current_disc_state.disc_orient_z_vect[1],
         throw_container.current_disc_state.disc_orient_x_vect[2], throw_container.current_disc_state.disc_orient_y_vect[2], throw_container.current_disc_state.disc_orient_z_vect[2];

  // per the note above, we need to transpose this
  Rwd = Rwd.transpose();

  // Now we should be able to rotate the XYZ angular rates into the disc frame:

  Eigen::Vector3d world_ang_vel_radps_rpy = {world_ang_vel_radps[0], world_ang_vel_radps[1], world_ang_vel_radps[2]};
  Eigen::Vector3d world_ang_vel_delta_radps_rpy = {world_ang_vel_delta_radps[0], world_ang_vel_delta_radps[1], world_ang_vel_delta_radps[2]};

  Eigen::Vector3d local_ang_vel_radps = Rwd * world_ang_vel_radps_rpy;
  Eigen::Vector3d local_ang_vel_delta_radps = Rwd * world_ang_vel_delta_radps_rpy;

  // reversed due to swapped Z axis in Unreal
  //local_ang_vel_radps[0] *= -1;
  //local_ang_vel_radps[1] *= -1;
  local_ang_vel_radps[2]       *= -1;
  local_ang_vel_delta_radps[2] *= -1;

  throw_container.collision_input.ang_vel_radps = local_ang_vel_radps;
  throw_container.collision_input.ang_vel_delta_radps = local_ang_vel_delta_radps;

  log_string(FString("Collision DF ang vel"));
  FVector df_ang_vel = FVector(throw_container.collision_input.ang_vel_radps[0], throw_container.collision_input.ang_vel_radps[1], throw_container.collision_input.ang_vel_radps[2]);
  log_string(df_ang_vel.ToString());
  log_string(FString("Collision WF ang vel"));
  FVector wf_ang_vel = FVector(world_ang_vel_radps_rpy[0], world_ang_vel_radps_rpy[1], world_ang_vel_radps_rpy[2]);
  log_string(wf_ang_vel.ToString());
  log_string(FString("Collision UDF ang vel"));
  FVector udf_ang_vel = FVector(ang_vel[0], ang_vel[1], ang_vel[2]);
  log_string(udf_ang_vel.ToString());

/*  GEngine->AddOnScreenDebugMessage(-1, 30.f, FColor::Green,FString(EigenVect3dToString(throw_container.collision_input.ang_vel_radps).c_str()));
  GEngine->AddOnScreenDebugMessage(-1, 30.f, FColor::Yellow,FString(EigenVect3dToString(world_ang_vel_radps_rpy).c_str()));
  GEngine->AddOnScreenDebugMessage(-1, 30.f, FColor::Red,(FString::SanitizeFloat(ang_vel[0])));
  GEngine->AddOnScreenDebugMessage(-1, 30.f, FColor::Red,(FString::SanitizeFloat(ang_vel[1])));
  GEngine->AddOnScreenDebugMessage(-1, 30.f, FColor::Red,(FString::SanitizeFloat(ang_vel[2])));*/

  // 1. derive torque from the ang vel DELTA:
  const double Ix = 1.0/4.0 * throw_container.disc_object.mass * (throw_container.disc_object.radius*throw_container.disc_object.radius);
  const double Iy = 1.0/4.0 * throw_container.disc_object.mass * (throw_container.disc_object.radius*throw_container.disc_object.radius);
  const double Iz = 1.0/2.0 * throw_container.disc_object.mass * (throw_container.disc_object.radius*throw_container.disc_object.radius);

  // Get current ang vel in local disc airspeed vel frame RPY -> rotation rates about disc plane XYZ
  Eigen::Vector3d last_local_ang_vel_radps = {throw_container.current_disc_state.disc_rolling_vel, throw_container.current_disc_state.disc_pitching_vel, throw_container.current_disc_state.disc_rotation_vel};

  Eigen::Vector3d local_ang_accel_radps2 = Rwd * world_ang_acc;

  Eigen::Vector3d local_ang_torque_Nm;
  local_ang_torque_Nm[0] = local_ang_accel_radps2[0] * Ix;
  local_ang_torque_Nm[1] = local_ang_accel_radps2[1] * Iy;
  local_ang_torque_Nm[2] = local_ang_accel_radps2[2] * Iz;
  throw_container.collision_input.ang_torque_from_delta_vel_Nm = local_ang_torque_Nm;

  // OR
  // 2. Derive torque from the hit location

  // due to horrible mid-hit propagation and sub-stepping in unreal, we can't trust the hit_location magnitude
  // we are receiving. Attenuate the signal so the disc doesn't freak out (HACK)
  const double moment_arm_scale = 0.1;

  int k = 0;
  // reset torque sum
  throw_container.collision_input.ang_torque_from_impulses_Nm *= 0;
  throw_container.collision_input.lin_force_from_impulses_N *= 0;
  for(i = 0; i < total_hit_events; i++)
  {
    Eigen::Vector3d world_frame_moment_arm;
    Eigen::Vector3d world_frame_normal_force_N;
    for(k = 0; k < 3; k++)
    {
      world_frame_moment_arm[k] = hit_location[i][k]*0.01 * moment_arm_scale; // cm to m
      // Unreal forces are in "kg cm / s^2" (which is 0.01 N)
      // Presuming then that their impulses are in 'kg cm / s' -> 0.01 Ns ? (this seems to be correct)
      world_frame_normal_force_N[k] = normal_impulse[i][k]/dt*0.01; 
    }
   
    // Compute torque
    Eigen::Vector3d world_frame_torque_Nm = world_frame_moment_arm.cross(world_frame_normal_force_N);

    // rotate to disc frame
    Eigen::Vector3d disc_frame_torque_Nm = Rwd * world_frame_torque_Nm;

    // add linear force to total force vector
    throw_container.collision_input.lin_force_from_impulses_N += world_frame_normal_force_N;

    // add impulse step to input torque
    throw_container.collision_input.ang_torque_from_impulses_Nm += disc_frame_torque_Nm;

    //GEngine->AddOnScreenDebugMessage(-1, 30.f, FColor::Green,FString(EigenVect3dToString(world_frame_moment_arm).c_str()));
    //GEngine->AddOnScreenDebugMessage(-1, 30.f, FColor::Red,(FString::SanitizeFloat(world_frame_moment_arm.norm())));
  }

  // get ang vel back from the resulting torques since we have decided not to trust Unreal to update our roll/pitch vel
  Eigen::Vector3d ang_accel_from_impulses_radps2 = throw_container.collision_input.ang_torque_from_impulses_Nm;
  ang_accel_from_impulses_radps2[0] /= Ix;
  ang_accel_from_impulses_radps2[1] /= Iy;
  ang_accel_from_impulses_radps2[2] /= Iz;

  // integrate previous ang vel state
  throw_container.collision_input.ang_vel_from_impulses_Nm = last_local_ang_vel_radps + ang_accel_from_impulses_radps2 * dt; //throw_container.collision_input.ang_vel_radps + ang_accel_from_impulses_radps2 * dt;

  // overwrite previous ang vel state (this wil behave as if the propagation had occurred on the k-1 timestep, so we see the gyro response during time 'k')
  // only doing this for roll/pitch for now
  //throw_container.collision_input.ang_vel_radps[0] = ang_vel_from_impulses_Nm[0];
  //throw_container.collision_input.ang_vel_radps[1] = ang_vel_from_impulses_Nm[1];

  // Check for ang vel reversal, this is annoying, but necessary, probably due to the atan stuff at the top of this file
  /*int reverse_count = 0;
  if(signum(throw_container.current_disc_state.disc_pitching_vel) != signum(throw_container.collision_input.ang_vel_radps[0]))
  {
    //reverse_count++;
  }
  if(signum(throw_container.current_disc_state.disc_rolling_vel) != signum(throw_container.collision_input.ang_vel_radps[1]))
  {
    //reverse_count++;
  }
  if(signum(throw_container.current_disc_state.disc_rotation_vel) != signum(throw_container.collision_input.ang_vel_radps[2]))
  {
    reverse_count++;
  }

  if(reverse_count > 0) // just base this on the spin for now, since the roll/pitch are derived from the impulse?? Nope.
  {
    //GEngine->AddOnScreenDebugMessage(-1, 30.f, FColor::Red,FString("FORCED TO RECTIFY ANG VELS!"));
    throw_container.collision_input.ang_vel_radps *= -1;
  }*/

  // compute linear force from fixed mass of 170g and linear vel change for comparison
  Eigen::Vector3d lin_force_from_vel_delta;
  lin_force_from_vel_delta[0] = lin_vel_delta[0]*0.01 / dt * 0.170; //cm to m, world frame
  lin_force_from_vel_delta[1] = lin_vel_delta[1]*0.01 / dt * 0.170; //cm to m, world frame
  lin_force_from_vel_delta[2] = lin_vel_delta[2]*0.01 / dt * 0.170; //cm to m, world frame


  //GEngine->AddOnScreenDebugMessage(-1, 30.f, FColor::Green, FString(EigenVect3dToString(throw_container.collision_input.lin_force_from_impulses_N).c_str()));
  //GEngine->AddOnScreenDebugMessage(-1, 30.f, FColor::Green, FString(" "));
  Eigen::Vector3d curret_disc_ang_vel = 
  {
    throw_container.current_disc_state.disc_rolling_vel,
    throw_container.current_disc_state.disc_pitching_vel,
    throw_container.current_disc_state.disc_rotation_vel
  };

  std::stringstream ss0;
  ss0 << "ang_vel_from_impulses_Nm = ";
  ss0 << EigenVect3dToString(throw_container.collision_input.ang_vel_from_impulses_Nm);
  log_string(FString(ss0.str().c_str()));

  std::stringstream ss1;
  ss1 << "ang_vel_from_rot_world_vel = ";
  ss1 << EigenVect3dToString(throw_container.collision_input.ang_vel_radps);
  log_string(FString(ss1.str().c_str()));

  std::stringstream ss2;
  ss2 << "ang_vel DfisX k-1 = ";
  ss2 << EigenVect3dToString(curret_disc_ang_vel);
  log_string(FString(ss2.str().c_str()));

  
  // We could choose to reject on_collision impulses here if we think they are buggy Unreal (pun intended) garbage
  // To do this, look at the linear distance travelled, and the kinetci energy and momentum, 
  // and use that to determine the max force we allow
  
  Eigen::Vector3d delta_distance = throw_container.collision_input.lin_pos_m - throw_container.current_disc_state.disc_location;
  //float distance_travelled = delta_distance.norm();

  // 3 axis kinetic energy
  //cwiseProduct for multiplication and cwiseQuotient to divide vectors element-wise
  Eigen::Vector3d kinetic_energy = 
    (throw_container.collision_input.lin_vel_mps.cwiseProduct(throw_container.collision_input.lin_vel_mps)) * 
    throw_container.disc_object.mass * 0.5;

  Eigen::Vector3d kinetic_momentum_N = 
  {
    kinetic_energy[0] / MAX(delta_distance[0], CLOSE_TO_ZERO),
    kinetic_energy[1] / MAX(delta_distance[1], CLOSE_TO_ZERO),
    kinetic_energy[2] / MAX(delta_distance[2], CLOSE_TO_ZERO)
  };
  // add gravity
  Eigen::Vector3d grav_vector_N = {0, 0, GRAV * throw_container.disc_object.mass};
  kinetic_momentum_N += grav_vector_N;

  
  bool skip_input = false;
  // check that we exceed the max force expected
  if(throw_container.collision_input.lin_force_from_impulses_N.norm() > kinetic_momentum_N.norm())
  {
    //GEngine->AddOnScreenDebugMessage(-1, 30.f, FColor::Yellow,(FString("Rejected impulse due to force in excess of our momentum!")));
    //skip_input = true;
    //GEngine->AddOnScreenDebugMessage(-1, 30.f, FColor::Yellow,(FString::SanitizeFloat(kinetic_momentum_N.norm())));
    //GEngine->AddOnScreenDebugMessage(-1, 30.f, FColor::Red,(FString::SanitizeFloat(throw_container.collision_input.lin_force_from_impulses_N.norm())));
    //GEngine->AddOnScreenDebugMessage(-1, 30.f, FColor::Green, FString(" "));
  }
  

  // check for TOO BIG roll and pitch vels
 /* float rp_vel_limit = 50.0;// rad/s
  if(abs(throw_container.collision_input.ang_vel_radps[0]) > rp_vel_limit || abs(throw_container.collision_input.ang_vel_radps[1]) > rp_vel_limit)
  {
    GEngine->AddOnScreenDebugMessage(-1, 30.f, FColor::Yellow,(FString("Rejected update due to bad ang vel!")));
    GEngine->AddOnScreenDebugMessage(-1, 30.f, FColor::Red,   FString(EigenVect3dToString(throw_container.collision_input.ang_vel_radps).c_str()));
    skip_input = true;
  }*/

  if(!DISABLE_COMPLEX_DISC_COLLISION && !skip_input)
  {
    throw_container.collision_input.consumed_input = 0;
  }
  
}



