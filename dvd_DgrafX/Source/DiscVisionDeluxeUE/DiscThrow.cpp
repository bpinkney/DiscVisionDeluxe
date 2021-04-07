// Fill out your copyright notice in the Description page of Project Settings.


#include "DiscThrow.h"
#include "FollowFlight.h"
#include "dvd_maths.hpp"
#include "Math/Vector.h"
#include "disc_layouts.hpp"

// include for debug stuff
#include "Daero.hpp"

#include "UI/RangeHUD.h"


 //ACameraManager* camera_manager;
 //AThrowInputController* throw_input_controller;
//memset(&latest_disc_throw, 0, sizeof(ADiscThrow::ADiscThrow));
ADiscThrow* ADiscThrow::latest_disc_throw = nullptr;
  // Sets default values
ADiscThrow::ADiscThrow()
{
   // Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.

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
  latest_disc_throw = this;
  ptr_disc_character = static_cast<ADiscCharacter*>(this->GetOwner());
  ptr_camera_manager = ptr_disc_character->ptr_camera_manager;
  follow_flight_hue = 000.0;
  }
}

// Called every frame
void ADiscThrow::Tick(const float DeltaTime)
{
  Super::Tick(DeltaTime);
  if (is_throw_simulating)
  {
    // actually step DfisX
    DfisX::step_simulation(&throw_container, DeltaTime);
    generate_flight_cumulative_stats();
    
    //converting dfisx disc state into unreal usable forms
    DfisX::Disc_State disc_state = DfisX::get_disc_state(&throw_container);
    float xx = disc_state.disc_location[0]*100;
    float yy = disc_state.disc_location[1]*100;
    float zz = disc_state.disc_location[2]*100;
    FVector disc_position = {xx,yy,zz};

    float ii = disc_state.disc_orientation[0];
    float jj = disc_state.disc_orientation[1];
    float kk = disc_state.disc_orientation[2];

    float pitch =-atan(ii/kk)*57.3;
    float yaw =atan(jj/kk)*57.3;
    float roll =0.0;
    

    FVector disc_direction = FVector (disc_state.disc_velocity[0],disc_state.disc_velocity[1],disc_state.disc_velocity[2]);
    
    float ll = disc_state.disc_velocity[0]*100;
    float mm = disc_state.disc_velocity[1]*100;
    float nn = disc_state.disc_velocity[2]*100;

    FVector disc_velocity = {ll,mm,nn};
  
    //float disc_spin = -disc_state.disc_rotation/10;
    //FVector ang_velocity = FVector (0,0,-disc_state.disc_rotation_vel);
    FVector ang_velocity = FVector (0,0,-disc_state.disc_rotation*57.3);

   FRotator disc_rotation = {pitch,roll,yaw};

  
    //ptr_disc_projectile->SetDiscPosRot(disc_position,disc_rotation,disc_velocity,disc_spin_rate);
   ptr_disc_projectile->SetDiscVelRot(disc_rotation,disc_velocity,ang_velocity);
    //finish converting dfisx disc state into unreal usable forms

    //unused sim states for now: SIM_STATE_STOPPED,SIM_STATE_STARTED,SIM_STATE_SKIPPING,SIM_STATE_TREE_HIT,SIM_STATE_ROLLING,SIM_STATE_SLIDING  transition_to_colour
    
    ptr_follow_flight->log_position();
    //end ff stuff
    //GEngine->AddOnScreenDebugMessage(-1, 1.0f, FColor::Red, TEXT("Stimulating!"));
  }
}



void ADiscThrow::GenerateDiscEnv(DfisX::Disc_Env * disc_environment)
{
  // use default for now
  disc_environment->wind_vector_xyz = Eigen::Vector3d(0,0,0);
  disc_environment->gust_factor = DfisX::Gust_Factor::ZERO_DEAD_DIDDLY;
  disc_environment->air_density = ISA_RHO;
}



///convert to world frame here
void ADiscThrow::new_throw_camera_relative(
  const int disc_index, 
  const FVector thrown_disc_position, 
  const float thrown_disc_speed, 
  const float thrown_disc_direction, 
  const float thrown_disc_loft, 
  const float thrown_disc_roll,
  const float thrown_disc_pitch,
  const float thrown_disc_spin_percent, 
  const float thrown_disc_wobble)
{
	/*
  spawn_disc_and_follow_flight();

  // get current wind state (we'll need to update this periodically later as the disc changes environments)
  GenerateDiscEnv(&(throw_container.disc_environment));

  DfisX::new_throw(
    &throw_container,
    static_cast<DiscIndex>(disc_index),
    Eigen::Vector3d(thrown_disc_position.X/100,thrown_disc_position.Y/100,thrown_disc_position.Z/100+1.4),
    thrown_disc_speed,
    thrown_disc_direction,
    thrown_disc_loft,
    thrown_disc_roll,
    thrown_disc_pitch,
    thrown_disc_spin_percent,
    thrown_disc_wobble);
    */
}


///used for captured throws
void ADiscThrow::new_throw_world_frame(
  const int disc_index,
  const FVector thrown_disc_position,
  const FVector thrown_disc_velocity, 
  const float thrown_disc_roll, 
  const float thrown_disc_pitch, 
  const float thrown_disc_radians_per_second, 
  const float thrown_disc_wobble)
{
  spawn_disc_and_follow_flight();

  Eigen::Vector3d v3d_thrown_disc_position = Eigen::Vector3d(thrown_disc_position.X/100,thrown_disc_position.Y/100,thrown_disc_position.Z/100);
  Eigen::Vector3d v3d_thrown_disc_velocity = Eigen::Vector3d(thrown_disc_velocity.X,thrown_disc_velocity.Y,thrown_disc_velocity.Z);
  
  // get current wind state (we'll need to update this periodically later as the disc changes environments)
  GenerateDiscEnv(&(throw_container.disc_environment));

  DfisX::new_throw(
    &throw_container,
    static_cast<DiscIndex>(disc_index),
    v3d_thrown_disc_position,
    v3d_thrown_disc_velocity,
    thrown_disc_roll,
    thrown_disc_pitch,
    thrown_disc_radians_per_second,
    thrown_disc_wobble);



  //TEXT(throw_container.disc_object.mold_name)));

  std::stringstream ss;
  ss << "Threw a ";
  ss << throw_container.disc_object.manufacturer << " ";
  ss << throw_container.disc_object.mold_name;

  FString output_text(ss.str().c_str());
  GEngine->AddOnScreenDebugMessage(-1, 5.0f, FColor::Red, output_text); 





/////////        Initial release Stats          /////////////////////////////

  	//these 3 arent for hud 
    initial_release_stats.initial_direction_vector = FVector (throw_container.current_disc_state.disc_velocity[0],throw_container.current_disc_state.disc_velocity[1],throw_container.current_disc_state.disc_velocity[2]);
    initial_release_stats.initial_location_vector = FVector (throw_container.current_disc_state.disc_location[0],throw_container.current_disc_state.disc_location[1],throw_container.current_disc_state.disc_location[2]);
    initial_release_stats.initial_orientation_vector = FVector (throw_container.current_disc_state.disc_orientation[0],throw_container.current_disc_state.disc_orientation[1],throw_container.current_disc_state.disc_orientation[2]);
	  initial_release_stats.initial_polarity = copysignf(1.0, throw_container.current_disc_state.disc_rotation_vel);

    initial_release_stats.initial_direction_vector.Normalize();
    initial_release_stats.initial_location_vector.Normalize();
    initial_release_stats.initial_orientation_vector.Normalize();
    //end these 3 arent for hud 

    //initial_speed
  	initial_release_stats.initial_speed = throw_container.current_disc_state.disc_velocity.norm();

    //inital_spin_rate
    initial_release_stats.initial_spin_rate = throw_container.current_disc_state.disc_rotation_vel / 6.28 * 60;

  	//initial_spin_percent
  	float theoretical_max_spin_speed = initial_release_stats.initial_speed / throw_container.disc_object.radius;
  	initial_release_stats.initial_spin_percent = throw_container.current_disc_state.disc_rotation_vel / theoretical_max_spin_speed * 100;

  	//initial_direction
  	initial_release_stats.initial_direction = 0;

  	//initial_loft
  	float vertical_speed   = throw_container.current_disc_state.disc_velocity [2];
	  float horizontal_speed = sqrt (throw_container.current_disc_state.disc_velocity [0] * throw_container.current_disc_state.disc_velocity [0] +throw_container.current_disc_state.disc_velocity [1] * throw_container.current_disc_state.disc_velocity [1]);
  	initial_release_stats.initial_loft = 57.3 * atan(vertical_speed/horizontal_speed);

  	//initial_hyzer
    //directions as unit vectors
    initial_release_stats.initial_disc_right_vector = initial_release_stats.initial_direction_vector ^ FVector (0,0,1);
    FVector velocity_up_vector = initial_release_stats.initial_disc_right_vector ^ initial_release_stats.initial_direction_vector;

    //projections of orientation onto the direction vectors
    initial_release_stats.initial_disc_right_vector = (initial_release_stats.initial_orientation_vector | initial_release_stats.initial_disc_right_vector) * initial_release_stats.initial_disc_right_vector;
    velocity_up_vector = (initial_release_stats.initial_orientation_vector | velocity_up_vector) * velocity_up_vector;

  	initial_release_stats.initial_hyzer = atan2 (initial_release_stats.initial_disc_right_vector.Size() , velocity_up_vector.Size()) * 57.3;
    /*
     GEngine->AddOnScreenDebugMessage(-1, 30.f, FColor::Green,(FString::SanitizeFloat(initial_release_stats.initial_orientation_vector[1])));
 GEngine->AddOnScreenDebugMessage(-1, 30.f, FColor::Green,(FString::SanitizeFloat(disc_right_vector.Size())));
  GEngine->AddOnScreenDebugMessage(-1, 30.f, FColor::Green,(FString::SanitizeFloat(velocity_up_vector.Size())));
    GEngine->AddOnScreenDebugMessage(-1, 30.f, FColor::Green,(FString::SanitizeFloat(initial_release_stats.initial_hyzer)));
*/
  	//initial_nose_up
  	initial_release_stats.initial_nose_up = throw_container.current_disc_state.forces_state.aoar * 57.3;

  	//initial_wobble
  	initial_release_stats.initial_wobble = 0;
  	
/////////////     end Initial release Stats            ////////////////////////////////////
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
  GEngine->AddOnScreenDebugMessage(-1, 5.0f, FColor::Red, TEXT("New Disc Throw made with captured data!"));
  //float character_rotation = disc_character->GetActorRotation()->Yaw;
  FRotator character_rotation = ptr_disc_character->GetActorRotation();
  FVector character_location = ptr_disc_character->GetActorLocation();
  FVector disc_rotation = FVector(captured_world_roll,captured_world_pitch,0); ///this is a vector just for the transform function
  FVector forward_offset = FVector(100,0,0); ///temporarily offset forward so we dont collide with the invisible mesh

  FVector thrown_disc_position = character_location + FTransform(character_rotation).TransformVector(captured_position+forward_offset);
  FVector thrown_disc_velocity = FTransform(character_rotation).TransformVector(captured_velocity);
  FVector thrown_disc_rotation = FTransform(character_rotation).TransformVector(disc_rotation);
  
  float thrown_disc_roll  = thrown_disc_rotation.X;
  float thrown_disc_pitch = thrown_disc_rotation.Y;
  new_throw_world_frame(captured_disc_index,thrown_disc_position,thrown_disc_velocity,thrown_disc_roll,thrown_disc_pitch,captured_spin_speed,captured_wobble);
}

void ADiscThrow::spawn_disc_and_follow_flight()
{
  is_throw_simulating = true;
  
  //DestroyDiscs();
    // Get the camera transform.
    FVector forward_offset = FVector (0,0,40);///temp offset to prevent from colliding with invisible character model 
    FVector current_location = forward_offset + ptr_disc_character->GetActorLocation();
    
    UWorld* World = GetWorld();
    FActorSpawnParameters SpawnParams;
    SpawnParams.Owner = this;
    SpawnParams.Instigator = ptr_disc_character;

    ptr_disc_projectile = World->SpawnActor<ADiscProjectile>(ProjectileClass, current_location, FRotator(0,0,0), SpawnParams);

    ptr_camera_manager->focus_on_disc(ptr_disc_projectile);



////Follow flight spawn and init
    SpawnParams.Owner = ptr_disc_projectile;
    ptr_follow_flight = World->SpawnActor<AFollowFlight>(FollowFlightBP, FVector(0,0,0), FRotator(0,0,0), SpawnParams);

	float set_hue = 000.0;
	enum_ff_display_shape set_shape = enum_ff_display_shape::Spiral;//Bandsaw;
	bool set_rainbow = true;

    ptr_follow_flight->init (set_hue,set_shape,set_rainbow);
    ////end Follow flight spawn and init


  //GEngine->AddOnScreenDebugMessage(-1, 30.f, FColor::Green,(FString::SanitizeFloat(thrown_disc_position.Z)));
  
}

void ADiscThrow::end_throw_simulation ()
{
  is_throw_simulating = false;
  ptr_follow_flight->unselect();
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
  	flight_cumulative_stats.current_spin       = throw_container.current_disc_state.disc_rotation_vel/6.28*60;
    

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

  void ADiscThrow::on_collision(
    const FVector disc_position,  //world frame
    const FVector hit_location,   //world frame
    const FVector hit_normal,     //unit direction
    const FVector normal_impulse, //looks si, magnitude and direction
    const FRotator rotation_delta,//FRotator, oof
    const float delta_time)       //si

    {

    const FVector disc_relative_hit_location = hit_location-disc_position; 

      //hit_location is world location in unreal unit (cm)
    //GEngine->AddOnScreenDebugMessage(-1, 10.f, FColor::Green,(hit_location.ToString()));
    //GEngine->AddOnScreenDebugMessage(-1, 10.f, FColor::Green,((hit_location-disc_position).ToString()));
    //GEngine->AddOnScreenDebugMessage(-1, 10.6f, FColor::Green,((normal_impulse).ToString()));

 
    }


