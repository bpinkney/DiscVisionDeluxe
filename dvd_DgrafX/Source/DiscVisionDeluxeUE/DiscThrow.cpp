// Fill out your copyright notice in the Description page of Project Settings.


#include "DiscThrow.h"

 //ACameraManager* camera_manager;
 //AThrowInputController* throw_input_controller;

  // Sets default values
ADiscThrow::ADiscThrow()
{
   // Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
  PrimaryActorTick.bCanEverTick = true;
  memset(&throw_container, 0, sizeof(DfisX::Throw_Container));
}

// Called when the game starts or when spawned
void ADiscThrow::BeginPlay()
{
  Super::BeginPlay();

  if (GEngine)
  {
    ptr_disc_character = static_cast<ADiscCharacter*>(this->GetOwner());
    ptr_camera_manager = ptr_disc_character->ptr_camera_manager;
  }
}

// Called every frame
void ADiscThrow::Tick(const float DeltaTime)
{
  Super::Tick(DeltaTime);
  if (is_throw_simulating)
  {
    DfisX::Disc_State disc_state = DfisX::get_disc_state();
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
    

    FVector disc_direction = FVector(disc_state.disc_velocity[0],disc_state.disc_velocity[1],disc_state.disc_velocity[2]);
    
    float ll = disc_state.disc_velocity[0];
    float mm = disc_state.disc_velocity[1];
    float nn = disc_state.disc_velocity[2];

    FVector disc_velocity = {ll,mm,nn};
    
    float disc_spin = -disc_state.disc_rotation/10;

    FRotator disc_rotation = {pitch,roll,yaw};
    
    ptr_disc_projectile->SetDiscPosRot(disc_position,disc_rotation,disc_velocity,disc_spin);
    ptr_follow_flight->log_position();
  }
}


void ADiscThrow::new_throw_camera_relative(
  const int disc_mold_enum, 
  const FVector thrown_disc_position, 
  const float thrown_disc_speed, 
  const float thrown_disc_direction, 
  const float thrown_disc_loft, 
  const float thrown_disc_roll,
  const float thrown_disc_pitch,
  const float thrown_disc_spin_percent, 
  const float thrown_disc_wobble)
{
  spawn_disc_and_follow_flight();
  DfisX::new_throw(
    *throw_container,
    static_cast<DfisX::Disc_Mold_Enum>(disc_mold_enum),
    Eigen::Vector3d(thrown_disc_position.X/100,thrown_disc_position.Y/100,thrown_disc_position.Z/100+1.4),
    thrown_disc_speed,
    thrown_disc_direction,
    thrown_disc_loft,
    thrown_disc_roll,
    thrown_disc_pitch,
    thrown_disc_spin_percent,
    thrown_disc_wobble);
}


///used for captured throws
void ADiscThrow::new_throw_world_frame(
  const int disc_mold_enum,
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
  DfisX::new_throw(
    &throw_container,
    static_cast<DfisX::Disc_Mold_Enum>(disc_mold_enum),
    v3d_thrown_disc_position,
    v3d_thrown_disc_velocity,
    thrown_disc_roll,
    thrown_disc_pitch,
    thrown_disc_radians_per_second,
    thrown_disc_wobble);
}

void ADiscThrow::new_captured_throw(
  const int captured_disc_mold_enum, 
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
  FVector disc_rotation = FVector (captured_world_roll,captured_world_pitch,0); ///this is a vector just for the transform function
  FVector forward_offset = FVector (100,0,0); ///temporarily offset forward so we dont collide with the invisible mesh

  FVector thrown_disc_position = character_location + FTransform(character_rotation).TransformVector(captured_position+forward_offset);
  FVector thrown_disc_velocity = FTransform(character_rotation).TransformVector(captured_velocity);
  FVector thrown_disc_rotation = FTransform(character_rotation).TransformVector(disc_rotation);

  
  float thrown_disc_roll  = thrown_disc_rotation.X;
  float thrown_disc_pitch = thrown_disc_rotation.Y;
  new_throw_world_frame ( captured_disc_mold_enum,thrown_disc_position,thrown_disc_velocity,thrown_disc_roll,thrown_disc_pitch,captured_spin_speed,captured_wobble);
}

void ADiscThrow::spawn_disc_and_follow_flight()
{
  is_throw_simulating = true;
  DestroyDiscs();

  // Get the camera transform.
  FVector forward_offset = FVector (0,0,40);///temp offset to prevent from colliding with invisible character model 
  FVector current_location = forward_offset + ptr_disc_character->GetActorLocation();
  
  UWorld* World = GetWorld();
  FActorSpawnParameters SpawnParams;
  SpawnParams.Owner = this;
  SpawnParams.Instigator = ptr_disc_character;

  ptr_disc_projectile = World->SpawnActor<ADiscProjectile>(ProjectileClass, current_location, FRotator(0,0,0), SpawnParams);
  SpawnParams.Owner = ptr_disc_projectile;
  ptr_follow_flight = World->SpawnActor<AFollowFlight>(FollowFlightBP, FVector(0,0,0), FRotator(0,0,0), SpawnParams);
  ptr_camera_manager->focus_on_disc(ptr_disc_projectile);
  //GEngine->AddOnScreenDebugMessage(-1, 30.f, FColor::Green,(FString::SanitizeFloat(thrown_disc_position.Z)));
  


}

void ADiscThrow::end_throw_simulation ()
{
  is_throw_simulating = false;
}



