// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "DfisX.hpp"
#include "DiscCharacter.h"
#include "CameraManager.h"
#include "DiscProjectile.h"



#include "GameFramework/Actor.h"
#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "DiscThrow.generated.h"


UCLASS()
class DISCVISIONDELUXEUE_API ADiscThrow : public AActor
{
  GENERATED_BODY()
  
public:  
  // Sets default values for this actor's properties
  ADiscThrow();
  
  struct Initial_Release_Stats
  {
  	FVector initial_direction_vector; ////maybe not needed for hud
  	FVector initial_location_vector; ////maybe not needed for hud
    FVector initial_orientation_vector; ////maybe not needed for hud
    FVector initial_disc_right_vector; ////maybe not needed for hud
    float initial_polarity;          ////maybe not needed for hud

  	float initial_speed;
  	float initial_spin_percent;
    float initial_spin_rate;
  	float initial_direction;
  	float initial_loft;
  	float initial_hyzer;
  	float initial_nose_up;
  	float initial_wobble;
  };
  struct Flight_Cumulative_Stats
  {
  	float current_distance;
  	float current_speed;
  	float current_spin;
  	float current_turnfade; 
  	float current_wobble;
  };
  DfisX::Throw_Container throw_container;
  
  UFUNCTION(BlueprintImplementableEvent,BlueprintCallable, Category="Disc Throwing")
  void log_string(const FString& stringlog);

  UFUNCTION(BlueprintImplementableEvent,BlueprintCallable, Category="Disc Throwing")
  void log_hit(bool issa_hit);

  UFUNCTION(BlueprintImplementableEvent, Category="Disc Throwing")
  void log_launch(bool issa_launch);

  UFUNCTION(BlueprintImplementableEvent, Category="Disc Throwing")
  void dump_string_log();

protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

  


public:	

  Initial_Release_Stats initial_release_stats;
  Flight_Cumulative_Stats flight_cumulative_stats;
  
   Initial_Release_Stats* get_initial_release_stats();
   Flight_Cumulative_Stats* get_flight_cumulative_stats();
   void generate_flight_cumulative_stats();

	// Called every frame
	virtual void Tick(float DeltaTime) override;

  UFUNCTION(BlueprintImplementableEvent, Category="World Action Item")
  void DestroyDiscs();

  void GenerateDiscEnv(DfisX::Disc_Env * disc_environment);

  // Projectile class to spawn.


  UFUNCTION(BlueprintCallable, Category="Disc Throwing")
  void new_captured_throw(
    const int captured_disc_index, 
    const FVector captured_position, 
    const FVector captured_velocity, 
    const float captured_world_roll, 
    const float captured_world_pitch, 
    const float captured_spin_speed, 
    const float captured_wobble);

  UFUNCTION(BlueprintCallable, Category="Disc Throwing")
  void new_throw_world_frame(
    const int disc_index,
    const FVector thrown_disc_position,
    const FVector v3d_thrown_disc_velocity, 
    const float thrown_disc_roll, 
    const float thrown_disc_pitch, 
    const float thrown_disc_radians_per_second, 
    const float thrown_disc_wobble);

  UFUNCTION(BlueprintCallable, Category="Disc Throwing")
  void on_collision(
    const FVector disc_position,
    const FVector disc_rotation,
    const TArray<FVector> hit_location,
    const TArray<FVector> hit_normal,
    const TArray<FVector> normal_impulse,
    const FVector lin_vel,
    const FVector lin_vel_delta,
    const FVector ang_vel,
    const FVector world_ang_vel,
    const FVector world_ang_vel_delta,
    const FVector ang_vel_delta,
    const int total_hit_events,
    const float delta_time,
    const TArray<float> hit_friction,
    const TArray<float> hit_restitution
    );

  UFUNCTION(BlueprintCallable, Category="Disc Throwing")
  void end_throw_simulation();

  UFUNCTION(BlueprintCallable, Category="Disc Throwing")
  void spawn_disc_and_follow_flight();

  UPROPERTY(EditDefaultsOnly, Category = "Blueprints")
  TSubclassOf<class ADiscProjectile> ProjectileClass;

  UPROPERTY(EditDefaultsOnly, Category = "Blueprints")
  TSubclassOf<class AFollowFlight> FollowFlightBP;

  UPROPERTY(EditDefaultsOnly, Category = "Blueprints")
  TSubclassOf<class ADiscCharacter> DiscCharacterBP;

  UPROPERTY(EditDefaultsOnly, Category = "Blueprints")
  TSubclassOf<class AFlightLog> FlightLogBP;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = Pointers)
    class ADiscCharacter* ptr_disc_character;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = Pointers)
    class ACameraManager* ptr_camera_manager;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = Pointers)
    class AFollowFlight* ptr_follow_flight;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = Pointers)
    class ADiscProjectile* ptr_disc_projectile;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = Pointers)
    class AFlightLog* ptr_flight_log;

   // UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = Pointers)
    //  class ADiscProjectile* ptr_HUD_Main_Readout;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = Gameplay)
      bool is_throw_simulating;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = FollowFlight)
      float follow_flight_hue;  //0..360

};
