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

protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

public:	
	// Called every frame
	virtual void Tick(float DeltaTime) override;

        UFUNCTION(BlueprintImplementableEvent, Category="World Action Item")
    void DestroyDiscs();

  // Projectile class to spawn.
  UFUNCTION(BlueprintCallable, Category="Disc Throwing")
  void new_throw_camera_relative(
    const int disc_mold_enum, 
    const FVector thrown_disc_position, 
    const float thrown_disc_speed, 
    const float thrown_disc_direction, 
    const float thrown_disc_loft, 
    const float thrown_disc_roll,
    const float thrown_disc_pitch,
    const float thrown_disc_spin_percent, 
    const float thrown_disc_wobble);

  UFUNCTION(BlueprintCallable, Category="Disc Throwing")
  void new_captured_throw(
    const int captured_disc_mold_enum, 
    const FVector captured_position, 
    const FVector captured_velocity, 
    const float captured_world_roll, 
    const float captured_world_pitch, 
    const float captured_spin_speed, 
    const float captured_wobble);

  UFUNCTION(BlueprintCallable, Category="Disc Throwing")
  void new_throw_world_frame(
    const int disc_mold_enum,
    const FVector thrown_disc_position,
    const FVector v3d_thrown_disc_velocity, 
    const float thrown_disc_roll, 
    const float thrown_disc_pitch, 
    const float thrown_disc_radians_per_second, 
    const float thrown_disc_wobble);

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

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = Pointers)
      class ADiscCharacter* ptr_disc_character;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = Pointers)
      class ACameraManager* ptr_camera_manager;

        UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = Pointers)
      class AFollowFlight* ptr_follow_flight;

        UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = Pointers)
      class ADiscProjectile* ptr_disc_projectile;

   // UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = Pointers)
    //  class ADiscProjectile* ptr_HUD_Main_Readout;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = Gameplay)
      bool is_throw_simulating;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = FollowFlight)
      float follow_flight_hue;  //0..360

};
