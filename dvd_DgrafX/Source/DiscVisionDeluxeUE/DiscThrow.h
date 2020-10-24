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
	  void new_throw_camera_relative (int disc_mold_enum, FVector thrown_disc_position, float thrown_disc_speed, float thrown_disc_direction, float thrown_disc_loft, float thrown_disc_roll,float thrown_disc_pitch,float thrown_disc_spin_percent, float thrown_disc_wobble);

	UFUNCTION(BlueprintCallable, Category="Disc Throwing")
	  void new_captured_throw(int captured_disc_mold_enum, FVector captured_position, FVector captured_velocity, float captured_world_roll, float captured_world_pitch, float captured_spin_speed, float captured_wobble);

	UFUNCTION(BlueprintCallable, Category="Disc Throwing")
		void new_throw_world_frame ( int disc_mold_enum,FVector thrown_disc_position,FVector v3d_thrown_disc_velocity, float thrown_disc_roll, float thrown_disc_pitch, float thrown_disc_radians_per_second, float thrown_disc_wobble);

	UFUNCTION(BlueprintCallable, Category="Disc Throwing")
		void end_throw_simulation ();

	UFUNCTION(BlueprintCallable, Category="Disc Throwing")
		void spawn_disc_and_follow_flight ();

    UPROPERTY(EditDefaultsOnly, Category = "Blueprints")
        TSubclassOf<class ADiscProjectile> ProjectileClass;

    UPROPERTY(EditDefaultsOnly, Category = "Blueprints")
        TSubclassOf<class AFollowFlight> FollowFlightBP;

    UPROPERTY(EditDefaultsOnly, Category = "Blueprints")
        TSubclassOf<class ADiscCharacter> DiscCharacterBP;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = Gameplay)
      class ADiscCharacter* ptr_disc_character;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = Gameplay)
      class ACameraManager* ptr_camera_manager;

        UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = Gameplay)
      class AFollowFlight* ptr_follow_flight;

        UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = Gameplay)
      class ADiscProjectile* ptr_disc_projectile;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = Gameplay)
      bool is_throw_simulating;

};
