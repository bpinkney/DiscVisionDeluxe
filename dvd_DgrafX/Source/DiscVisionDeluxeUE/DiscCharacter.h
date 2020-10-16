// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "DfisX.hpp"
#include "CoreMinimal.h"
#include "GameFramework/Character.h"
#include "CameraManager.h"


// always last include
#include "DiscCharacter.generated.h"

UCLASS()
class DISCVISIONDELUXEUE_API ADiscCharacter : public ACharacter
{
	GENERATED_BODY()

public:
	// Sets default values for this character's properties
	ADiscCharacter();


protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;



public:	
	// Called every frame
	virtual void Tick(float DeltaTime) override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;
	virtual void BeginDestroy() override;
	virtual void Destroyed() override;

	/*
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;
	virtual void BeginDestroy() override;
	virtual void Destroyed() override;
	*/

	// Called to bind functionality to input
	virtual void SetupPlayerInputComponent(class UInputComponent* PlayerInputComponent) override;



	// Handles input for moving forward and backward.
	UFUNCTION()
		void MoveForward(float Value);

	// Handles input for moving right and left.
	UFUNCTION()
		void MoveRight(float Value);

	// Function that handles firing projectiles.
	UFUNCTION()
	  void Fire();

	UFUNCTION()
	  void Quit();

	UFUNCTION()
	  void Action1();

	UFUNCTION()
	  void Action2();

	UFUNCTION()
	  void Action3();

	UFUNCTION()
	  void Action4();

	UFUNCTION(BlueprintImplementableEvent, Category="World Action Item")
	void DestroyDiscs();

	UFUNCTION(BlueprintCallable, Category="Disc Throwing")
	  void new_throw_camera_realtive (int disc_mold_enum, FVector thrown_disc_position, float thrown_disc_speed, float thrown_disc_direction, float thrown_disc_loft, float thrown_disc_roll,float thrown_disc_pitch,float thrown_disc_spin_percent, float thrown_disc_wobble);

	UFUNCTION(BlueprintImplementableEvent, Category="Disc Throwing")
	  void new_captured_throw(int captured_disc_mold_enum, FVector captured_position, FVector captured_velocity, float captured_world_roll, float captured_world_pitch, float captured_spin_speed, float captured_wobble);

	UFUNCTION(BlueprintCallable, Category="Disc Throwing")
		void new_throw_world_frame ( int disc_mold_enum,FVector thrown_disc_position,FVector v3d_thrown_disc_velocity, float thrown_disc_roll, float thrown_disc_pitch, float thrown_disc_radians_per_second, float thrown_disc_wobble);


	// Gun muzzle's offset from the camera location.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = Gameplay)
		FVector MuzzleOffset;

	// Projectile class to spawn.
	UPROPERTY(EditDefaultsOnly, Category = "Disc Throwing")
		TSubclassOf<class ADiscProjectile> ProjectileClass;

	UPROPERTY(EditDefaultsOnly, Category = "Camera Manager")
		TSubclassOf<class ACameraManager> CameraManagerBP;

	UPROPERTY(EditDefaultsOnly, Category = "Throw Input")
		TSubclassOf<class AThrowInputController> ThrowInputControllerBP;
};
