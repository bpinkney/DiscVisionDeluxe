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


	// Gun muzzle's offset from the camera location.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = Gameplay)
		FVector MuzzleOffset;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = Gameplay)
      class ACameraManager* ptr_camera_manager;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = Gameplay)
      class AThrowInputController* ptr_throw_input_controller;

	// Projectile class to spawn.
	UPROPERTY(EditDefaultsOnly, Category = "Disc Throwing")
		TSubclassOf<class ADiscProjectile> ProjectileClass;

	UPROPERTY(EditDefaultsOnly, Category = "Camera Manager")
		TSubclassOf<class ACameraManager> CameraManagerBP;

	UPROPERTY(EditDefaultsOnly, Category = "Throw Input")
		TSubclassOf<class AThrowInputController> ThrowInputControllerBP;
};
