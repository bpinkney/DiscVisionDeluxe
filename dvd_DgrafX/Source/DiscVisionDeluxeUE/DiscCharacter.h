// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "DfisX.hpp"
#include "CoreMinimal.h"
#include "GameFramework/Character.h"
#include "CameraManager.h"
#include "disc_layouts.hpp"

// always last include
#include "DiscCharacter.generated.h"

UCLASS()
class DISCVISIONDELUXEUE_API ADiscCharacter : public ACharacter
{
	GENERATED_BODY()

public:
	// Sets default values for this character's properties
	ADiscCharacter();

	//dvd_DvisEst Interface Function Handlers
	// Call this to create the thread and start it going
	void DvisEstInterface_StartProcess();

	// Call this to print the current state of the thread
	void DvisEstInterface_PrintStuff();

protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

	//dvd_DvisEst Interface Function Handlers
	bool DvisEstInterface_IsComplete() const;

	class DvisEstInterface* dvisEstInterface = nullptr;
	FRunnableThread* DvisEstInterfaceThread = nullptr;

public:	
	// Called every frame
	virtual void Tick(float DeltaTime) override;
	virtual void BeginDestroy() override;
	virtual void Destroyed() override;

	// Called to bind functionality to input
	virtual void SetupPlayerInputComponent(class UInputComponent* PlayerInputComponent) override;

	void PerformThrow(const bool use_default_throw, disc_init_state_t * new_disc_init_state);

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


	// Gun muzzle's offset from the camera location.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = Gameplay)
		FVector MuzzleOffset;

	// Projectile class to spawn.
	UPROPERTY(EditDefaultsOnly, Category = Projectile)
		TSubclassOf<class ADiscProjectile> ProjectileClass;

	UPROPERTY(EditDefaultsOnly, Category = "Camera Manager")
		TSubclassOf<class ACameraManager> CameraManagerBP;
};
