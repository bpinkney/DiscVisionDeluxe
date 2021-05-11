// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "DfisX.hpp"
#include "CoreMinimal.h"
#include "GameFramework/Character.h"
#include "CameraManager.h"


// always last include
#include "DiscCharacter.generated.h"

UENUM(BlueprintType)
enum class enum_character_state: uint8
{
 	PAUSED    		    UMETA(DisplayName = "Paused"),
 	READY_TO_THROW      UMETA(DisplayName = "Ready to throw!"),
 	SIMULATING	   	    UMETA(DisplayName = "Simulating...")
};	

UCLASS()
class DISCVISIONDELUXEUE_API ADiscCharacter : public ACharacter
{
	GENERATED_BODY()

public:
	// Sets default values for this character's properties
	ADiscCharacter();
	FTransform set_location;
	int teepad_locations_iterator;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Editor Set Pieces")
	TArray<FTransform> teepad_locations;

	enum_character_state FSM_character_state;
	enum_character_state FSM_held_state;
	FString context_string;
	FString location_name_string;


protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;




public:	
	// Called every frame
	virtual void Tick(float DeltaTime) override;

	// Called to bind functionality to input
	virtual void SetupPlayerInputComponent(class UInputComponent* PlayerInputComponent) override;

	UFUNCTION(BlueprintCallable, Category="Buttons")
	void main_menu_next_btn();

	UFUNCTION(BlueprintCallable, Category="Buttons")
	void main_menu_prev_btn();

	UFUNCTION(BlueprintCallable, Category="Buttons")
	void main_menu_choose_location_btn();



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




///////////////////////////State Machine Stuff//////////////////////////////


////state machine functions
	UFUNCTION(BlueprintCallable, Category="FSM getter setters")
	 FString get_state_display_string();

///state machine transitions
	UFUNCTION(BlueprintCallable, Category="FSM transitions")
	  void disc_was_thrown();
	UFUNCTION(BlueprintCallable, Category="FSM transitions")
	  void throw_was_finished();

	UFUNCTION(BlueprintCallable, Category="FSM transitions")
	  void character_was_paused();
	UFUNCTION(BlueprintCallable, Category="FSM transitions")
	  void pause_was_finished();

///state machine getters
	UFUNCTION(BlueprintCallable, Category="FSM getter setters")
	  bool get_is_ready_to_throw();

	UFUNCTION(BlueprintCallable, Category="FSM getter setters")
	  bool get_is_throw_simulating();

	UFUNCTION(BlueprintCallable, Category="FSM getter setters")
	  bool get_is_paused();

};
