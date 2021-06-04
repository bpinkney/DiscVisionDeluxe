// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "DiscVisionDeluxeUE.h"
#include "Components/SphereComponent.h"
#include "GameFramework/ProjectileMovementComponent.h"		
#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "FollowFlight.h"
#include "DiscProjectile.generated.h"


////required in disc projectile and disc throw
UENUM(BlueprintType)
enum class enum_disc_form: uint8
  {
   PUTTER             UMETA(DisplayName = "Putter"),
   MIDRANGE           UMETA(DisplayName = "Midrange"),
   FAIRWAY            UMETA(DisplayName = "Fairway Driver"),
   DRIVER             UMETA(DisplayName = "Distance Driver"),
   FRISBEE            UMETA(DisplayName = "Fribee")
  };  

UCLASS()
class DISCVISIONDELUXEUE_API ADiscProjectile : public AActor
{
	GENERATED_BODY()
	
public:	
	// Sets default values for this actor's properties
	ADiscProjectile();

protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

public:	
	// Called every frame
	virtual void Tick(float DeltaTime) override;

	UFUNCTION(BlueprintImplementableEvent, Category="World Action Item")
	void SetDiscPosRot(FVector position,FRotator rotation, FVector velocity, float disc_spin);

	UFUNCTION(BlueprintImplementableEvent, Category="World Action Item")
	void SetDiscVelRot(FVector velocity, FVector ang_velocity, FRotator ang_position, float disc_spin_position);


	UFUNCTION(BlueprintImplementableEvent, Category="World Action Item")
	void kill_control();
	UFUNCTION(BlueprintImplementableEvent, Category="World Action Item")
	void kill_control_with_delay();
	UFUNCTION(BlueprintImplementableEvent, Category="World Action Item")
	void end_of_throw_camera();

	UFUNCTION(BlueprintImplementableEvent, Category="World Action Item")
	void set_dither_location();

	UFUNCTION(BlueprintImplementableEvent, Category="World Action Item")
	void set_disc_mesh_and_mass(enum_disc_form disc_static_mesh,float mass_in_kg);

	UFUNCTION(BlueprintImplementableEvent, Category="World Action Item")
	void set_disc_texture(const FString & path_to_texture_file);

	// Sphere collision component.
	UPROPERTY(VisibleDefaultsOnly, Category = Projectile)
	USphereComponent* CollisionComponent;

	// Projectile movement component.
	UPROPERTY(VisibleAnywhere, Category = Movement)
	UProjectileMovementComponent* ProjectileMovementComponent;


	//UPROPERTY()
	//AFollowFlight* followflight;

	UPROPERTY(EditDefaultsOnly, Category = FollowFlight)
	TSubclassOf<class AFollowFlight> FollowFlightBP;

	// Function that initializes the projectile's velocity in the shoot direction.


	

};
