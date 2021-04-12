// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "DiscVisionDeluxeUE.h"
#include "Components/SphereComponent.h"
#include "GameFramework/ProjectileMovementComponent.h"		
#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "FollowFlight.h"
#include "DiscProjectile.generated.h"



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
	void SetDiscVelRot(FRotator rotation, FVector velocity, FVector ang_velocity, float spin_position);

	UFUNCTION(BlueprintImplementableEvent, Category="World Action Item")
	void kill_control();
	UFUNCTION(BlueprintImplementableEvent, Category="World Action Item")
	void kill_control_with_delay();

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
