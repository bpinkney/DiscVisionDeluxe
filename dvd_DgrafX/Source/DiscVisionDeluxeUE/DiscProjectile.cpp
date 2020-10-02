// Fill out your copyright notice in the Description page of Project Settings.


#include "DiscProjectile.h"
//#include "Components/SphereComponent.h"
//#include "GameFramework/ProjectileMovementComponent.h"



// Sets default values
ADiscProjectile::ADiscProjectile()
{
 	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;

	

	

	//Dies after 5 seconds.
	InitialLifeSpan = 15.0f;

	//UPROPERTY(EditDefaultsOnly, BlueprintReadOnly, Category = Curve)
		//UCurveFloat* FlightCurve;
}

// Called when the game starts or when spawned
void ADiscProjectile::BeginPlay()
{
	Super::BeginPlay();

}


// Called every frame
void ADiscProjectile::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);
}

// Function that initializes the projectile's velocity in the shoot direction.
void ADiscProjectile::FireInDirection(const FVector& ShootDirection)
{
	//ProjectileMovementComponent->Velocity = ShootDirection * ProjectileMovementComponent->InitialSpeed;
}
