// Fill out your copyright notice in the Description page of Project Settings.


#include "DiscProjectile.h"
#include "Components/SphereComponent.h"
#include "GameFramework/ProjectileMovementComponent.h"



// Sets default values
ADiscProjectile::ADiscProjectile()
{
 	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;

	// Use a sphere as a simple collision representation.
	CollisionComponent = CreateDefaultSubobject<USphereComponent>(TEXT("DiscV01"));
	CollisionComponent->BodyInstance.SetCollisionProfileName(TEXT("Projectile"));
	CollisionComponent->OnComponentHit.AddDynamic(this, &ADiscProjectile::OnHit);

	// Set the sphere's collision radius.
	CollisionComponent->InitSphereRadius(15.0f);
	// Set the root component to be the collision component.
	RootComponent = CollisionComponent;

	// Use this component to drive this projectile's movement.
	ProjectileMovementComponent = CreateDefaultSubobject<UProjectileMovementComponent>(TEXT("ProjectileMovementComponent"));
	ProjectileMovementComponent->SetUpdatedComponent(CollisionComponent);
	ProjectileMovementComponent->InitialSpeed = 2000.0f;
	//ProjectileMovementComponent->MaxSpeed = 5000.0f;
	ProjectileMovementComponent->bRotationFollowsVelocity = true;
	ProjectileMovementComponent->bShouldBounce = true;
	ProjectileMovementComponent->Bounciness = 0.1f;

	//Dies after 5 seconds.
	InitialLifeSpan = 5.0f;

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
	GEngine->AddOnScreenDebugMessage(-1, 5.f, FColor::Orange, FString::Printf(TEXT("Disc Location is: %s"), *MuzzleRotation.ToString()));

}

// Function that initializes the projectile's velocity in the shoot direction.
void ADiscProjectile::FireInDirection(const FVector& ShootDirection)
{
	ProjectileMovementComponent->Velocity = ShootDirection * ProjectileMovementComponent->InitialSpeed;
}

// Function that is called when the projectile hits something.
void ADiscProjectile::OnHit(UPrimitiveComponent* HitComponent, AActor* OtherActor, UPrimitiveComponent* OtherComponent, FVector NormalImpulse, const FHitResult& Hit)
{
	if (OtherActor != this && OtherComponent->IsSimulatingPhysics())
	{
		OtherComponent->AddImpulseAtLocation(ProjectileMovementComponent->Velocity * 100.0f, Hit.ImpactPoint);
	}
}