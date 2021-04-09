// Fill out your copyright notice in the Description page of Project Settings.

#include "DiscProjectile.h"
#include <math.h>
#include "DfisX\DfisX.hpp"
//#include "Components/SphereComponent.h"
//#include "GameFramework/ProjectileMovementComponent.h"



// Sets default values
ADiscProjectile::ADiscProjectile()
{
 	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;

	

	

	//Dies after 10m seconds.
	InitialLifeSpan = 600.0f;
	
	
	
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

	//GEngine->AddOnScreenDebugMessage(-1, 30.f, FColor::Orange, FString::Printf(TEXT("Disc Location is: %s"), *disc_position.ToString()));
}

