// Fill out your copyright notice in the Description page of Project Settings.

#include <math.h>  
#include "DiscProjectile.h"
#include "DfisX\DfisX.hpp"
//#include "Components/SphereComponent.h"
//#include "GameFramework/ProjectileMovementComponent.h"

AFollowFlight* fflight;

// Sets default values
ADiscProjectile::ADiscProjectile()
{
 	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;
	
	

	

	//Dies after 5 seconds.
	InitialLifeSpan = 15.0f;
	
	
	
}

// Called when the game starts or when spawned
void ADiscProjectile::BeginPlay()
{
	Super::BeginPlay();


	//followflight spawning
	FActorSpawnParameters SpawnParams;
    SpawnParams.Owner = this;
    SpawnParams.Instigator = GetInstigator();
    UWorld* World = GetWorld();
    FVector thisLocation = FVector (0.0,0.0,0.0);
    FRotator thisRotation = FRotator (0.0,0.0,0.0);
	fflight = World->SpawnActor<AFollowFlight>(FollowFlightBP, thisLocation, thisRotation, SpawnParams);
	



}


// Called every frame
void ADiscProjectile::Tick(float DeltaTime)
{
	
	DfisX::Disc_State disc_state = DfisX::get_disc_state ();
	float xx = disc_state.disc_location[0]*100;
	float yy = disc_state.disc_location[1]*100;
	float zz = disc_state.disc_location[2]*100;
	FVector disc_position = {xx,yy,zz};

	float ii = disc_state.disc_orientation[0];
	float jj = disc_state.disc_orientation[1];
	float kk = disc_state.disc_orientation[2];

	float pitch =-atan(ii/kk)*57.3;
	float yaw =atan(jj/kk)*57.3;
	float roll =0.0;
	



	FVector disc_direction = FVector (disc_state.disc_velocity[0],disc_state.disc_velocity[1],disc_state.disc_velocity[2]);
	
	float ll = disc_state.disc_velocity[0];
	float mm = disc_state.disc_velocity[1];
	float nn = disc_state.disc_velocity[2];

	FVector disc_velocity = {ll,mm,nn};
	
    float disc_spin = -disc_state.disc_rotation/10;

	FRotator disc_rotation = {pitch,roll,yaw};

	

 	

	SetDiscPosRot(disc_position,disc_rotation,disc_velocity,disc_spin);
	fflight->log_position ();
	Super::Tick(DeltaTime);

	//GEngine->AddOnScreenDebugMessage(-1, 30.f, FColor::Orange, FString::Printf(TEXT("Disc Location is: %s"), *disc_position.ToString()));
}

