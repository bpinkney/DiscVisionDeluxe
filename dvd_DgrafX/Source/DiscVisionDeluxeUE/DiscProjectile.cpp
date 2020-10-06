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

	

	

	//Dies after 5 seconds.
	InitialLifeSpan = 15.0f;
	
	
	
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
	DfisX::Disc_State disc_state = DfisX::get_disc_state ();
	float xx = disc_state.disc_location[0]*100;
	float yy = disc_state.disc_location[1]*100;
	float zz = disc_state.disc_location[2]*100;
	FVector disc_position = {xx,yy,zz};

	float ii = disc_state.disc_orientation[0];
	float jj = disc_state.disc_orientation[1];
	float kk = disc_state.disc_orientation[2];

	float pitch =-atan(ii/kk)*57.3;
	float roll =0;
	float yaw =atan(jj/kk)*57.3;
	
	FRotator disc_rotation = {pitch,roll,yaw};


	SetDiscPosRot(disc_position,disc_rotation);
}
/*
void SetDiscPosRot(FVector posrot)
{
	;
}
*/
// Function that initializes the projectile's velocity in the shoot direction.
void ADiscProjectile::FireInDirection(const FVector& ShootDirection)
{
	//ProjectileMovementComponent->Velocity = ShootDirection * ProjectileMovementComponent->InitialSpeed;

}
