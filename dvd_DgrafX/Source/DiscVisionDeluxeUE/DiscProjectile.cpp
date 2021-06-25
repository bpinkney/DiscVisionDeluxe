// Fill out your copyright notice in the Description page of Project Settings.

#include "DiscProjectile.h"
#include <math.h>
//#include "DfisX\DfisX.hpp"
//#include "Components/SphereComponent.h"
//#include "GameFramework/ProjectileMovementComponent.h"



// Sets default values
ADiscProjectile::ADiscProjectile()
{
 	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;

	//kill_control_with_delay();

	

	//Dies after 10m seconds.
	InitialLifeSpan = 600.0f;
	
	
	
}

// Called when the game starts or when spawned
void ADiscProjectile::BeginPlay()
{
	Super::BeginPlay();

 //kill_control_with_delay();

}
/*
void ADiscProjectile::initialize_custom_disc(const FCustom_disc & custom_disc)
{
	/*
	FString 		mold_name		="None";					//for lookup with disc params
	enum_disc_form 	mold_form		=enum_disc_form::FRISBEE; 	//see above
	float			disc_mass  		=0.175						//mass in kg
	float 			disc_wear		=0.0;   					//0..1
	FColor			base_colour		=FColor::Red;				//main colour of plastic
	FColor			secondary_colour=FColor::Blue;				//colour of patterning on disc
	FColor			rim_colour		=FColor::Black;				//colour of rim
	FString			decal_path		="None.png";				//filepath to decal texture
	int 			pattern_enum	=1;							//TODO change to pattern enum when implemented
	FString			player_name     ="";					//the player who created disc
	FDateTime		date_created; 								//date of creation
	
	if (custom_disc.player_name=="")   //not form a player bag ; this would be if this disc is initialized from a throw generator or somesuch
	{



	} else  // normal case
	{

	set_disc_mesh_and_mass		(custom_disc.mold_form,custom_disc.disc_mass);
	set_disc_wear				(custom_disc.disc_wear);
	set_disc_base_colour 		(custom_disc.base_colour);
	set_disc_secondary_colour 	(custom_disc.secondary_colour);
	set_disc_rim_colour  		(custom_disc.rim_colour);
	set_disc_texture			(custom_disc.decal_path);
	set_disc_pattern			(custom_disc.pattern_enum,custom_disc.pattern_seed);
	//set_disc_pattern (TODO)

	}

}
*/
// Called every frame
void ADiscProjectile::Tick(float DeltaTime)
{
	
	
	Super::Tick(DeltaTime);

	//GEngine->AddOnScreenDebugMessage(-1, 30.f, FColor::Orange, FString::Printf(TEXT("Disc Location is: %s"), *disc_position.ToString()));
}



