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

struct Custom_disc
{
	FString 		mold_name		="None";					//for lookup with disc params
	enum_disc_form 	mold_form		=enum_disc_form::FRISBEE; 	//see above
	float			disc_mass  		=0.175;						//mass in kg
	float 			disc_wear		=0.0;   					//0..1
	FColor			base_colour		=FColor::Red;				//main colour of plastic
	FColor			secondary_colour=FColor::Blue;				//colour of patterning on disc
	FColor			rim_colour		=FColor::Black;				//colour of rim
	FString			decal_path		="None.png";				//filepath to decal texture
	int 			pattern_enum	=1;							//TODO change to pattern enum when implemented
	FString			player_name		="";						//the player who created disc
	FDateTime		date_created; 								//date of creation    

	// player name = "bob" and date_created = defined     a saved custom disc
	// player name = "bob" and date_created = undefined   a bagged custom disc    there could still be a saved verison
	// player name = "" and date_created = undefined      a generated disc
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

	UFUNCTION(BlueprintImplementableEvent, Category="Bag Maker")
	void set_disc_mesh(enum_disc_form disc_static_mesh);

	UFUNCTION(BlueprintImplementableEvent, Category="Bag Maker")
	void set_disc_mass(float mass_in_kg);

	UFUNCTION(BlueprintImplementableEvent, Category="Bag Maker")
	void set_disc_base_colour (FColor set_base_colour);

	UFUNCTION(BlueprintImplementableEvent, Category="Bag Maker")
	void set_disc_rim_colour (FColor set_rim_colour);

	UFUNCTION(BlueprintImplementableEvent, Category="Bag Maker")
	void set_disc_secondary_colour (FColor set_secondary_colour);

	UFUNCTION(BlueprintImplementableEvent, Category="Bag Maker")
	void set_disc_texture(const FString & path_to_texture_file);

	UFUNCTION(BlueprintImplementableEvent, Category="Bag Maker")
	void set_disc_pattern(int set_pattern_enum);

	void initialize_custom_disc(const Custom_disc & custom_disc);

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
