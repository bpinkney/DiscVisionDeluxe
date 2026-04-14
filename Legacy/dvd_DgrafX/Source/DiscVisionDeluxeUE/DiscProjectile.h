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

UENUM(BlueprintType)
enum class enum_pattern_type: uint8
{
   SPIRAL             UMETA(DisplayName = "Spiral Pattern"),
};  

USTRUCT(BlueprintType)
struct FCustom_disc
{

	GENERATED_USTRUCT_BODY();	

	public:

	UPROPERTY(BlueprintReadWrite, EditAnywhere)
	FString 		mold_name		="None";					//for lookup with disc params
	UPROPERTY(BlueprintReadWrite, EditAnywhere)
	enum_disc_form 	mold_form		=enum_disc_form::FRISBEE; 	//see above
	UPROPERTY(BlueprintReadWrite, EditAnywhere)
	float			disc_mass  		=0.175;						//mass in kg
	UPROPERTY(BlueprintReadWrite, EditAnywhere)
	float 			disc_wear		=0.0;   					//0..1
	UPROPERTY(BlueprintReadWrite, EditAnywhere)
	FLinearColor	base_colour		=FLinearColor::Red;			//main colour of plastic
	UPROPERTY(BlueprintReadWrite, EditAnywhere)
	FLinearColor	secondary_colour=FLinearColor::Blue;		//colour of patterning on disc
	UPROPERTY(BlueprintReadWrite, EditAnywhere)
	FLinearColor	rim_colour		=FLinearColor::Black;		//colour of rim
	UPROPERTY(BlueprintReadWrite, EditAnywhere)
	FString			decal_path		="None.png";				//filepath to decal texture
	UPROPERTY(BlueprintReadWrite, EditAnywhere)
	enum_pattern_type pattern_enum = enum_pattern_type::SPIRAL;	//TODO change to pattern enum when implemented
	UPROPERTY(BlueprintReadWrite, EditAnywhere)
	int 			pattern_seed	=0;							//
	UPROPERTY(BlueprintReadWrite, EditAnywhere)
	FString			player_name		="";						//the player who created disc
	UPROPERTY(BlueprintReadWrite, EditAnywhere)
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

	UFUNCTION(BlueprintImplementableEvent, BlueprintCallable, Category="Bag Maker")
	void set_disc_mesh(enum_disc_form disc_static_mesh);

	UFUNCTION(BlueprintImplementableEvent, BlueprintCallable, Category="Bag Maker")
	void set_disc_mass(float mass_in_kg);

	UFUNCTION(BlueprintImplementableEvent, BlueprintCallable, Category="Bag Maker")
	void set_disc_base_colour (FLinearColor set_base_colour);

	UFUNCTION(BlueprintImplementableEvent, BlueprintCallable, Category="Bag Maker")
	void set_disc_rim_colour (FLinearColor set_rim_colour);

	UFUNCTION(BlueprintImplementableEvent, BlueprintCallable, Category="Bag Maker")
	void set_disc_secondary_colour (FLinearColor set_secondary_colour);

	UFUNCTION(BlueprintImplementableEvent, BlueprintCallable, Category="Bag Maker")
	void set_disc_texture(const FString & path_to_texture_file);

	UFUNCTION(BlueprintImplementableEvent, BlueprintCallable, Category="Bag Maker")
	void set_disc_pattern(enum_pattern_type set_pattern_enum, int set_pattern_seed);

	UFUNCTION(BlueprintImplementableEvent, BlueprintCallable, Category="Bag Maker")
	void set_disc_wear(const float set_wear);

	UFUNCTION(BlueprintImplementableEvent, BlueprintCallable, Category="Bag Maker")
	void initialize_custom_disc(const FCustom_disc & custom_disc);


	// local save/load functions (hopefully multiple threads won't make this fight itself...)
	FString custom_disc_get_save_path(void);
	void custom_disc_save(FCustom_disc * custom_disc);
	// generate name string from struct components
	FString custom_disc_generate_name_string(FCustom_disc * custom_disc);
	// load by name string match, true if matching disc was found
	bool custom_disc_load(FString custom_disc_name, FCustom_disc * custom_disc);
	//void custom_disc_load_all(TArray<FCustom_disc> custom_disc_array);
	void custom_disc_load_disc_names(TArray<FString> * custom_disc_names);
	// quick test function
	void custom_disc_test_save_and_load(void);


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
