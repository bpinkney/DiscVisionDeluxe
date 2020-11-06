// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "FollowFlight.generated.h"



	UENUM(BlueprintType)
	enum class enum_ff_display_shape: uint8
	{
 	 Ribbon     		UMETA(DisplayName = "Ribbon"),
 	 Ribbon_Wide      	UMETA(DisplayName = "Wide Ribbon"),
 	 Spiral	   			UMETA(DisplayName = "Spiral"),
 	 Bandsaw			UMETA(DisplayName = "Bandsaw")
	};	


UCLASS()
class DISCVISIONDELUXEUE_API AFollowFlight : public AActor
{
	GENERATED_BODY()
	
public:	
	// Sets default values for this actor's properties
	AFollowFlight();

protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

public:	
	// Called every frame
	virtual void Tick(float DeltaTime) override;


	

	UFUNCTION(BlueprintImplementableEvent, Category = "FollowFlight")
	void log_position ();

	UFUNCTION(BlueprintImplementableEvent, Category = "FollowFlight")
	void select ();

	UFUNCTION(BlueprintImplementableEvent, Category = "FollowFlight")
	void unselect ();

//immediately changes colour of ff to set_colour    0..360
	UFUNCTION(BlueprintImplementableEvent, Category = "FollowFlight")
	void init (float set_hue,enum_ff_display_shape set_shape, bool set_rainbow);

//transitions ff to desired_colour
	UFUNCTION(BlueprintImplementableEvent, Category = "FollowFlight")
	void transition_to_colour (float desired_colour);


	UPROPERTY(EditDefaultsOnly, Category = "ff display")
  	int ff_display_hue;

  	UPROPERTY(EditDefaultsOnly, Category = "ff display")
  	bool ff_is_rainbow;

	




};
