// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "FollowFlight.generated.h"

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
	void set_colour (float set_colour);

//transitions ff to desired_colour
	UFUNCTION(BlueprintImplementableEvent, Category = "FollowFlight")
	void transition_to_colour (float desired_colour);

	



};
