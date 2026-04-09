// Fill out your copyright notice in the Description page of Project Settings.


#include "FollowFlight.h"

// Sets default values
AFollowFlight::AFollowFlight()
{
 	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = false;

}

// Called when the game starts or when spawned
void AFollowFlight::BeginPlay()
{
	Super::BeginPlay();
	
}

// Called every frame
void AFollowFlight::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

}



