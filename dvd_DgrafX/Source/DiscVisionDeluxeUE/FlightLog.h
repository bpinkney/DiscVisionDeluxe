// Fill out your copyright notice in the Description page of Project Settings.

#pragma once
#include "DiscProjectile.h"
#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "FlightLog.generated.h"

UCLASS()
class DISCVISIONDELUXEUE_API AFlightLog : public AActor
{
	GENERATED_BODY()
	
public:	
	// Sets default values for this actor's properties
	AFlightLog();


protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

public:	
	// Called every frame
	virtual void Tick(float DeltaTime) override;




  	UFUNCTION(BlueprintImplementableEvent, Category="Disc Throwing")
	void log_position(float DeltaTime);


	UFUNCTION(BlueprintImplementableEvent, Category="Disc Throwing")
	void toggle_displayed_mode();



	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = Pointers)
    class ADiscProjectile* ptr_disc_projectile;

};


