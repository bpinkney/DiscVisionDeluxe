// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "disc_layouts.hpp"
#include "ThrowInputController.generated.h"

UCLASS()
class DISCVISIONDELUXEUE_API AThrowInputController : public AActor
{
	GENERATED_BODY()
	
public:	
	// Sets default values for this actor's properties
	AThrowInputController();


	//dvd_DvisEst Interface Function Handlers
	// Call this to create the thread and start it going
	void DvisEstInterface_StartProcess();

	// Call this to print the current state of the thread
	void DvisEstInterface_PrintStuff();

protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

    //dvd_DvisEst Interface Function Handlers
    bool DvisEstInterface_IsComplete() const;

	class DvisEstInterface* dvisEstInterface = nullptr;
	FRunnableThread* DvisEstInterfaceThread = nullptr;

public:	
	// Called every frame
	virtual void Tick(float DeltaTime) override;


    void PerformCapturedThrow(disc_init_state_t * new_disc_init_state);

    UPROPERTY(EditDefaultsOnly, Category = "Blueprints")
    	TSubclassOf<class ADiscThrow> DiscThrowBP;
};
