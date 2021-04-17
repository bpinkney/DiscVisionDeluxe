// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "Runtime/Engine/Classes/GameFramework/PlayerController.h"
#include "CameraManager.generated.h"

/**
 * 
 */
UCLASS()
class DISCVISIONDELUXEUE_API ACameraManager : public APlayerCameraManager
{
	GENERATED_BODY()


	public:	

	UFUNCTION(BlueprintImplementableEvent, Category="Camera Management")
	void focus_on_player();

	UFUNCTION(BlueprintImplementableEvent, Category="Camera Management")
	void set_player_target(APlayerController* player_target);

	UFUNCTION(BlueprintImplementableEvent, Category="Camera Management")
	void focus_on_disc(AActor* new_disc_target);

	UFUNCTION(BlueprintImplementableEvent, Category="Camera Management")
	void focus_on_teepad(int teepad_id);

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category ="Camera Management")
	AActor* disc_target;
	
};
