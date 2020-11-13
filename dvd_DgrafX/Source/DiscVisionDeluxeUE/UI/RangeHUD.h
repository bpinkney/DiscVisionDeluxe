// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/HUD.h"
#include "Components/WidgetComponent.h"
#include "RangeUserWidget.h"
#include "RangeHUD.generated.h"


/**
 * 
 */
UCLASS()
class DISCVISIONDELUXEUE_API ARangeHUD : public AHUD
{
	GENERATED_BODY()
	
public:
	ARangeHUD();

	virtual void DrawHUD() override;

	virtual void BeginPlay() override;

	virtual void Tick(float DeltaSeconds) override;

	UFUNCTION()
	void PopulateHUD(float current_distance, float current_speed, float current_spin, float current_turnfade, float current_wobble);

	UFUNCTION()
	void UpdateHUD();

	UPROPERTY(EditDefaultsOnly, Category = "Widgets")
	TSubclassOf<UUserWidget> RangeHUDClass;

private:
	URangeUserWidget* RangeUserWidget;
};
