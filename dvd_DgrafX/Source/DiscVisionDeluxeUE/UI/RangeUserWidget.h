// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Blueprint/UserWidget.h"
#include "Runtime/UMG/Public/UMG.h"
#include "RangeUserWidget.generated.h"

/**
 * 
 */
UCLASS()
class DISCVISIONDELUXEUE_API URangeUserWidget : public UUserWidget
{
	GENERATED_BODY()
	
public:
	
	URangeUserWidget(const FObjectInitializer& ObjectInitializer);

	virtual void NativeConstruct() override;

	void PopulateHUD(float current_distance, float current_speed, float current_spin, float current_turnfade, float current_wobble);

	void UpdateHUD();

	// UPROPERTY binding, by name, for Range Widget variables. Include all variables to be used here.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (BindWidget))
	class UTextBlock* currentdistance;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (BindWidget))
	class UTextBlock* discspeed;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (BindWidget))
	class UTextBlock* spinpercentage;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (BindWidget))
	class UTextBlock* turn;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (BindWidget))
	class UTextBlock* fade;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (BindWidget))
	class UTextBlock* wobble;

};
