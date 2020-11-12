// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Blueprint/UserWidget.h"
#include "RangeUserWidget.generated.h"

/**
 * 
 */
UCLASS()
class DISCVISIONDELUXEUE_API URangeUserWidget : public UUserWidget
{
	GENERATED_BODY()
	
public:
	UPROPERTY(meta = (BindWidget))
	class UTextBlock* discspeed;
	URangeUserWidget(const FObjectInitializer& ObjectInitializer);
	void PopulateHUD();
	void UpdateHUD();

};
