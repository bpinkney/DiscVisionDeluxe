// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include <cmath>
#include "CoreMinimal.h"
#include "Blueprint/UserWidget.h"
#include "Runtime/UMG/Public/UMG.h"
#include "../DiscThrow.h"

#include "RangeUserWidget.generated.h"



/**
 * 
 */
UCLASS()
class DISCVISIONDELUXEUE_API URangeUserWidget : public UUserWidget
{
	GENERATED_BODY()
	
public:

	ADiscThrow::Initial_Release_Stats* release_stats;
	ADiscThrow::Flight_Cumulative_Stats* cumulative_stats;
	
	URangeUserWidget(const FObjectInitializer& ObjectInitializer);

	virtual void NativeConstruct() override;

	void PopulateHUD(ADiscThrow* ptr_disc_throw);

	void UpdateHUD();

	// UPROPERTY binding, by name, for Range Widget variables. Include all variables to be used here.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (BindWidget))
	class UTextBlock* currentdistance;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (BindWidget))
	class UTextBlock* discspeed;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (BindWidget))
	class UTextBlock* releasediscspeed;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (BindWidget))
	class UTextBlock* turnfade;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (BindWidget))
	class UTextBlock* discspin;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (BindWidget))
	class UTextBlock* currentwobble;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (BindWidget))
	class UTextBlock* releasewobble;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (BindWidget))
	class UTextBlock* releasespinpercent;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (BindWidget))
	class UTextBlock* releasespinrate;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (BindWidget))
	class UTextBlock* releasehyzer;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (BindWidget))
	class UTextBlock* releaseloft;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (BindWidget))
	class UTextBlock* releasenoseangle;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (BindWidget))
	class UTextBlock* releasedirection;

};
