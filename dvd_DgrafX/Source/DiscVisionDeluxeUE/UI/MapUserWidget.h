// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Blueprint/UserWidget.h"
#include "Runtime/UMG/Public/UMG.h"
#include "../DiscThrow.h"
#include "MapUserWidget.generated.h"

/**
 * 
 */
UCLASS()
class DISCVISIONDELUXEUE_API UMapUserWidget : public UUserWidget
{
	GENERATED_BODY()

public:

	UMapUserWidget(const FObjectInitializer& ObjectInitializer);

	virtual void NativeConstruct() override;

	FVector2D GetGameViewportSize();

	FVector2D GetGameResolution();

	void DrawTopLine();

	UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (BindWidget))
	class UImage* TopDownMap;

	UFUNCTION(BlueprintImplementableEvent, Category="Functions")
	void draw_line_from_disc_throw_ptr(ADiscThrow* ptr_disc_throw);
};
