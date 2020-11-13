// Fill out your copyright notice in the Description page of Project Settings.


#include "RangeHUD.h"

ARangeHUD::ARangeHUD()
{

}

void ARangeHUD::DrawHUD()
{
	Super::DrawHUD();
}

void ARangeHUD::BeginPlay()
{
	Super::BeginPlay();

	if (RangeHUDClass)
	{
		RangeUserWidget = CreateWidget<URangeUserWidget>(GetWorld(), RangeHUDClass);
		if (RangeUserWidget)
		{
			RangeUserWidget->AddToViewport();
		}
	}

	if (RangeDebugClass)
	{
		DebugWidget = CreateWidget<UDebugWidget>(GetWorld(), RangeDebugClass);
		if (DebugWidget)
		{
			DebugWidget->AddToViewport();
		}
	}
}

void ARangeHUD::Tick(float DeltaSeconds)
{
	Super::Tick(DeltaSeconds);
}

void ARangeHUD::PopulateHUD(float current_distance, float current_speed, float current_spin, float current_turnfade, float current_wobble)
{
	if (RangeUserWidget)
	{
		RangeUserWidget->PopulateHUD(current_distance, current_speed, current_spin, current_turnfade, current_wobble);
	}
}

void ARangeHUD::UpdateHUD()
{
	if (RangeUserWidget)
	{
		RangeUserWidget->UpdateHUD();
	}
}