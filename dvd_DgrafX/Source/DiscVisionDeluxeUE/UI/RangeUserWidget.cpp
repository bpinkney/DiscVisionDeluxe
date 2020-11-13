// Fill out your copyright notice in the Description page of Project Settings.


#include "RangeUserWidget.h"


URangeUserWidget::URangeUserWidget(const FObjectInitializer& ObjectInitializer) : Super(ObjectInitializer)
{

}

void URangeUserWidget::NativeConstruct()
{
	Super::NativeConstruct();
}

void URangeUserWidget::PopulateHUD(float current_distance, float current_speed, float current_spin, float current_turnfade, float current_wobble)
{
	currentdistance->SetText(FText::FromString(FString::SanitizeFloat(current_distance)));
	discspeed->SetText(FText::FromString(FString::SanitizeFloat(current_speed)));
	spinpercentage->SetText(FText::FromString(FString::SanitizeFloat(current_spin)));
	turn->SetText(FText::FromString(FString::SanitizeFloat(current_turnfade)));
	wobble->SetText(FText::FromString(FString::SanitizeFloat(current_wobble)));

}

void URangeUserWidget::UpdateHUD()
{
	
}
