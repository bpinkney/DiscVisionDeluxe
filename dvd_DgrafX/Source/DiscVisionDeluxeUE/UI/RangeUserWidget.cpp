// Fill out your copyright notice in the Description page of Project Settings.


#include "RangeUserWidget.h"


URangeUserWidget::URangeUserWidget(const FObjectInitializer& ObjectInitializer) : Super(ObjectInitializer)
{

}

void URangeUserWidget::NativeConstruct()
{
	Super::NativeConstruct();
}

// Takes in params from new Throw, populates the HUD. Intended for initial population.
void URangeUserWidget::PopulateHUD(ADiscThrow* DiscThrowPtr)
{
	ADiscThrow::Initial_Release_Stats release_stats;
	ADiscThrow::Flight_Cumulative_Stats cumulative_stats;
	DiscThrowPtr->get_initial_release_stats(&release_stats);
	DiscThrowPtr->get_flight_cumulative_stats(&cumulative_stats);

	currentdistance->SetText(FText::FromString(FString::SanitizeFloat(cumulative_stats.current_distance)));
	//discspeed->SetText(FText::FromString(FString::SanitizeFloat()));
	spinpercentage->SetText(FText::FromString(FString::SanitizeFloat(release_stats.initial_spin_percent)));
	//turn->SetText(FText::FromString(FString::SanitizeFloat()));
	//wobble->SetText(FText::FromString(FString::SanitizeFloat()));

}

// TODO: Takes in params from new Throw, update fields in HUD. Intended to update continuously in Tick
void URangeUserWidget::UpdateHUD()
{
	
}
