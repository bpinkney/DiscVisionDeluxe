// Fill out your copyright notice in the Description page of Project Settings.

#include "RangeUserWidget.h"
#include "RangeHUD.h"



URangeUserWidget::URangeUserWidget(const FObjectInitializer& ObjectInitializer) : Super(ObjectInitializer)
{

}

void URangeUserWidget::NativeConstruct()
{
	Super::NativeConstruct();
}

// Takes in params from new Throw, populates the HUD. Intended for initial population.
void URangeUserWidget::PopulateHUD()
{
	ARangeHUD* RangeHUD = Cast<ARangeHUD>(GetWorld()->GetFirstPlayerController()->GetHUD());
	if (IsValid(RangeHUD)) 
	{
		ADiscThrow* latest = nullptr;
		RangeHUD->GetLatestDiscThrow(latest);
		if (IsValid(latest))
		{
			release_stats = latest->get_initial_release_stats();
			cumulative_stats = latest->get_flight_cumulative_stats();



			currentdistance->SetText(FText::FromString(FString::SanitizeFloat(round(cumulative_stats->current_distance))));
			discspeed->SetText(FText::FromString(FString::SanitizeFloat(round(cumulative_stats->current_speed))));
			turnfade->SetText(FText::FromString(FString::SanitizeFloat(round(cumulative_stats->current_turnfade))));
			discspin->SetText(FText::FromString(FString::SanitizeFloat(round(abs(cumulative_stats->current_spin)))));
			//currentwobble->SetText(FText::FromString(FString::SanitizeFloat(cumulative_stats->current_wobble)));



			releasespinpercent->SetText(FText::FromString(FString::SanitizeFloat(round(abs(release_stats->initial_spin_percent)))));
			releasespinrate->SetText(FText::FromString(FString::SanitizeFloat(round(abs(release_stats->initial_spin_rate)))));
			releasediscspeed->SetText(FText::FromString(FString::SanitizeFloat(round(release_stats->initial_speed))));
			releasehyzer->SetText(FText::FromString(FString::SanitizeFloat(round(release_stats->initial_hyzer), 2)));
			releasenoseangle->SetText(FText::FromString(FString::SanitizeFloat(round(release_stats->initial_nose_up), 2)));
			releaseloft->SetText(FText::FromString(FString::SanitizeFloat(round(release_stats->initial_loft), 2)));
			releasedirection->SetText(FText::FromString(FString::SanitizeFloat(release_stats->initial_direction, 2)));
		}

	}
}

// TODO: Takes in params from new Throw, update fields in HUD. Intended to update continuously in Tick
void URangeUserWidget::UpdateHUD()
{
	
}
