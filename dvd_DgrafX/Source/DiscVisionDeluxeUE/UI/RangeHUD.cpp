// Fill out your copyright notice in the Description page of Project Settings.


#include "RangeHUD.h"

ARangeHUD::ARangeHUD()
{
	
}

void ARangeHUD::SetLatestDiscThrow(ADiscThrow* latest)
{
	this->latest_disc_throw = latest;
}

void ARangeHUD::GetLatestDiscThrow(ADiscThrow* latest)
{
	latest = this->latest_disc_throw;
}

void ARangeHUD::DrawHUD()
{
	Super::DrawHUD();

}

// Check existence of HUD Class, if present, set HUD class to Widget, add to viewport.
void ARangeHUD::BeginPlay()
{
	Super::BeginPlay();
	if (IsValid(this->latest_disc_throw)) 
	{
		memset(this->latest_disc_throw, 0, sizeof(ADiscThrow));
	}

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
			DebugWidget->SetVisibility(ESlateVisibility::Hidden);
		}
	}

		if (MainMenuClass)
	{
		MainMenuWidget = CreateWidget<UMainMenuWidget>(GetWorld(), MainMenuClass);
		if (MainMenuWidget)
		{
			MainMenuWidget->AddToViewport();
			MainMenuWidget->SetVisibility(ESlateVisibility::Hidden);
		}
	}

		if (BagMakerClass)
	{
		BagMakerWidget = CreateWidget<UBagMakerWidget>(GetWorld(), BagMakerClass);
		if (BagMakerWidget)
		{
			BagMakerWidget->AddToViewport();
			BagMakerWidget->SetVisibility(ESlateVisibility::Hidden);
		}
	}

	if (MapDebugClass)
	{
		MapWidget = CreateWidget<UMapUserWidget>(GetWorld(), MapDebugClass);
		if (MapWidget)
		{
			MapWidget->AddToViewport();
		}
	}
}

void ARangeHUD::Tick(float DeltaSeconds)
{
	Super::Tick(DeltaSeconds);
	if (IsValid(this->latest_disc_throw))
	{

		RangeUserWidget->PopulateHUD(this->latest_disc_throw);
		MapWidget->draw_line_from_disc_throw_ptr(this->latest_disc_throw);
		//GEngine->AddOnScreenDebugMessage(-1, 5.0f, FColor::Red, TEXT("New Throw Detect!"));
	}
}

void ARangeHUD::PopulateHUD()
{
	if (RangeUserWidget)
	{
		
	}
}

void ARangeHUD::UpdateHUD()
{
	if (RangeUserWidget)
	{
		RangeUserWidget->UpdateHUD();
	}
}

void ARangeHUD::main_menu_open_btn()
{

	//GEngine->AddOnScreenDebugMessage(-1, 30.f, FColor::Green, FString(" mm open btn"));
			if (MainMenuWidget)
		{
			MainMenuWidget->SetVisibility(ESlateVisibility::Visible);
			//UGameplayStatics::SetGlobalTimeDilation(GetWorld(), 0.0);
			if (IsValid(ptr_disc_character))
			{
				ptr_disc_character->character_was_paused();
			}
		}
}


void ARangeHUD::main_menu_cancel_btn()
{

	//GEngine->AddOnScreenDebugMessage(-1, 30.f, FColor::Green, FString(" mm cancel btn"));
				if (MainMenuWidget)
		{
			MainMenuWidget->SetVisibility(ESlateVisibility::Hidden);
			ptr_disc_character->pause_was_finished();
		}
}

void ARangeHUD::main_menu_next_btn()
{

	GEngine->AddOnScreenDebugMessage(-1, 1.f, FColor::Green, FString(" hud next btn"));
	ptr_disc_character->main_menu_next_btn();
}

void ARangeHUD::main_menu_prev_btn()
{

	GEngine->AddOnScreenDebugMessage(-1, 1.f, FColor::Green, FString(" hud prev btn"));
	ptr_disc_character->main_menu_prev_btn();
}

void ARangeHUD::main_menu_choose_location_btn()
{
	GEngine->AddOnScreenDebugMessage(-1, 1.f, FColor::Green, FString(" hud engage btn"));
	ptr_disc_character->pause_was_finished();
	ptr_disc_character->main_menu_choose_location_btn();
		if (MainMenuWidget)
	{
			MainMenuWidget->RemoveFromViewport();
	}
}

void ARangeHUD::main_menu_bag_maker_btn()
{
	GEngine->AddOnScreenDebugMessage(-1, 1.f, FColor::Green, FString(" bag maker"));
	ptr_disc_character->main_menu_bag_maker_btn();
		if (MainMenuWidget)
	{
			MainMenuWidget->SetVisibility(ESlateVisibility::Hidden);
	}
	BagMakerWidget->SetVisibility(ESlateVisibility::Visible);
}

void ARangeHUD::bag_maker_exit_btn()
{
	GEngine->AddOnScreenDebugMessage(-1, 1.f, FColor::Green, FString("exit bag maker"));
	ptr_disc_character->bag_maker_exit_btn();
	BagMakerWidget->SetVisibility(ESlateVisibility::Hidden);
	MainMenuWidget->SetVisibility(ESlateVisibility::Visible);
}

	
void ARangeHUD::set_minimap_target(int teepad_number)
{
	MapWidget->set_minimap_target(teepad_number);
}

double ARangeHUD::GetGenThrow1Input()
{
	return DebugWidget->GetGenThrow1Input();
}
double ARangeHUD::GetGenThrow2Input()
{
	return DebugWidget->GetGenThrow2Input();
}
double ARangeHUD::GetGenThrow3Input()
{
	return DebugWidget->GetGenThrow3Input();
}
double ARangeHUD::GetGenThrow4Input()
{
	return DebugWidget->GetGenThrow4Input();
}
double ARangeHUD::GetGenThrow5Input()
{
	return DebugWidget->GetGenThrow5Input();
}
double ARangeHUD::GetGenThrow6Input()
{
	return DebugWidget->GetGenThrow6Input();
}
double ARangeHUD::GetGenThrow7Input()
{
	return DebugWidget->GetGenThrow7Input();
}
int32 ARangeHUD::GetGenThrow8Input()
{
	return DebugWidget->GetGenThrow8Input();
}

void ARangeHUD::InitializeDAero()
{
	DebugWidget->InitializeDAero();
}

FString ARangeHUD::GetAero1Text()
{
		return DebugWidget->GetAero1Text();
}
double ARangeHUD::GetAero1Input()
{
		return DebugWidget->GetAero1Input();
}

FString ARangeHUD::GetAero2Text()
{
		return DebugWidget->GetAero2Text();
}
double ARangeHUD::GetAero2Input()
{
		return DebugWidget->GetAero2Input();
}

FString ARangeHUD::GetAero3Text()
{
		return DebugWidget->GetAero3Text();
}
double ARangeHUD::GetAero3Input()
{
		return DebugWidget->GetAero3Input();
}

FString ARangeHUD::GetAero4Text()
{
		return DebugWidget->GetAero4Text();
}
double ARangeHUD::GetAero4Input()
{
		return DebugWidget->GetAero4Input();
}

FString ARangeHUD::GetAero5Text()
{
		return DebugWidget->GetAero5Text();
}
double ARangeHUD::GetAero5Input()
{
		return DebugWidget->GetAero5Input();
}

FString ARangeHUD::GetAero6Text()
{
		return DebugWidget->GetAero6Text();
}
double ARangeHUD::GetAero6Input()
{
		return DebugWidget->GetAero6Input();
}

void ARangeHUD::SetAero1Text(FString aero1)
{
	if (DebugWidget)
	{
		DebugWidget->SetAero1Text(aero1);
	}
}
void ARangeHUD::SetAero1Input(double aero1in)
{
	if (DebugWidget)
	{
		DebugWidget->SetAero1Input(aero1in);
	}
}

void ARangeHUD::SetAero2Text(FString aero2)
{
	if (DebugWidget)
	{
		DebugWidget->SetAero2Text(aero2);
	}
}
void ARangeHUD::SetAero2Input(double aero2in)
{
	if (DebugWidget)
	{
		DebugWidget->SetAero2Input(aero2in);
	}
}

void ARangeHUD::SetAero3Text(FString aero3)
{
	if (DebugWidget)
	{
		DebugWidget->SetAero3Text(aero3);
	}
}
void ARangeHUD::SetAero3Input(double aero3in)
{
	if (DebugWidget)
	{
		DebugWidget->SetAero3Input(aero3in);
	}
}

void ARangeHUD::SetAero4Text(FString aero4)
{
	if (DebugWidget)
	{
		DebugWidget->SetAero4Text(aero4);
	}
}
void ARangeHUD::SetAero4Input(double aero4in)
{
	if (DebugWidget)
	{
		DebugWidget->SetAero4Input(aero4in);
	}
}

void ARangeHUD::SetAero5Text(FString aero5)
{
	if (DebugWidget)
	{
		DebugWidget->SetAero5Text(aero5);
	}
}
void ARangeHUD::SetAero5Input(double aero5in)
{
	if (DebugWidget)
	{
		DebugWidget->SetAero5Input(aero5in);
	}
}

void ARangeHUD::SetAero6Text(FString aero6)
{
	if (DebugWidget)
	{
		DebugWidget->SetAero6Text(aero6);
	}
}
void ARangeHUD::SetAero6Input(double aero6in)
{
	if (DebugWidget)
	{
		DebugWidget->SetAero6Input(aero6in);
	}
}