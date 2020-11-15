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