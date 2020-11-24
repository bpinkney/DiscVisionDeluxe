// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/HUD.h"
#include "Components/WidgetComponent.h"
#include "RangeUserWidget.h"
#include "DebugWidget.h"
#include "../DiscThrow.h"
#include "RangeHUD.generated.h"


/**
 * 
 */
UCLASS()
class DISCVISIONDELUXEUE_API ARangeHUD : public AHUD
{
	GENERATED_BODY()
	
public:
	ARangeHUD();

	virtual void DrawHUD() override;

	virtual void BeginPlay() override;

	virtual void Tick(float DeltaSeconds) override;

	UFUNCTION()
	void PopulateHUD(ADiscThrow* DiscThrowPtr);

	UFUNCTION()
	void UpdateHUD();

	UFUNCTION()
	double GetGenThrow1Input();
	UFUNCTION()
	double GetGenThrow2Input();
	UFUNCTION()
	double GetGenThrow3Input();
	UFUNCTION()
	double GetGenThrow4Input();
	UFUNCTION()
	double GetGenThrow5Input();
	UFUNCTION()
	double GetGenThrow6Input();
	UFUNCTION()
	double GetGenThrow7Input();
	UFUNCTION()
	int32 GetGenThrow8Input();


	UFUNCTION()
	void InitializeDAero();

	UFUNCTION()
	FString GetAero1Text();
	UFUNCTION()
	double GetAero1Input();

	UFUNCTION()
	FString GetAero2Text();
	UFUNCTION()
	double GetAero2Input();

	UFUNCTION()
	FString GetAero3Text();
	UFUNCTION()
	double GetAero3Input();

	UFUNCTION()
	FString GetAero4Text();
	UFUNCTION()
	double GetAero4Input();

	UFUNCTION()
	FString GetAero5Text();
	UFUNCTION()
	double GetAero5Input();

	UFUNCTION()
	FString GetAero6Text();
	UFUNCTION()
	double GetAero6Input();

	UFUNCTION()
	void SetAero1Text(FString aero1);
	UFUNCTION()
	void SetAero1Input(double aero1in);

	UFUNCTION()
	void SetAero2Text(FString aero2);
	UFUNCTION()
	void SetAero2Input(double aero2in);

	UFUNCTION()
	void SetAero3Text(FString aero3);
	UFUNCTION()
	void SetAero3Input(double aero3in);

	UFUNCTION()
	void SetAero4Text(FString aero4);
	UFUNCTION()
	void SetAero4Input(double aero4in);

	UFUNCTION()
	void SetAero5Text(FString aero5);
	UFUNCTION()
	void SetAero5Input(double aero5in);

	UFUNCTION()
	void SetAero6Text(FString aero6);
	UFUNCTION()
	void SetAero6Input(double aero6in);

	// Set UProperties for HUD classes.
	UPROPERTY(EditDefaultsOnly, Category = "Widgets")
	TSubclassOf<UUserWidget> RangeHUDClass;

	UPROPERTY(EditDefaultsOnly, Category = "Widgets")
	TSubclassOf<UUserWidget> RangeDebugClass;

private:
	// Pointers to Widgets to be used.
	URangeUserWidget* RangeUserWidget;

	UDebugWidget* DebugWidget;
};
