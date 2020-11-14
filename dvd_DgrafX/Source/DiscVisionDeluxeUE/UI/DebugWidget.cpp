// Fill out your copyright notice in the Description page of Project Settings.


#include "DebugWidget.h"

double daero_debug_input1 = 0;
double daero_debug_input2 = 0;
double daero_debug_input3 = 0;
double daero_debug_input4 = 0;
double daero_debug_input5 = 0;
double daero_debug_input6 = 0;

FString daero_debug_text1 = "";
FString daero_debug_text2 = "";
FString daero_debug_text3 = "";
FString daero_debug_text4 = "";
FString daero_debug_text5 = "";
FString daero_debug_text6 = "";

UDebugWidget::UDebugWidget(const FObjectInitializer& ObjectInitializer) : Super(ObjectInitializer)
{
	
}

void UDebugWidget::InitializeDAero()
{
	DebugAero1->SetText(FText::FromString(FString::SanitizeFloat(daero_debug_input1)));
	DebugAero2->SetText(FText::FromString(FString::SanitizeFloat(daero_debug_input2)));
	DebugAero3->SetText(FText::FromString(FString::SanitizeFloat(daero_debug_input3)));
	DebugAero4->SetText(FText::FromString(FString::SanitizeFloat(daero_debug_input4)));
	DebugAero5->SetText(FText::FromString(FString::SanitizeFloat(daero_debug_input5)));
	DebugAero6->SetText(FText::FromString(FString::SanitizeFloat(daero_debug_input6)));

	DebugAero1Text->SetText(FText::FromString(daero_debug_text1));
	DebugAero2Text->SetText(FText::FromString(daero_debug_text2));
	DebugAero3Text->SetText(FText::FromString(daero_debug_text3));
	DebugAero4Text->SetText(FText::FromString(daero_debug_text4));
	DebugAero5Text->SetText(FText::FromString(daero_debug_text5));
	DebugAero6Text->SetText(FText::FromString(daero_debug_text6));
}

void UDebugWidget::NativeConstruct()
{
	Super::NativeConstruct();
}

FString UDebugWidget::GetAero1Text()
{
	return daero_debug_text1;
}
double UDebugWidget::GetAero1Input()
{
	return daero_debug_input1;
}
FString UDebugWidget::GetAero2Text()
{
	return daero_debug_text2;
}
double UDebugWidget::GetAero2Input()
{
	return daero_debug_input2;
}
FString UDebugWidget::GetAero3Text()
{
	return daero_debug_text3;
}
double UDebugWidget::GetAero3Input()
{
	return daero_debug_input3;
}
FString UDebugWidget::GetAero4Text()
{
	return daero_debug_text4;
}
double UDebugWidget::GetAero4Input()
{
	return daero_debug_input4;
}
FString UDebugWidget::GetAero5Text()
{
	return daero_debug_text5;
}
double UDebugWidget::GetAero5Input()
{
	return daero_debug_input5;
}
FString UDebugWidget::GetAero6Text()
{
	return daero_debug_text6;
}
double UDebugWidget::GetAero6Input()
{
	return daero_debug_input6;
}

void UDebugWidget::SetAero1Text(FString aero1)
{
	daero_debug_text1 = aero1;
}
void UDebugWidget::SetAero1Input(double aero1in)
{
	daero_debug_input1 = aero1in;
}
void UDebugWidget::SetAero2Text(FString aero2)
{
	daero_debug_text2 = aero2;
}
void UDebugWidget::SetAero2Input(double aero2in)
{
	daero_debug_input2 = aero2in;
}
void UDebugWidget::SetAero3Text(FString aero3)
{
	daero_debug_text3 = aero3;
}
void UDebugWidget::SetAero3Input(double aero3in)
{
	daero_debug_input3 = aero3in;
}
void UDebugWidget::SetAero4Text(FString aero4)
{
	daero_debug_text4 = aero4;
}
void UDebugWidget::SetAero4Input(double aero4in)
{
	daero_debug_input4 = aero4in;
}
void UDebugWidget::SetAero5Text(FString aero5)
{
	daero_debug_text5 = aero5;
}
void UDebugWidget::SetAero5Input(double aero5in)
{
	daero_debug_input5 = aero5in;
}
void UDebugWidget::SetAero6Text(FString aero6)
{
	daero_debug_text6 = aero6;
}
void UDebugWidget::SetAero6Input(double aero6in)
{
	daero_debug_input6 = aero6in;
}