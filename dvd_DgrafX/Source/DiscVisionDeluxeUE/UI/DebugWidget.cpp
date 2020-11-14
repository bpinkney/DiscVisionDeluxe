// Fill out your copyright notice in the Description page of Project Settings.


#include "DebugWidget.h"

UDebugWidget::UDebugWidget(const FObjectInitializer& ObjectInitializer) : Super(ObjectInitializer)
{
	
}

void UDebugWidget::InitializeDAero()
{
	DebugAero1->SetText(FText::FromString(FString::SanitizeFloat(gv_aero_debug0)));
	DebugAero2->SetText(FText::FromString(FString::SanitizeFloat(gv_aero_debug1)));
	DebugAero3->SetText(FText::FromString(FString::SanitizeFloat(gv_aero_debug2)));
	DebugAero4->SetText(FText::FromString(FString::SanitizeFloat(gv_aero_debug3)));
	DebugAero5->SetText(FText::FromString(FString::SanitizeFloat(gv_aero_debug4)));
	DebugAero6->SetText(FText::FromString(FString::SanitizeFloat(gv_aero_debug5)));

	DebugAero1Text->SetText(FText::FromString(FString(gv_aero_label_debug0.c_str())));
	DebugAero2Text->SetText(FText::FromString(FString(gv_aero_label_debug1.c_str())));
	DebugAero3Text->SetText(FText::FromString(FString(gv_aero_label_debug2.c_str())));
	DebugAero4Text->SetText(FText::FromString(FString(gv_aero_label_debug3.c_str())));
	DebugAero5Text->SetText(FText::FromString(FString(gv_aero_label_debug4.c_str())));
	DebugAero6Text->SetText(FText::FromString(FString(gv_aero_label_debug5.c_str())));
}

void UDebugWidget::NativeConstruct()
{
	Super::NativeConstruct();
}

FString UDebugWidget::GetAero1Text()
{
	return FString(gv_aero_label_debug0.c_str());
}
double UDebugWidget::GetAero1Input()
{
	return gv_aero_debug0;
}
FString UDebugWidget::GetAero2Text()
{
	return FString(gv_aero_label_debug1.c_str());
}
double UDebugWidget::GetAero2Input()
{
	return gv_aero_debug1;
}
FString UDebugWidget::GetAero3Text()
{
	return FString(gv_aero_label_debug2.c_str());
}
double UDebugWidget::GetAero3Input()
{
	return gv_aero_debug2;
}
FString UDebugWidget::GetAero4Text()
{
	return FString(gv_aero_label_debug3.c_str());
}
double UDebugWidget::GetAero4Input()
{
	return gv_aero_debug3;
}
FString UDebugWidget::GetAero5Text()
{
	return FString(gv_aero_label_debug4.c_str());
}
double UDebugWidget::GetAero5Input()
{
	return gv_aero_debug4;
}
FString UDebugWidget::GetAero6Text()
{
	return FString(gv_aero_label_debug5.c_str());
}
double UDebugWidget::GetAero6Input()
{
	return gv_aero_debug5;
}

void UDebugWidget::SetAero1Text(FString aero1)
{
	gv_aero_label_debug0 = std::string(TCHAR_TO_UTF8(*aero1));
}
void UDebugWidget::SetAero1Input(double aero1in)
{
	gv_aero_debug0 = aero1in;
}
void UDebugWidget::SetAero2Text(FString aero2)
{
	gv_aero_label_debug1 = std::string(TCHAR_TO_UTF8(*aero2));
}
void UDebugWidget::SetAero2Input(double aero2in)
{
	gv_aero_debug1 = aero2in;
}
void UDebugWidget::SetAero3Text(FString aero3)
{
	gv_aero_label_debug2 = std::string(TCHAR_TO_UTF8(*aero3));
}
void UDebugWidget::SetAero3Input(double aero3in)
{
	gv_aero_debug2 = aero3in;
}
void UDebugWidget::SetAero4Text(FString aero4)
{
	gv_aero_label_debug3 = std::string(TCHAR_TO_UTF8(*aero4));
}
void UDebugWidget::SetAero4Input(double aero4in)
{
	gv_aero_debug3 = aero4in;
}
void UDebugWidget::SetAero5Text(FString aero5)
{
	gv_aero_label_debug4 = std::string(TCHAR_TO_UTF8(*aero5));
}
void UDebugWidget::SetAero5Input(double aero5in)
{
	gv_aero_debug4 = aero5in;
}
void UDebugWidget::SetAero6Text(FString aero6)
{
	gv_aero_label_debug5 = std::string(TCHAR_TO_UTF8(*aero6));
}
void UDebugWidget::SetAero6Input(double aero6in)
{
	gv_aero_debug5 = aero6in;
}