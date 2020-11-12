// Fill out your copyright notice in the Description page of Project Settings.


#include "RangeUserWidget.h"
#include <Runtime\UMG\Public\Components\TextBlock.h>


URangeUserWidget::URangeUserWidget(const FObjectInitializer& ObjectInitializer) : Super(ObjectInitializer)
{

}

void URangeUserWidget::PopulateHUD() 
{
	discspeed->SetText(FText::FromString("hey"));
}

void URangeUserWidget::UpdateHUD()
{
}
