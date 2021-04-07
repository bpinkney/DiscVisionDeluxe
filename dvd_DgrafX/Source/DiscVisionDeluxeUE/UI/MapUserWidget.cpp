// Fill out your copyright notice in the Description page of Project Settings.


#include "MapUserWidget.h"

UMapUserWidget::UMapUserWidget(const FObjectInitializer& ObjectInitializer) : Super(ObjectInitializer)
{

}

void UMapUserWidget::NativeConstruct()
{
	Super::NativeConstruct();
}

void OnPaint(UPARAM(ref) FPaintContext& Context) const override
{

}

FVector2D GetGameViewportSize()
{
    FVector2D Result = FVector2D(1, 1);

    if (GEngine && GEngine->GameViewport)
    {
        GEngine->GameViewport->GetViewportSize( /*out*/Result);
    }

    return Result;
}

FVector2D GetGameResolution()
{
    FVector2D Result = FVector2D(1, 1);

    Result.X = GSystemResolution.ResX;
    Result.Y = GSystemResolution.ResY;

    return Result;
}

void DrawTopLine()
{
	

}

