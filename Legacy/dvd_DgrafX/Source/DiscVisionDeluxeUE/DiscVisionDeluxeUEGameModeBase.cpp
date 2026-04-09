// Copyright Epic Games, Inc. All Rights Reserved.


#include "DiscVisionDeluxeUEGameModeBase.h"
#include "DiscVisionDeluxeUE.h"

void ADiscVisionDeluxeUEGameModeBase::StartPlay()
{
    Super::StartPlay();

    if (GEngine)
    {
        // Display a debug message for five seconds. 
        // The -1 "Key" value (first argument) indicates that we will never need to update or refresh this message.
        GEngine->AddOnScreenDebugMessage(-1, 5.0f, FColor::Yellow, TEXT("Hello World, this is FPSGameMode!"));
    }
}