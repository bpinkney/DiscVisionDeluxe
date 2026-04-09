// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/GameModeBase.h"
#include "DiscVisionDeluxeUEGameModeBase.generated.h"

/**
 * 
 */
UCLASS()
class DISCVISIONDELUXEUE_API ADiscVisionDeluxeUEGameModeBase : public AGameModeBase
{
	GENERATED_BODY()
	
	virtual void StartPlay() override;
};
