// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Blueprint/UserWidget.h"


#include "../DfisX/disc_params.hpp"
#include "../DfisX/DfisX.hpp"


#include "BagMakerWidget.generated.h"

USTRUCT(BlueprintType)
struct FListDisc
{
	GENERATED_USTRUCT_BODY();	

public:
	UPROPERTY(BlueprintReadWrite, EditAnywhere)
		FString mold_name = "";	

	UPROPERTY(BlueprintReadWrite, EditAnywhere)
		FString manufacturer = "";	

	UPROPERTY(BlueprintReadWrite, EditAnywhere)
		FString disc_type = "";	
		/*Returns the property 'ExampleProperty'*/
	//float GetExampleProperty() const { return ExampleProperty; };
		/*Sets the property 'ExampleProperty' to Value, Clamped between 0 and 1*/
	//void SetExampleProperty(const float Value) { ExampleProperty = FMath::Clamp(Value, 0.0f, 1.0f); };


};
/**
 * 
 */
UCLASS()
class DISCVISIONDELUXEUE_API UBagMakerWidget : public UUserWidget
{
	GENERATED_BODY()
	public:
		
	virtual void NativeConstruct() override;

	UPROPERTY(BlueprintReadWrite, EditAnywhere)	
	TArray<FListDisc> list_disc_array;

	UFUNCTION(BlueprintPure, Category = "Default")
 	static bool get_file_from_directory (TArray<FString>& Files, FString RootFolderFullPath, FString Ext);
	
};

