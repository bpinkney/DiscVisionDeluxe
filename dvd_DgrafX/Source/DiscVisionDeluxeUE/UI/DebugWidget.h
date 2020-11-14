// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Blueprint/UserWidget.h"
#include "Runtime/UMG/Public/UMG.h"
#include "DebugWidget.generated.h"


// add some runtime tuning global hook-up
extern std::string gv_aero_label_debug0;
extern double gv_aero_debug0;

extern std::string gv_aero_label_debug1;
extern double gv_aero_debug1;

extern std::string gv_aero_label_debug2;
extern double gv_aero_debug2;

extern std::string gv_aero_label_debug3;
extern double gv_aero_debug3;

extern std::string gv_aero_label_debug4;
extern double gv_aero_debug4;

extern std::string gv_aero_label_debug5;
extern double gv_aero_debug5;

extern std::string gv_aero_label_debug6;
extern double gv_aero_debug6;

/**
 * 
 */
UCLASS()
class DISCVISIONDELUXEUE_API UDebugWidget : public UUserWidget
{
	GENERATED_BODY()
	
public:

	UDebugWidget(const FObjectInitializer& ObjectInitializer);

	void InitializeDAero();

	virtual void NativeConstruct() override;

	double daero_debug_input1;
	double daero_debug_input2;
	double daero_debug_input3;
	double daero_debug_input4;
	double daero_debug_input5;
	double daero_debug_input6;

	FString daero_debug_text1;
	FString daero_debug_text2;
	FString daero_debug_text3;
	FString daero_debug_text4;
	FString daero_debug_text5;
	FString daero_debug_text6;

	FString GetAero1Text();
	double GetAero1Input();

	FString GetAero2Text();
	double GetAero2Input();

	FString GetAero3Text();
	double GetAero3Input();

	FString GetAero4Text();
	double GetAero4Input();

	FString GetAero5Text();
	double GetAero5Input();

	FString GetAero6Text();
	double GetAero6Input();

	void SetAero1Text(FString aero1);
	void SetAero1Input(double aero1in);

	void SetAero2Text(FString aero2);
	void SetAero2Input(double aero2in);

	void SetAero3Text(FString aero03);
	void SetAero3Input(double aero3in);

	void SetAero4Text(FString aero4);
	void SetAero4Input(double aero4in);

	void SetAero5Text(FString aero5);
	void SetAero5Input(double aero5in);

	void SetAero6Text(FString aero6);
	void SetAero6Input(double aero6in);

	UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (BindWidget))
	class UTextBlock* DebugAero1Text;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (BindWidget))
	class UTextBlock* DebugAero2Text;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (BindWidget))
	class UTextBlock* DebugAero3Text;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (BindWidget))
	class UTextBlock* DebugAero4Text;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (BindWidget))
	class UTextBlock* DebugAero5Text;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (BindWidget))
	class UTextBlock* DebugAero6Text;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (BindWidget))
	class UEditableTextBox* DebugAero1;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (BindWidget))
	class UEditableTextBox* DebugAero2;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (BindWidget))
	class UEditableTextBox* DebugAero3;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (BindWidget))
	class UEditableTextBox* DebugAero4;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (BindWidget))
	class UEditableTextBox* DebugAero5;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, meta = (BindWidget))
	class UEditableTextBox* DebugAero6;
};
