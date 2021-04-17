// Fill out your copyright notice in the Description page of Project Settings.



//Unreal stuff
#include "DiscCharacter.h"
#include "DiscVisionDeluxeUE.h"
#include "DiscProjectile.h"
#include "Kismet/GameplayStatics.h"
#include "Kismet/KismetSystemLibrary.h"


// DfisX stuff
#include "dvd_maths.hpp"
#include "DfisX\DfisX.hpp"
#include "ThrowInputController.h"







// Sets default values
ADiscCharacter::ADiscCharacter()
{
 	// Set this character to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;
	teepad_locations_iterator = 0;



}

// Called when the game starts or when spawned

void ADiscCharacter::BeginPlay()
{
	Super::BeginPlay();
	
  
  ///Camera manager init
  FActorSpawnParameters SpawnParams;
  SpawnParams.Owner = this;
  SpawnParams.Instigator = GetInstigator();
  ptr_camera_manager = GetWorld()->SpawnActor<ACameraManager>(CameraManagerBP, FVector(0,0,0), FRotator(0,0,0), SpawnParams);
  APlayerController* PC = UGameplayStatics::GetPlayerController(this, 0);
  ptr_camera_manager->set_player_target(PC);
  ptr_camera_manager->focus_on_player();

  ///throw input controller init  
  ptr_throw_input_controller = GetWorld()->SpawnActor<AThrowInputController>(ThrowInputControllerBP, FVector(0,0,0), FRotator(0,0,0), SpawnParams);


	if (GEngine)
	{
		// Put up a debug message for five seconds. The -1 "Key" value (first argument) indicates that we will never need to update or refresh this message.
		GEngine->AddOnScreenDebugMessage(-1, 5.0f, FColor::Red, TEXT("We are using DiscCharacter."));
	}
	
}



// Called every frame
void ADiscCharacter::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

  //RunTimingLoops();
}

// Called to bind functionality to input
void ADiscCharacter::SetupPlayerInputComponent(UInputComponent* PlayerInputComponent)
{
	Super::SetupPlayerInputComponent(PlayerInputComponent);

	PlayerInputComponent->BindAxis("MoveForward", this, &ADiscCharacter::MoveForward);
	PlayerInputComponent->BindAxis("MoveRight", this, &ADiscCharacter::MoveRight);	

	//Set up "action" bindings.
	//PlayerInputComponent->BindAction("Fire", IE_Pressed, this, &ADiscCharacter::Fire);

    //PlayerInputComponent->BindAxis("PitchCamera", this, &ADiscCharacter::AddControllerPitchInput);
    //PlayerInputComponent->BindAxis("TurnCamera", this, &ADiscCharacter::AddControllerYawInput);

    PlayerInputComponent->BindAction("Quit", IE_Pressed, this, &ADiscCharacter::Quit);
    PlayerInputComponent->BindAction("Action1", IE_Pressed, this, &ADiscCharacter::Action1);
    PlayerInputComponent->BindAction("Action2", IE_Pressed, this, &ADiscCharacter::Action2);
    PlayerInputComponent->BindAction("Action3", IE_Pressed, this, &ADiscCharacter::Action3);
    PlayerInputComponent->BindAction("Action4", IE_Pressed, this, &ADiscCharacter::Action4);

}

void ADiscCharacter::main_menu_next_btn()
{
	//GEngine->AddOnScreenDebugMessage(-1, 30.f, FColor::Green, FString(" dc next btn"));
	teepad_locations_iterator ++;
	if (teepad_locations_iterator > (teepad_locations.Num()-1)) teepad_locations_iterator = 0;
    ptr_camera_manager->focus_on_teepad (teepad_locations_iterator);
}

void ADiscCharacter::main_menu_prev_btn()
{
	//GEngine->AddOnScreenDebugMessage(-1, 30.f, FColor::Green, FString(" dc prev btn"));
	teepad_locations_iterator--;
	if (teepad_locations_iterator < 0) teepad_locations_iterator =  teepad_locations.Num()-1;
	ptr_camera_manager->focus_on_teepad (teepad_locations_iterator);
}

void ADiscCharacter::main_menu_choose_location_btn()
{
	GEngine->AddOnScreenDebugMessage(-1, 30.f, FColor::Green, FString(" dc engage btn"));

	FHitResult OutSweepHitResult = FHitResult();
	GetController()->SetControlRotation(teepad_locations[teepad_locations_iterator].GetRotation().Rotator());
	SetActorTransform(
	teepad_locations[teepad_locations_iterator],
    false,
    0,
    ETeleportType::None);
    ptr_camera_manager->focus_on_player();


}

void ADiscCharacter::MoveForward(float Value)
{
	// Find out which way is "forward" and record that the player wants to move that way.
	FVector Direction = FRotationMatrix(Controller->GetControlRotation()).GetScaledAxis(EAxis::X);
	AddMovementInput(Direction, Value);
}

void ADiscCharacter::MoveRight(float Value)
{
  // Find out which way is "right" and record that the player wants to move that way.
  FVector Direction = FRotationMatrix(Controller->GetControlRotation()).GetScaledAxis(EAxis::Y);
  AddMovementInput(Direction, Value);
}


/*
void ADiscCharacter::ZoomCamera(float Value)
{
    // Find out which way is "right" and record that the player wants to move that way.
    FVector Direction = FRotationMatrix(Controller->GetControlRotation()).GetScaledAxis(EAxis::Y);
    AddMovementInput(Direction, Value);
}
*/

void ADiscCharacter::Quit()
{
  // override with handy quit key mapping for now
    GEngine->AddOnScreenDebugMessage(-1, 30.f, FColor::Orange, "Alt QQ");
    UKismetSystemLibrary::QuitGame(this, nullptr, EQuitPreference::Quit, false);
  
}
    void ADiscCharacter::Action1()
{
GEngine->AddOnScreenDebugMessage(-1, 30.f, FColor::Orange, "Action 1");
SetActorLocationAndRotation(FVector(-381,-1300,3091),FRotator(0,-90,0), false, 0, ETeleportType::None);
APlayerController* PC = UGameplayStatics::GetPlayerController(this, 0);
PC->SetControlRotation(FRotator(0,-90,0));


}
    void ADiscCharacter::Action2()
{
GEngine->AddOnScreenDebugMessage(-1, 30.f, FColor::Orange, "Action 2");
SetActorLocationAndRotation(FVector(-360,-70,3091),FRotator(0,0,0), false, 0, ETeleportType::None);
APlayerController* PC = UGameplayStatics::GetPlayerController(this, 0);
PC->AController::SetControlRotation(FRotator(0,0,0));

}
    void ADiscCharacter::Action3()
{
GEngine->AddOnScreenDebugMessage(-1, 30.f, FColor::Orange, "Action 3");
}
    void ADiscCharacter::Action4()
{
GEngine->AddOnScreenDebugMessage(-1, 30.f, FColor::Orange, "Action 4");
}


void ADiscCharacter::Fire()
{
  // override with handy quit key mapping for now
  //UKismetSystemLibrary::QuitGame(this, nullptr, EQuitPreference::Quit, false);
  //PerformThrow(true, nullptr);
}



