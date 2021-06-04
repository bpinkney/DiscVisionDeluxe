// Fill out your copyright notice in the Description page of Project Settings.



//Unreal stuff
#include "DiscCharacter.h"
#include "DiscVisionDeluxeUE.h"
#include "DiscProjectile.h"
#include "Kismet/GameplayStatics.h"
#include "Kismet/KismetSystemLibrary.h"
#include "ThrowInputController.h"
#include "UI/RangeHUD.h"

// DfisX stuff
#include "dvd_maths.hpp"
#include "DfisX\DfisX.hpp"



#define one_throw_at_a_time true




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
	//UGameplayStatics::SetGlobalTimeDilation(GetWorld(), 0.1);
	////Finite state machine for character
	FSM_character_state = enum_character_state::READY_TO_THROW;
	//used to hold current state when pausing
	FSM_held_state = enum_character_state::READY_TO_THROW;
	//used to get output text for display
	context_string = FString(" no context set ");
	location_name_string = FString(" no location set ");
	
	
  
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



}



// Called every frame
void ADiscCharacter::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);	
  //RunTimingLoops();
	
	GEngine->AddOnScreenDebugMessage(100, 15.0f, FColor::White, get_state_display_string());
}

// Called to bind functionality to input
void ADiscCharacter::SetupPlayerInputComponent(UInputComponent* PlayerInputComponent)
{
	Super::SetupPlayerInputComponent(PlayerInputComponent);

	//PlayerInputComponent->BindAxis("MoveForward", this, &ADiscCharacter::MoveForward);
	//PlayerInputComponent->BindAxis("MoveRight", this, &ADiscCharacter::MoveRight);	

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

    ARangeHUD * rangeHUD = Cast<ARangeHUD>(GetWorld()->GetFirstPlayerController()->GetHUD());
    rangeHUD->set_minimap_target(teepad_locations_iterator);


}

void ADiscCharacter::MoveForward(float Value)
{
	// Find out which way is "forward" and record that the player wants to move that way.
	//FVector Direction = FRotationMatrix(Controller->GetControlRotation()).GetScaledAxis(EAxis::X);
	//AddMovementInput(Direction, Value);
}

void ADiscCharacter::MoveRight(float Value)
{
  // Find out which way is "right" and record that the player wants to move that way.
  //FVector Direction = FRotationMatrix(Controller->GetControlRotation()).GetScaledAxis(EAxis::Y);
  //AddMovementInput(Direction, Value);
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
}
    void ADiscCharacter::Action2()
{
GEngine->AddOnScreenDebugMessage(-1, 30.f, FColor::Orange, "Action 2");
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


///////////////////////////State Machine Stuff//////////////////////////////
///fsm functions
FString ADiscCharacter::get_state_display_string()
{

	if (get_is_paused())
	{
    FText statetext = UEnum::GetDisplayValueAsText(FSM_character_state);
    FText heldtext = UEnum::GetDisplayValueAsText(FSM_held_state);
    //FText contexttext = UEnum::GetDisplayValueAsText(FSM_character_state);
	return (heldtext.ToString()+statetext.ToString()+context_string);
	} 
	else
	{
    FText statetext = UEnum::GetDisplayValueAsText(FSM_character_state);
	return (statetext.ToString()+location_name_string);
	}
}

//fsm transitions
void ADiscCharacter::disc_was_thrown()    {if (one_throw_at_a_time) FSM_character_state = enum_character_state::SIMULATING;}
void ADiscCharacter::throw_was_finished() {if (one_throw_at_a_time) FSM_character_state = enum_character_state::READY_TO_THROW;}

void ADiscCharacter::character_was_paused()
{
  if (!get_is_paused()) FSM_held_state = FSM_character_state;
    FSM_character_state = enum_character_state::PAUSED;
    UGameplayStatics::SetGlobalTimeDilation(GetWorld(), 0.0);
}
void ADiscCharacter::pause_was_finished()
 {
  	FSM_character_state = FSM_held_state;
    UGameplayStatics::SetGlobalTimeDilation(GetWorld(), 1.0);
}
//fsm getters
bool ADiscCharacter::get_is_ready_to_throw(){return FSM_character_state==enum_character_state::READY_TO_THROW;}
bool ADiscCharacter::get_is_throw_simulating(){return FSM_character_state==enum_character_state::SIMULATING;}
bool ADiscCharacter::get_is_paused(){return FSM_character_state==enum_character_state::PAUSED;}

