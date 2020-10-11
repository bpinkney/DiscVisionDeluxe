// Fill out your copyright notice in the Description page of Project Settings.



#include "DiscCharacter.h"
#include "DiscVisionDeluxeUE.h"
#include "DiscProjectile.h"
#include "Kismet/GameplayStatics.h"
#include "Kismet/KismetSystemLibrary.h"
#include "DfisX\DfisX.hpp"
#include "HAL/RunnableThread.h"
#include "DvisEstInterface.h"

// Sets default values
ADiscCharacter::ADiscCharacter()
{
 	// Set this character to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;



}

// Called when the game starts or when spawned
ACameraManager* camera_manager;
void ADiscCharacter::BeginPlay()
{
	Super::BeginPlay();
    DfisX::init();

    DvisEstInterface_StartProcess();

    ///Camera manager init
    //UWorld* World = GetWorld();
    FActorSpawnParameters SpawnParams;
    SpawnParams.Owner = this;
    SpawnParams.Instigator = GetInstigator();
    camera_manager = GetWorld()->SpawnActor<ACameraManager>(CameraManagerBP, FVector(0,0,0), FRotator(0,0,0), SpawnParams);
    APlayerController* PC = UGameplayStatics::GetPlayerController(this, 0);
    camera_manager->set_player_target(PC);
   


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

    DvisEstInterface_PrintStuff();

    DfisX::step_simulation (DeltaTime);
}

// Called to bind functionality to input
void ADiscCharacter::SetupPlayerInputComponent(UInputComponent* PlayerInputComponent)
{
	Super::SetupPlayerInputComponent(PlayerInputComponent);

	PlayerInputComponent->BindAxis("MoveForward", this, &ADiscCharacter::MoveForward);
	PlayerInputComponent->BindAxis("MoveRight", this, &ADiscCharacter::MoveRight);	

	//Set up "action" bindings.
	PlayerInputComponent->BindAction("Fire", IE_Pressed, this, &ADiscCharacter::Fire);

  PlayerInputComponent->BindAxis("PitchCamera", this, &ADiscCharacter::AddControllerPitchInput);
  PlayerInputComponent->BindAxis("TurnCamera", this, &ADiscCharacter::AddControllerYawInput);

    PlayerInputComponent->BindAction("Quit", IE_Pressed, this, &ADiscCharacter::Quit);
    PlayerInputComponent->BindAction("Action1", IE_Pressed, this, &ADiscCharacter::Action1);
    PlayerInputComponent->BindAction("Action2", IE_Pressed, this, &ADiscCharacter::Action2);
    PlayerInputComponent->BindAction("Action3", IE_Pressed, this, &ADiscCharacter::Action3);
    PlayerInputComponent->BindAction("Action4", IE_Pressed, this, &ADiscCharacter::Action4);

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
void DestroyDiscs()
{
    ;
}

void ADiscCharacter::Quit()
{
GEngine->AddOnScreenDebugMessage(-1, 30.f, FColor::Orange, "Alt QQ");
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

    // Attempt to fire a projectile.
    if (ProjectileClass)
    {
        DestroyDiscs();
        // Get the camera transform.
        FVector CameraLocation;
        FRotator CameraRotation;
        GetActorEyesViewPoint(CameraLocation, CameraRotation);

        // Transform MuzzleOffset from camera space to world space.
        FVector MuzzleLocation = CameraLocation + FTransform(CameraRotation).TransformVector(MuzzleOffset);
        MuzzleLocation += FVector (0.0,0.0,-80.0);
        FRotator MuzzleRotation = CameraRotation;
        // Skew the aim to be slightly upwards.
        
        UWorld* World = GetWorld();
        if (World)
        {
            FActorSpawnParameters SpawnParams;
            SpawnParams.Owner = this;
            SpawnParams.Instigator = GetInstigator();
            // Spawn the projectile at the muzzle.
            ADiscProjectile* Projectile = World->SpawnActor<ADiscProjectile>(ProjectileClass, MuzzleLocation, MuzzleRotation, SpawnParams);
            camera_manager->focus_on_disc(Projectile);
            if (Projectile)
            {
                // Set the projectile's initial trajectory.
                
                FVector fire_direction = MuzzleRotation.Vector();
                fire_direction *= 21;
                double throw_pitch;
                double throw_roll;
                double aim_up = 10.0;
                if ((MuzzleRotation.Pitch+aim_up)>180)
                {
                    
                throw_pitch = cos (MuzzleRotation.Yaw/57.3) * ((MuzzleRotation.Pitch+aim_up)-360)/57.3;
                throw_roll  = -sin (MuzzleRotation.Yaw/57.3) * ((MuzzleRotation.Pitch+aim_up)-360)/57.3;
                }
                else
                {
                throw_pitch = cos (MuzzleRotation.Yaw/57.3) * (MuzzleRotation.Pitch+aim_up)/57.3;
                throw_roll  = -sin (MuzzleRotation.Yaw/57.3) * (MuzzleRotation.Pitch+aim_up)/57.3;    
                }

                DfisX::new_throw (DfisX::NONE,Eigen::Vector3d (MuzzleLocation.X/100,MuzzleLocation.Y/100,MuzzleLocation.Z/100),Eigen::Vector3d (fire_direction.X,fire_direction.Y,fire_direction.Z),throw_roll, throw_pitch, -95.0, 0.0);

                
            }
        }
    }
}

// Start dvd_DvisEst Interface
// This portable block should be able to be moved to any high-level Unreal Object later
void ADiscCharacter::DvisEstInterface_StartProcess()
{
  if (!DvisEstInterfaceThread && FPlatformProcess::SupportsMultithreading())
  {
    // Run the thread until we've found many random numbers
    dvisEstInterface = new DvisEstInterface();
    DvisEstInterfaceThread = FRunnableThread::Create(dvisEstInterface, TEXT("DvisEstInterfaceThread"));
  }
}

bool ADiscCharacter::DvisEstInterface_IsComplete() const
{
  return !DvisEstInterfaceThread || dvisEstInterface->IsComplete();
}

void ADiscCharacter::DvisEstInterface_PrintStuff()
{
  if (!DvisEstInterfaceThread || !dvisEstInterface)
    return;

  if (DvisEstInterface_IsComplete())
  {
    if (GEngine)
    {
      // This should only occur when this thread is killed!
    }
  }
  else
  {
    if (GEngine)
    {
      // How the hell is this access threadsafe???
      FString test_string = dvisEstInterface->GetTestString();

      GEngine->AddOnScreenDebugMessage(-1, 0, FColor::Green,
        FString::Printf(TEXT("Sample Thread Still Working Away %d, %s"),
        dvisEstInterface->ProcessedNumbers.Num(),
        *test_string));
    }
  }
}
// End dvd_DvisEst Interface
