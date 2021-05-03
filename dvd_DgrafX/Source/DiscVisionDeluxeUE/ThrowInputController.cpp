// Fill out your copyright notice in the Description page of Project Settings.

#include "ThrowInputController.h"
#include "HAL/RunnableThread.h"
#include "DvisEstInterface.h"
#include "DiscThrow.h"
#include "DiscCharacter.h"
#include "UI/RangeHUD.h"

// convenience settings for dvd_DvisEst interface
#define DVISEST_INTERFACE_ENABLED              (true)
#define DVISEST_INTERFACE_USE_GENERATED_THROWS (true)

// all throws in queue
//std::deque<ADiscThrow> DiscThrowQueue;
//TQueue<ADiscThrow> DiscThrowQueue;
ADiscThrow *sv_latest_discthrow;
// Sets default values
AThrowInputController::AThrowInputController()
{
 	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;

}

// Called when the game starts or when spawned
void AThrowInputController::BeginPlay()
{
	Super::BeginPlay();

	if(DVISEST_INTERFACE_ENABLED)
		DvisEstInterface_StartProcess();
	
}

// gets called when Unreal ends play on this actor pre exit
void AThrowInputController::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
  // terminate DvisEst Process
  if(DVISEST_INTERFACE_ENABLED)
    dvisEstInterface->Exit();

  Super::EndPlay(EndPlayReason);
}


// Called every frame
void AThrowInputController::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

	// dvd_DvisEst Interface
  if(DVISEST_INTERFACE_ENABLED)
  {

    if(dvisEstInterface->IsNewThrowReady())
    {
      disc_init_state_t new_disc_init_state;
      dvisEstInterface->GetDiscInitState(&new_disc_init_state);

      PerformCapturedThrow(&new_disc_init_state);
    }
    ARangeHUD* RangeHUD = Cast<ARangeHUD>(GetWorld()->GetFirstPlayerController()->GetHUD());
    if (RangeHUD)
    {
        //RangeHUD->PopulateHUD(sv_latest_discthrow);
    }
    DvisEstInterface_PrintStuff();
  }
  // end dvd_DvisEst Interface

}


void AThrowInputController::PerformCapturedThrow(disc_init_state_t * new_disc_init_state)
{
	GEngine->AddOnScreenDebugMessage(-1, 5.0f, FColor::Green, TEXT("Perform Captured throw!."));

	UWorld* World = GetWorld();
    FActorSpawnParameters SpawnParams;
    SpawnParams.Owner = GetOwner();
    SpawnParams.Instigator = GetInstigator();

    ADiscThrow* disc_throw = World->SpawnActor<ADiscThrow>(DiscThrowBP, FVector(0,0,0), FRotator(0,0,0), SpawnParams);
    sv_latest_discthrow = disc_throw;
    // add new disc_throw to queue
    //const bool queue_empty_before = DiscThrowQueue.IsEmpty();
    //DiscThrowQueue.Enqueue(*disc_throw);
    //const bool queue_empty_after = DiscThrowQueue.IsEmpty();

    //GEngine->AddOnScreenDebugMessage(-1, 5.0f, FColor::Green, 
      //FString::Printf(TEXT("DiscThrowQueue empty? = %d/%d"),
        //queue_empty_before, queue_empty_after));

          // Some magic will happen here for the mapping between the DvisEst DiscIndex
          // e.g. MIDRANGE_OS
          // and the DfisX/DgrafX Disc_Mold_Enum
          // e.g. 175g big-Z BUZZZ

          // The world axes appear to be defined in a different way to our defs (we defined X forward, Y Right, Z up)
          // Maybe you can fix this one up Mike? for now I just have random -1s scattered

          // perform a new throw!
	 //DfisX::Disc_Mold_Enum new_disc_enum = static_cast<DfisX::Disc_Mold_Enum>(new_disc_init_state->discmold);
          

            disc_throw->new_captured_throw(static_cast<int>(new_disc_init_state->discmold),
            FVector(0,0,0),FVector(30,0,10),1.57,0,0,0);

/*
          disc_throw->new_captured_throw(
            static_cast<int>(new_disc_init_state->discmold),   //disc_mold_enum goes here, static cast it to int though because it is passing through uproperties first. it will get cast back when it hits dfisx
            FVector(
              100 * new_disc_init_state->lin_pos_xyz[0], // negative for some reason? no idea what the world frame is here
              100 * new_disc_init_state->lin_pos_xyz[1], // negative for some reason? no idea what the world frame is here
              100 * new_disc_init_state->lin_pos_xyz[2]), // just zero this until the UI is sorted
            FVector(
              1 * new_disc_init_state->lin_vel_xyz[0], // negative for some reason? no idea what the world frame is here
              1 * new_disc_init_state->lin_vel_xyz[1], // negative for some reason? no idea what the world frame is here
                   new_disc_init_state->lin_vel_xyz[2]),
            1 *   new_disc_init_state->ang_pos_hps[0], // negative for some reason? no idea what the world frame is here
            1 *   new_disc_init_state->ang_pos_hps[1], // negative for some reason? no idea what the world frame is here
                   new_disc_init_state->ang_vel_hps[2],
                   new_disc_init_state->wobble);
*/
}



// Start dvd_DvisEst Interface
// This portable block should be able to be moved to any high-level Unreal Object later
void AThrowInputController::DvisEstInterface_StartProcess()
{
	GEngine->AddOnScreenDebugMessage(-1, 5.0f, FColor::Green, TEXT("VisEst Process Start"));
  if (!DvisEstInterfaceThread && FPlatformProcess::SupportsMultithreading())
  {
    // Run the thread indefinitely
    dvisEstInterface = new DvisEstInterface(DVISEST_INTERFACE_USE_GENERATED_THROWS);
    DvisEstInterfaceThread = FRunnableThread::Create(dvisEstInterface, TEXT("DvisEstInterfaceThread"));
  }
}

bool AThrowInputController::DvisEstInterface_IsComplete() const
{
  return !DvisEstInterfaceThread || dvisEstInterface->IsComplete();
}

void AThrowInputController::DvisEstInterface_PrintStuff()
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

      GEngine->AddOnScreenDebugMessage(-1, 0, FColor::White,
        FString::Printf(TEXT("dvd_DvisEst Thread is Still Working Away:%s"),
        *test_string));
    }
  }
}
// End dvd_DvisEst Interface
      
    
  



