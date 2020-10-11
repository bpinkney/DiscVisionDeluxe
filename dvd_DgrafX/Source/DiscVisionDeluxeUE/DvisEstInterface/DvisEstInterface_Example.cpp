#include "DvisEstInterface_Example.h"
//#include "DiscVisionDeluxeUE.h"
#include "HAL/RunnableThread.h"
#include "DvisEstInterface.h"


UDvisEstInterface_Example::UDvisEstInterface_Example(const FObjectInitializer& ObjectInitializer)
  : Super(ObjectInitializer)
{
}


void UDvisEstInterface_Example::StartProcess()
{
  if (!CurrentThread && FPlatformProcess::SupportsMultithreading())
  {
    // Run the thread until we've found 999 random numbers
    dvisEstInterface = new DvisEstInterface(999);
    CurrentThread = FRunnableThread::Create(dvisEstInterface, TEXT("Any old thread name"));
  }
}


bool UDvisEstInterface_Example::IsComplete() const
{
  return !CurrentThread || dvisEstInterface->IsComplete();
}


void UDvisEstInterface_Example::PrintStuff()
{
  if (!CurrentThread || !dvisEstInterface)
    return;

  if (IsComplete())
  {
    if (GEngine)
    {
      GEngine->AddOnScreenDebugMessage(-1, 0, FColor::Green,
        FString::Printf(TEXT("Numbers generated:: %d, First 3 are: %d, %d, %d"),
        dvisEstInterface->ProcessedNumbers.Num(),
        (dvisEstInterface->ProcessedNumbers.Num() > 0)
          ? dvisEstInterface->ProcessedNumbers[0] : -1,
        (dvisEstInterface->ProcessedNumbers.Num() > 1)
          ? dvisEstInterface->ProcessedNumbers[1] : -1,
        (dvisEstInterface->ProcessedNumbers.Num() > 2)
          ? dvisEstInterface->ProcessedNumbers[2] : -1));
    }
  }
  else
  {
    if (GEngine)
    {
      GEngine->AddOnScreenDebugMessage(-1, 0, FColor::Green,
        FString::Printf(TEXT("Still processing: %d"),
        dvisEstInterface->ProcessedNumbers.Num()));
    }
  }
}
