//#include "MyProjectPCH.h"
#include "DiscVisionDeluxeUE.h"
#include "HAL/RunnableThread.h"
#include "DvisEstInterfaceThread.h"
#include "DvisEstInterface.h"

DvisEstInterface::DvisEstInterface(const FObjectInitializer& ObjectInitializer)
  : Super(ObjectInitializer)
{
}


void DvisEstInterface::StartProcess()
{
  if (!CurrentThread && FPlatformProcess::SupportsMultithreading())
  {
    // Run the thread until we've found 999 random numbers
    MyDvisEstInterfaceThread = new DvisEstInterfaceThread(999);
    CurrentThread = FRunnableThread::Create(MyDvisEstInterfaceThread, TEXT("Any old thread name"));
  }
}


bool DvisEstInterface::IsComplete() const
{
  return !CurrentThread || MyDvisEstInterfaceThread->IsComplete();
}


void DvisEstInterface::PrintStuff()
{
  if (!CurrentThread || !MyDvisEstInterfaceThread)
    return;

  if (IsComplete())
  {
    if (GEngine)
    {
      GEngine->AddOnScreenDebugMessage(-1, 0, FColor::Green,
        FString::Printf(TEXT("Numbers generated:: %d, First 3 are: %d, %d, %d"),
        MyDvisEstInterfaceThread->ProcessedNumbers.Num(),
        (MyDvisEstInterfaceThread->ProcessedNumbers.Num() > 0)
          ? MyDvisEstInterfaceThread->ProcessedNumbers[0] : -1,
        (MyDvisEstInterfaceThread->ProcessedNumbers.Num() > 1)
          ? MyDvisEstInterfaceThread->ProcessedNumbers[1] : -1,
        (MyDvisEstInterfaceThread->ProcessedNumbers.Num() > 2)
          ? MyDvisEstInterfaceThread->ProcessedNumbers[2] : -1));
    }
  }
  else
  {
    if (GEngine)
    {
      GEngine->AddOnScreenDebugMessage(-1, 0, FColor::Green,
        FString::Printf(TEXT("Still processing: %d"),
        MyDvisEstInterfaceThread->ProcessedNumbers.Num()));
    }
  }
}
