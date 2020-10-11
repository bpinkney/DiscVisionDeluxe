#include "DvisEstInterface.h"

DvisEstInterface::DvisEstInterface(int32 InTargetCount)
{
  TargetCount = InTargetCount;
  FoundCount = 0;
}


uint32 DvisEstInterface::Run()
{
  bStopThread = false;

  // Keep processing until we're cancelled through Stop() or we're done,
  // although this thread will suspended for other stuff to happen at the same time
  while (!bStopThread && !IsComplete())
  {
    // This is where we would do our expensive threaded processing

    // Instead we're going to make a reaaaally busy while loop to slow down processing
    // You can change INT_MAX to something smaller if you want it to run faster
    int32 x = 0;
    while (x < INT_MAX)
    {
      x++;
    }
    ProcessedNumbers.Add(FMath::RandRange(0, 999));
    FoundCount += 1;
  }

  // Return success
  return 0;
}


void DvisEstInterface::Exit()
{
  // Here's where we can do any cleanup we want to 
}


void DvisEstInterface::Stop()
{
  // Force our thread to stop early
  bStopThread = true;
}


bool DvisEstInterface::IsComplete() const
{
  return FoundCount >= TargetCount;
}
