#pragma once

#include "HAL/Runnable.h"

// Note that we do not have to mark our class as UCLASS() if we don't want to
class DvisEstInterfaceThread : public FRunnable
{
public:
  // Custom constructor for setting up our thread with its target
  DvisEstInterfaceThread(int32 InTargetCount);

  // FRunnable functions
  virtual uint32 Run() override;
  virtual void Stop() override;
  virtual void Exit() override;
  // FRunnable

  TArray<int32> ProcessedNumbers;

  bool IsComplete() const;

protected:
  int32 TargetCount = -1;
  int32 FoundCount = -1;

  bool bStopThread = false;
};
