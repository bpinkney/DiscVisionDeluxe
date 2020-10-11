#pragma once

#include "HAL/Runnable.h"

// Note that we do not have to mark our class as UCLASS() if we don't want to
class DvisEstInterface : public FRunnable
{
public:
  // Custom constructor for setting up our thread with its target
  DvisEstInterface();

  // FRunnable functions
  virtual uint32 Run() override;
  virtual void Stop() override;
  virtual void Exit() override;
  // FRunnable

  FString GetTestString();

  TArray<int32> ProcessedNumbers;

  std::string test_string;

  bool IsComplete() const;

protected:

  void RunDvisEst();

  bool bStopThread = false;
};
