#pragma once

#include "HAL/Runnable.h"
#include "disc_layouts.hpp"

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

  bool IsComplete() const;

  FString GetTestString();

  void GetDiscInitState(disc_init_state_t * disc_init_state);
  bool IsReadyToThrow();
  bool IsNewThrowReady();

  disc_init_state_t disc_init_state;
  bool ReadyToThrow;
  bool NewThrowReady;
  DiscIndex LastDiscIndex;

protected:

  void RunDvisEst();
  void ParseDvisEstLine(std::string result);

  bool bStopThread = false;
};
