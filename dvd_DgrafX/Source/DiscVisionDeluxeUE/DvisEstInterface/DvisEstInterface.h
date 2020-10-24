#pragma once

#include "HAL/Runnable.h"
#include "disc_layouts.hpp"
#include <mutex>
#include <atomic>

// Note that we do not have to mark our class as UCLASS() if we don't want to
class DvisEstInterface : public FRunnable
{
public:
  // Custom constructor for setting up our thread with its target
  DvisEstInterface(const bool generated_throws);

  // FRunnable functions
  virtual uint32 Run() override;
  virtual void Stop() override;
  virtual void Exit() override;
  // FRunnable

  bool IsComplete() const;

  std::string test_string;

  FString GetTestString();

  void GetDiscInitState(disc_init_state_t * disc_init_state);
  bool IsReadyToThrow();
  bool IsNewThrowReady();

  bool use_generated_throws;

  // we'll use mutexed access to keep this thread safe
  std::mutex NewThrowMutex;
  disc_init_state_t disc_init_state;

  // thse can just be made atomic to ensure thread safety
  std::atomic<bool> DvisEstInitComplete;
  std::atomic<bool> ReadyToThrow;
  std::atomic<bool> NewThrowReady;
  std::atomic<DiscIndex> LastDiscIndex;

protected:

  void RunDvisEst();
  void ParseDvisEstLine(std::string result);

  bool bStopThread = false;
};
