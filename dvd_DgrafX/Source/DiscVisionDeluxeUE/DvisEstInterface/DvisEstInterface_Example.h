#pragma once

#include "DvisEstInterface_Example.generated.h"

UCLASS()
class UDvisEstInterface_Example : public UObject
{
  GENERATED_BODY()

public:
  UDvisEstInterface_Example(const FObjectInitializer& ObjectInitializer);

  // Call this to create the thread and start it going
  void StartProcess();

  // Call this to print the current state of the thread
  void PrintStuff();

protected:
  bool IsComplete() const;

  class DvisEstInterface* dvisEstInterface = nullptr;
  FRunnableThread* CurrentThread = nullptr;
};
