#pragma once

//#include "DvisEstInterface.generated.h"

UCLASS()
class DvisEstInterface : public UObject
{
  GENERATED_BODY()

public:
  DvisEstInterface(const FObjectInitializer& ObjectInitializer);

  // Call this to create the thread and start it going
  void StartProcess();

  // Call this to print the current state of the thread
  void PrintStuff();

protected:
  bool IsComplete() const;

  class DvisEstInterfaceThread* MyDvisEstInterfaceThread = nullptr;
  FRunnableThread* CurrentThread = nullptr;
};
