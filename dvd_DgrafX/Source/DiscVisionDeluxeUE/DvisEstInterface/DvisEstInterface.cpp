#include "DvisEstInterface.h"
#include "GenericPlatform/GenericPlatformProcess.h"
#include "Misc/Paths.h"
#include "Misc/InteractiveProcess.h"

#include <cstdio>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>

DvisEstInterface::DvisEstInterface()
{
  // define class vars here
}

void DvisEstInterface::RunDvisEst() 
{
  //char buffer[512];

  FString dVisEst_bin_cmd(
    FPaths::ConvertRelativePathToFull(FPaths::GameSourceDir() + 
    "../../Binaries/dvd_DvisEst/2020-10-11/dvd_DvisEst.exe"));
  // wow FInterativeProcess makes me handle the separation of stderr here.... terrible job guys
  FString dVisEst_args("-cr -rm=5 -nc 2> nul");
  
  //std::string cmd = std::string(TCHAR_TO_UTF8(*dVisEst_bin_path)) + " -cr -rm=5 -nc";

  /*std::unique_ptr<FILE, decltype(&_pclose)> pipe(_popen(cmd.c_str(), "r"), _pclose);
  if (!pipe) 
  {
    throw std::runtime_error("popen() failed!");
  }
  while (!IsComplete() && fgets(buffer, sizeof buffer, pipe.get()) != nullptr) 
  {
    // show changes
    ProcessedNumbers.Add(FMath::RandRange(0, 999));

    const int buffer_len = strlen(buffer);
    if(buffer[buffer_len-1] == '\n')
    {
      //buffer[buffer_len-1] = '\0';
      //buffer[buffer_len-2] = '\0';
      //result = "";
      result += buffer;
    }    
  }*/

  FInteractiveProcess* WrapperProcess;
  WrapperProcess = new FInteractiveProcess(
    dVisEst_bin_cmd,
    dVisEst_args,
    true,
    false
    );

  WrapperProcess->OnOutput().BindLambda(
    [=](const FString& outputMessage)
    {
      ProcessedNumbers.Add(FMath::RandRange(0, 999));
      FString outputString(outputMessage);
      std::string result = std::string(TCHAR_TO_UTF8(*outputString));
      test_string += result;
    }
  );

  WrapperProcess->Launch();

  //test_string = result;
}

FString DvisEstInterface::GetTestString()
{
  FString new_string(test_string.c_str());
  return new_string;
}


uint32 DvisEstInterface::Run()
{
  // Keep processing until we're cancelled through Stop() or we're done,
  // although this thread will suspended for other stuff to happen at the same time
  //while (!IsComplete())
  //{
    // This is where we would do our expensive threaded processing

    
    RunDvisEst();
    //FPlatformProcess::Sleep(3.0);
  //}

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
  return false;
}
