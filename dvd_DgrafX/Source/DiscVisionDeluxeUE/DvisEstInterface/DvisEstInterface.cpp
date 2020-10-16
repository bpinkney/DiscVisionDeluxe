
#include "DvisEstInterface.h"
#include "GenericPlatform/GenericPlatformProcess.h"
#include "Misc/Paths.h"
#include "Misc/InteractiveProcess.h"

#include <cstdio>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <iostream>
#include <fstream>
#include <sstream>
#include <algorithm>

DvisEstInterface::DvisEstInterface(const bool generated_throws)
{
  use_generated_throws = generated_throws;
  // init
  memset(&disc_init_state, 0, sizeof(disc_init_state_t));
}

void DvisEstInterface::ParseDvisEstLine(std::string result)
{
  // split into keyvalue pairs
  std::string s = result;
  std::string delimiter_out = ",";
  std::string delimiter_in  = ":";
  // remove newlines
  s.erase(std::remove(s.begin(), s.end(), '\n'), s.end());

  bool new_throw = false;

  size_t pos_out = 0;
  size_t pos_in  = 0;
  std::string token;
  while ((pos_out = s.find(delimiter_out)) != std::string::npos) 
  {
    if (s.find(delimiter_out) != std::string::npos)
    {
      token = s.substr(0, pos_out);

      if (token.find(delimiter_in) != std::string::npos)
      {
        if((pos_in = token.find(delimiter_in)) != std::string::npos)
        {
          // parse key value pair
          std::string key = token.substr(0, pos_in);
          token.erase(0, pos_in + delimiter_in.length());
          float value = stof(token);
          if(key == "ready")
          {
            if(value > 0)
            {
              ReadyToThrow = true;
            }
            else
            {
              ReadyToThrow = false;
            }
          }
          else
          {
            dvd_DvisEst_set_key_value(key, value, &disc_init_state);
            if(key == "discmold")
            {
              LastDiscIndex = (DiscIndex)((int)value);
            }
            else if(key == "posx")
            {
              // we'll use the presence of posx to tip off whether a new throw has been received
              new_throw = true;
            }
          }
        }
      }

      s.erase(0, pos_out + delimiter_out.length());
    }
  }

  // now that all the fields are parsed (how the F does this thread safety work??)
  // we can tip off the main thread
  if(new_throw)
  {
    NewThrowReady = true;
  }
}

FString dvd_DvisEst_bin_path = "../../Binaries/dvd_DvisEst/2020-10-11/";

void DvisEstInterface::RunDvisEst() 
{
  // Holy Mother****er, if this is not the sketchiest pile of crap I have ever coded
  // Sweet Lord, what have you made me do Windows?

  FString dVisEst_bin_path(
    FPaths::ConvertRelativePathToFull(FPaths::GameSourceDir() + 
    dvd_DvisEst_bin_path));

  FString dVisEst_args("");

  std::ofstream batFileKill(std::string(TCHAR_TO_UTF8(*dVisEst_bin_path)) + "dvd_DvisEst_abstractor_kill.bat");

  // Windows is shite, and unreal FInteractiveProcess is shite, so we need to kill old dvd_DvisEst processes....
  // TEMPORARY! Killing process by name is not the way to handle this EVEN IN THE NEAR FUTURE
  batFileKill << "@Echo off & SetLocal EnableDelayedExpansion" << std::endl;
  batFileKill << "set \"PID=\"" << std::endl;
  batFileKill << "for /f \"tokens=2\" %%A in ('tasklist ^| findstr /i \"dvd_DvisEst.exe\" 2^>NUL') do @Set \"PID=!PID!,%%A\"" << std::endl;
  batFileKill << "if defined PID Echo dvd_DvisEst.exe has PID(s) %PID:~1%" << std::endl;
  // this line is the reason this needs to be 2 commands, 
  // since suppressing the STUPID "Terminate batch job (Y/N)?"" prompt is almost impossible
  batFileKill << "\"" + std::string(TCHAR_TO_UTF8(*dVisEst_bin_path)) + "windows-kill.exe" + "\"" + " -SIGINT %PID:~1%" << std::endl;
  batFileKill.close();

  // Run the kil thing separately since multi-line bat files are buggy and weird
  FInteractiveProcess* CmdProcessKill;
  FString dVisEst_bin_cmd_kill(
    dVisEst_bin_path + "dvd_DvisEst_abstractor_kill.bat"
    );

  CmdProcessKill = new FInteractiveProcess(
    dVisEst_bin_cmd_kill,
    dVisEst_args,
    true,
    false
    );

  CmdProcessKill->OnOutput().BindLambda(
    [=](const FString& outputMessage)
    {
      FString outputString(outputMessage);
      std::string result = std::string(TCHAR_TO_UTF8(*outputString));

      // getting test output is handy
      test_string += "KB->" + result + "\n";
    }
  );

  CmdProcessKill->Launch();

  // Now we can do the real call using the .bat file so we don't have to RX STDERR
  // real command
  std::ofstream batFile(std::string(TCHAR_TO_UTF8(*dVisEst_bin_path)) + "dvd_DvisEst_abstractor.bat");
   // ping until the process is dead
  batFile << ":loop" << std::endl;
  batFile << "ping -n 2 127.0.0.1 >nul" << std::endl; // delay for 1 second
  batFile << "tasklist /FI \"IMAGENAME eq dvd_DvisEst.exe\" 2>NUL | find /I /N \"dvd_DvisEst.exe\">NUL" << std::endl;
  batFile << "if \"%ERRORLEVEL%\"==\"0\" goto loop" << std::endl;
  if(use_generated_throws)
  {
    batFile << "\"" + std::string(TCHAR_TO_UTF8(*dVisEst_bin_path)) << "dvd_DvisEst.exe\" -cr -rm=10 -nc 2> nul";
  }
  else
  {
    batFile << "\"" + std::string(TCHAR_TO_UTF8(*dVisEst_bin_path)) << "dvd_DvisEst.exe\" -cr 2> nul";
  }
  batFile.close();

  FInteractiveProcess* CmdProcess;
  FString dVisEst_bin_cmd(
    dVisEst_bin_path + "dvd_DvisEst_abstractor.bat"
    );

  CmdProcess = new FInteractiveProcess(
    dVisEst_bin_cmd,
    dVisEst_args,
    true,
    false
    );

  CmdProcess->OnOutput().BindLambda(
    [=](const FString& outputMessage)
    {
      FString outputString(outputMessage);
      std::string result = std::string(TCHAR_TO_UTF8(*outputString));

      ParseDvisEstLine(result);

      // getting test output is handy
      test_string += "RB->" + result + "\n";
    }
  );

  CmdProcess->Launch();
}

FString DvisEstInterface::GetTestString()
{
  // build test string
  std::stringstream ss;
  ss << std::endl;
  if(ReadyToThrow)
    ss<< "Ready To Throw!" << std::endl;
  else
    ss << "NOT Ready To Throw!" << std::endl;

  if(NewThrowReady)
    ss << "New Throw RX!" << std::endl;

  // this parsing seems broken
  if(((int)LastDiscIndex) > 0)
    ss << "Last DiscMold = " << (int)LastDiscIndex << std::endl;

  ss << "Last Throw:" << std::endl << 
    "vel_xyz = [" << disc_init_state.lin_vel_xyz[0] << ", " << disc_init_state.lin_vel_xyz[1] << ", " << disc_init_state.lin_vel_xyz[2] << "] m/s" << std::endl <<
    "hyzer = " << (disc_init_state.ang_pos_hps[0]*57.2957795) << " deg" << std::endl <<
    "pitch = " << (disc_init_state.ang_pos_hps[1]*57.2957795) << " deg" << std::endl <<
    "spin_d = " << disc_init_state.ang_vel_hps[2] << " rad/s (" << (disc_init_state.ang_vel_hps[2]*9.54929658) << " rpm)" << std::endl <<
    std::endl;

  // getting test output is handy
  ss << test_string;

  FString new_string(ss.str().c_str());
  return new_string;
}

void DvisEstInterface::GetDiscInitState(disc_init_state_t * new_disc_init_state)
{
  // mark new throw as false on fetch
  NewThrowReady = false;

  memcpy(new_disc_init_state, &disc_init_state, sizeof(disc_init_state_t));
}

bool DvisEstInterface::IsReadyToThrow()
{
  return ReadyToThrow;
}

bool DvisEstInterface::IsNewThrowReady()
{
  return NewThrowReady;
}

uint32 DvisEstInterface::Run()
{
  // Keep processing until we're cancelled through Stop() or we're done,
  // although this thread will suspended for other stuff to happen at the same time
  //while (!bStopThread && !IsComplete())
  //{
    // This is where we would do our expensive threaded processing
    
    RunDvisEst();

  // Return success
  return 0;
}


void DvisEstInterface::Exit()
{
  // Here's where we can do any cleanup we want to
  FString dVisEst_bin_path(
  FPaths::ConvertRelativePathToFull(FPaths::GameSourceDir() + 
  dvd_DvisEst_bin_path));

  // try to kill any dvd_DvisEst instances running
  FString dVisEst_args("");

  // Run the kill file we generated earlier
  FInteractiveProcess* CmdProcessKill;
  FString dVisEst_bin_cmd_kill(
    dVisEst_bin_path + "dvd_DvisEst_abstractor_kill.bat"
    );

  CmdProcessKill = new FInteractiveProcess(
    dVisEst_bin_cmd_kill,
    dVisEst_args,
    true,
    false
    );

  CmdProcessKill->OnOutput().BindLambda(
    [=](const FString& outputMessage)
    {
      FString outputString(outputMessage);
      std::string result = std::string(TCHAR_TO_UTF8(*outputString));

      // getting test output is handy
      test_string += "KB->" + result + "\n";
    }
  );
  
  CmdProcessKill->Launch();
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
