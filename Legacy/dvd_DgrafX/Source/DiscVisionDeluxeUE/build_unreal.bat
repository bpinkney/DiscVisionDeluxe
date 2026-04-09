REM @echo off

cls

REM generate random number and use sed to modify DiscThrow.cpp with it
REM this will force Unreal to re-compile correctly instead of re-using local stuff like an idiot
shuf -i 1-10000 -n 1 > tmpFile 
set /p RANDNUM= < tmpFile 
del tmpFile

REM echo %RANDNUM%
sed -i "s,const int changevar.*,const int changevar = "%RANDNUM%";,g" "DiscThrow.cpp"

cls

"C:\Program Files\Epic Games\UE_4.25\Engine\Binaries\Win64\UnrealHeaderTool" "C:\DiscVisionDeluxe\dvd_DgrafX\DiscVisionDeluxeUE.uproject" "C:\DiscVisionDeluxe\dvd_DgrafX\Intermediate\Build\Win64\DiscVisionDeluxeUEEditor\Development\DiscVisionDeluxeUEEditor.uhtmanifest" -Od -LogCmds="loginit warning, logexit warning, logdatabase error" -Unattended -WarningsAsErrors -d2OptimizeHugeFunctions -abslog="C:\Users\picard\AppData\Local\UnrealBuildTool\Log_UHT.txt" -installed

"C:\Program Files\Epic Games\UE_4.25\Engine\Build\BatchFiles\Build.bat" DiscVisionDeluxeUEEditor Win64 Development -Project="C:\DiscVisionDeluxe\dvd_DgrafX\DiscVisionDeluxeUE.uproject" -WaitMutex -FromMsBuild -Od -d2OptimizeHugeFunctions