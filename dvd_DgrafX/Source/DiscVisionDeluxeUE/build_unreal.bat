cls
"C:\Program Files\Epic Games\UE_4.25\Engine\Binaries\Win64\UnrealHeaderTool" "C:\DiscVisionDeluxe\dvd_DgrafX\DiscVisionDeluxeUE.uproject" "C:\DiscVisionDeluxe\dvd_DgrafX\Intermediate\Build\Win64\DiscVisionDeluxeUEEditor\Development\DiscVisionDeluxeUEEditor.uhtmanifest" -LogCmds="loginit warning, logexit warning, logdatabase error" -Unattended -WarningsAsErrors -abslog="C:\Users\picard\AppData\Local\UnrealBuildTool\Log_UHT.txt" -installed

"C:\Program Files\Epic Games\UE_4.25\Engine\Build\BatchFiles\Build.bat" DiscVisionDeluxeUEEditor Win64 Development -Project="C:\DiscVisionDeluxe\dvd_DgrafX\DiscVisionDeluxeUE.uproject" -WaitMutex -FromMsBuild

"C:\Program Files\Epic Games\UE_4.25\Engine\Binaries\Win64\UE4Editor.exe" "C:\DiscVisionDeluxe\dvd_DgrafX\DiscVisionDeluxeUE.uproject" DGRange -game