:loop
ping -n 2 127.0.0.1 >nul
tasklist /FI "IMAGENAME eq dvd_DvisEst.exe" 2>NUL | find /I /N "dvd_DvisEst.exe">NUL
if "%ERRORLEVEL%"=="0" goto loop
"C:/DiscVisionDeluxe/Binaries/dvd_DvisEst/2020-10-11/dvd_DvisEst.exe" -cr -rm=3 -nc 2> nul