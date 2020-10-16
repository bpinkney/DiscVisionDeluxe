@Echo off & SetLocal EnableDelayedExpansion
set "PID="
for /f "tokens=2" %%A in ('tasklist ^| findstr /i "dvd_DvisEst.exe" 2^>NUL') do @Set "PID=!PID!,%%A"
if defined PID Echo dvd_DvisEst.exe has PID(s) %PID:~1%
"C:/DiscVisionDeluxe/Binaries/dvd_DvisEst/2020-10-11/windows-kill.exe" -SIGINT %PID:~1%
