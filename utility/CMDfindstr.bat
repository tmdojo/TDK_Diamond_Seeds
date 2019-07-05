@echo off

set /p INPUT="Enter the search words > "

findstr /n /i %INPUT% *

echo Search end

pause
CMDfindstr.bat