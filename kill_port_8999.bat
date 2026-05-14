@echo off
echo Checking for processes using port 8999...
netstat -ano | findstr :8999
echo.
echo Finding process ID...
for /f "tokens=5" %%a in ('netstat -ano ^| findstr :8999 ^| findstr LISTENING') do (
    echo Killing process ID: %%a
    taskkill /F /PID %%a
)
echo Done.
pause
