@echo off
setlocal
cd /d "%~dp0"

echo Checking port 8765...
for /f "tokens=5" %%a in ('netstat -ano ^| findstr :8765 ^| findstr LISTENING') do (
    echo Stopping old WalkRoute process %%a...
    taskkill /PID %%a /F >nul 2>&1
)

where py >nul 2>nul
if %errorlevel%==0 (
  start "WalkRoute" py -3 server.py
) else (
  start "WalkRoute" python server.py
)

timeout /t 2 /nobreak >nul
start http://127.0.0.1:8765
