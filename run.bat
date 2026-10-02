@echo off
setlocal
cd /d "%~dp0"
where py >nul 2>nul
if %errorlevel%==0 (
  start "WalkRoute" py -3 server.py
) else (
  start "WalkRoute" python server.py
)
timeout /t 2 /nobreak >nul
start http://127.0.0.1:8765
