@echo off
REM Combined bring-up: Discovery Server (WSL) + Isaac Sim (Windows)
REM Opens a separate window for the DS so its logs stay visible.
REM Each piece can be closed independently:
REM   - DS window: close it or Ctrl+C inside
REM   - Isaac:     close Isaac normally

setlocal

echo === Cargo Bot: Full Stack Launcher ===
echo.
echo [1/2] Starting Discovery Server in WSL (separate window)...

start "Cargo Bot Discovery Server" cmd /k "wsl.exe -d Ubuntu-22.04 -- bash /mnt/c/Users/agusp/cargo_bot_ws/config/start_discovery_server.sh"

echo       Waiting 4s for fastdds to bind to port 11811...
timeout /t 4 /nobreak >nul

echo.
echo [2/2] Launching Isaac Sim in this window...
echo.
call "%~dp0launch_isaac_ros.cmd"

echo.
echo Isaac Sim exited. Discovery Server window is still open -- close it manually if you're done.
endlocal
