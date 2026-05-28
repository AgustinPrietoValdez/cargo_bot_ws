@echo off
REM ====================================================================
REM Launches standalone_lidar_publisher.py with the SAME ROS env that the
REM (working) run_rtx_lidar_standalone.cmd uses.  This is the hybrid
REM option C from the third research agent's report: load the user's
REM scene.usda into a standalone SimulationApp and publish /scan_py from
REM the existing OmniLidar prim, bypassing the polluted Hydra render
REM product cache that the GUI workflow leaves behind.
REM
REM PREREQUISITE: Close the Isaac Sim GUI window before running this.
REM
REM USAGE (Windows cmd / PowerShell):
REM   C:\Users\agusp\cargo_bot_ws\run_standalone_lidar.cmd
REM ====================================================================

setlocal EnableDelayedExpansion

echo === cargo_bot_ws standalone lidar publisher ===

REM ------------------------------------------------------------------
REM Detect WSL2 IP (the Discovery Server endpoint)
REM ------------------------------------------------------------------
for /f "usebackq tokens=*" %%i in (`powershell -NoProfile -Command "(wsl.exe -d Ubuntu-22.04 -- hostname -I).Trim().Split(' ')[0]"`) do (
  set "WSL_IP=%%i"
)
if "!WSL_IP!"=="" (
    echo ERROR: Could not detect WSL2 IP.  Is WSL running?
    endlocal & exit /b 1
)
echo WSL2 IP detected: !WSL_IP!

REM ------------------------------------------------------------------
REM Patch fastdds_isaac.xml RemoteServer address so Isaac (CLIENT) finds
REM the WSL Discovery Server (SERVER 127.0.0.1:11811 inside WSL,
REM portproxy'd to WSL_IP:11811 from Windows).  Same regex as the
REM working run_rtx_lidar_standalone.cmd.
REM ------------------------------------------------------------------
set "XML_FILE=C:\Users\agusp\cargo_bot_ws\config\fastdds_isaac.xml"
powershell -NoProfile -Command "$p='%XML_FILE%'; $c=[IO.File]::ReadAllText($p); $new=[regex]::Replace($c, '(<RemoteServer[^>]*?>\s*<metatrafficUnicastLocatorList>\s*<locator>\s*<udpv4>\s*<address>)[^<]+', '${1}!WSL_IP!'); [IO.File]::WriteAllText($p, $new, (New-Object System.Text.UTF8Encoding $false))"

REM ------------------------------------------------------------------
REM ROS 2 env -- matches the working standalone run.
REM ------------------------------------------------------------------
set ROS_DOMAIN_ID=1
set RMW_IMPLEMENTATION=rmw_fastrtps_cpp
set FASTRTPS_DEFAULT_PROFILES_FILE=%XML_FILE%
echo ROS_DOMAIN_ID=%ROS_DOMAIN_ID%
echo RMW_IMPLEMENTATION=%RMW_IMPLEMENTATION%
echo FASTRTPS_DEFAULT_PROFILES_FILE=%FASTRTPS_DEFAULT_PROFILES_FILE%

call "C:\isaacsim_51_ga\setup_ros_env.bat"

echo.
echo Launching standalone_lidar_publisher.py ...
echo (takes 30-90s to load the scene before /scan_py appears)
echo Ctrl-C in this terminal to stop.
echo.

"C:\isaacsim_51_ga\python.bat" "C:\Users\agusp\cargo_bot_ws\src\cargo_bot_simulation\scripts\standalone_lidar_publisher.py"

endlocal
