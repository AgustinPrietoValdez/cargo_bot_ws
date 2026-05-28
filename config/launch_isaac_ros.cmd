@echo off
REM Launch script for Isaac Sim with ROS 2 Discovery Server (cargo_bot)
REM Detects WSL2 IP, patches fastdds_isaac.xml RemoteServer address, launches Isaac Sim

setlocal EnableDelayedExpansion

echo === Cargo Bot: Isaac Sim ROS 2 Launcher ===

REM Detect WSL2 IP address
for /f "usebackq tokens=*" %%i in (`powershell -NoProfile -Command "(wsl.exe -d Ubuntu-22.04 -- hostname -I).Trim().Split(' ')[0]"`) do (
  set "WSL_IP=%%i"
)
if "!WSL_IP!"=="" (
    echo ERROR: Could not detect WSL2 IP address. Is WSL running?
    endlocal & exit /b 1
)
echo WSL2 IP detected: !WSL_IP!

REM Patch ONLY the RemoteServer <address> in fastdds_isaac.xml (not interfaceWhiteList)
set "CONFIG_DIR=%~dp0"
set "XML_FILE=%CONFIG_DIR%fastdds_isaac.xml"

powershell -NoProfile -Command "$p='%XML_FILE%'; $c=[IO.File]::ReadAllText($p); $new=[regex]::Replace($c, '(<RemoteServer[^>]*?>\s*<metatrafficUnicastLocatorList>\s*<locator>\s*<udpv4>\s*<address>)[^<]+', '${1}!WSL_IP!'); [IO.File]::WriteAllText($p, $new, (New-Object System.Text.UTF8Encoding $false))"
if errorlevel 1 (
    echo ERROR: failed to rewrite %XML_FILE%
    endlocal & exit /b 1
)
echo FastDDS XML RemoteServer.address = !WSL_IP!

REM Set environment variables
set ROS_DOMAIN_ID=1
set RMW_IMPLEMENTATION=rmw_fastrtps_cpp
set FASTRTPS_DEFAULT_PROFILES_FILE=%XML_FILE%

echo ROS_DOMAIN_ID=%ROS_DOMAIN_ID%
echo RMW_IMPLEMENTATION=%RMW_IMPLEMENTATION%
echo FASTRTPS_DEFAULT_PROFILES_FILE=%FASTRTPS_DEFAULT_PROFILES_FILE%

REM Source Isaac Sim ROS env (adds bridge DLLs to PATH)
call "C:\isaacsim_51_ga\setup_ros_env.bat"

echo.
echo Environment ready. Launching Isaac Sim...
echo (Make sure Discovery Server is running in WSL2 first!)
echo.

REM Launch Isaac Sim with ROS 2 bridge enabled
"C:\isaacsim_51_ga\isaac-sim.bat" --/isaac/startup/ros_bridge_extension=isaacsim.ros2.bridge

endlocal
