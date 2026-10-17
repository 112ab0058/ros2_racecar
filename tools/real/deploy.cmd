@echo off
rem ============================================================
rem  Windows: deploy the real-car packages to the Orin and build.
rem  Double-click, or in PowerShell / cmd:  tools\real\deploy.cmd
rem
rem  1. git pull (stops if you have local changes)
rem  2. pack the 4 real-car packages with tar and send them over ONE ssh
rem     (one password prompt)
rem  3. on the Orin, tools/real/orin_build.sh replaces the old copies in
rem     ~/racecar_ws/src and runs colcon build
rem  wro2026_sim is never sent (CLAUDE.md: do not deploy it to the Jetson).
rem ============================================================
setlocal
set ORIN=user2@192.168.0.15
cd /d "%~dp0\..\.."

echo [1/2] git pull
git pull --ff-only || goto :fail

echo [2/2] send to %ORIN% and build (enter the Orin password if asked)
tar --exclude=__pycache__ -cf - tools/real/orin_build.sh src/wro2026_base_r550 src/wro2026_lidar_n10p src/wro2026_description src/wro2026_bringup | ssh %ORIN% "rm -rf ~/racecar_ws/.deploy && mkdir -p ~/racecar_ws/.deploy && tar -xf - -C ~/racecar_ws/.deploy && bash ~/racecar_ws/.deploy/tools/real/orin_build.sh"
if errorlevel 1 goto :fail

echo.
echo OK. On the Orin:  ros2 launch wro2026_bringup sensors.launch.py
pause
exit /b 0

:fail
echo.
echo FAILED - read the messages above.
pause
exit /b 1
