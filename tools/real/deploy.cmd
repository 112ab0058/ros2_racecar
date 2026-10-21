@echo off
rem ============================================================
rem  Windows: deploy the real-car packages to the Orin and build.
rem  Double-click, or in PowerShell / cmd:  tools\real\deploy.cmd
rem
rem  1. git pull (stops if you have local changes)
rem  2. deploy_send.cmd (started only after the pull, so it is always the
rem     new version): tar the 4 real-car packages + tools/real, send them over
rem     ONE ssh, and run tools/real/orin_build.sh on the Orin
rem  wro2026_sim is never sent (CLAUDE.md: do not deploy it to the Jetson).
rem ============================================================
setlocal
cd /d "%~dp0\..\.."

echo [1/2] git pull
git pull --ff-only || goto :fail
call tools\real\deploy_send.cmd || goto :fail

echo.
echo OK. On the Orin:  ros2 launch wro2026_bringup sensors.launch.py
pause
exit /b 0

:fail
echo.
echo FAILED - read the messages above.
pause
exit /b 1
