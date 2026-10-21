@echo off
rem Called by deploy.cmd after git pull. Sends the 4 real-car packages and
rem tools/real to the Orin over one ssh and builds them there.
setlocal
set ORIN=user2@192.168.0.15
cd /d "%~dp0\..\.."
echo [2/2] send to %ORIN% and build (enter the Orin password if asked)
tar --exclude=__pycache__ -cf - tools/real src/wro2026_base_r550 src/wro2026_lidar_n10p src/wro2026_description src/wro2026_bringup src/wro2026_sim_r550 | ssh %ORIN% "rm -rf ~/racecar_ws/.deploy && mkdir -p ~/racecar_ws/.deploy && tar -xf - -C ~/racecar_ws/.deploy && bash ~/racecar_ws/.deploy/tools/real/orin_build.sh"
exit /b %errorlevel%
