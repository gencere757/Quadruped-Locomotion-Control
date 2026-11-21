@echo off
REM ============================================================
REM Unattended test loop: launches gz-sim headless once, then
REM repeatedly resets the world and runs one controller. Keeping
REM gz-sim alive between runs avoids the startup cost and the
REM relaunch collisions described in kill_gz.ps1.
REM
REM Usage:  scripts\run_loop.bat [controller]      (default: champgait_wave.py)
REM Stop with Ctrl+C, then run scripts\kill_gz.ps1 to stop gz-sim.
REM ============================================================
setlocal
set SCRIPT=%~1
if "%SCRIPT%"=="" set SCRIPT=champgait_wave.py

cd /d "%~dp0.."

echo === launching gz-sim headless (once) ===
start "gz-sim-headless" /B pixi run gz sim -s -r --headless-rendering worlds\world1.sdf

REM give gz-sim time to load the world and model
timeout /t 8 /nobreak >nul

:loop
echo.
echo === %DATE% %TIME% - resetting world ===
pixi run python src\reset_sim.py

echo === running %SCRIPT% ===
pixi run python src\%SCRIPT%

timeout /t 3 /nobreak >nul
goto loop
