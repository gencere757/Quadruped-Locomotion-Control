@echo off
REM ============================================================
REM Runs one headless simulation: clears leftover gz-sim processes,
REM launches gz-sim headless, resets the world, runs one controller
REM once, then shuts gz-sim down again.
REM
REM Usage:  scripts\run_once.bat [controller]      (default: trot_demo.py)
REM   e.g.  scripts\run_once.bat turn_test.py
REM
REM Do not run two of these at once: two gz-sim instances against
REM the same world1.sdf collide and corrupt each other's telemetry.
REM ============================================================
setlocal
set SCRIPT=%~1
if "%SCRIPT%"=="" set SCRIPT=trot_demo.py

cd /d "%~dp0.."

echo === clearing leftover gz-sim processes ===
powershell -NoProfile -ExecutionPolicy Bypass -File "scripts\kill_gz.ps1"

echo === launching gz-sim headless ===
start "gz-sim-headless" /B pixi run gz sim -s -r --headless-rendering worlds\world1.sdf

REM give gz-sim time to load the world and model
timeout /t 8 /nobreak >nul

echo === resetting world ===
pixi run python src\reset_sim.py

echo === running %SCRIPT% ===
pixi run python src\%SCRIPT%

echo === stopping gz-sim ===
powershell -NoProfile -ExecutionPolicy Bypass -File "scripts\kill_gz.ps1"

echo === done ===
endlocal
