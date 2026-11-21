@echo off
REM ============================================================
REM Resets an already-running gz-sim (GUI or headless) and runs
REM one controller against it.
REM
REM Usage:  scripts\run.bat [controller]      (default: champgait.py)
REM ============================================================
setlocal
set SCRIPT=%~1
if "%SCRIPT%"=="" set SCRIPT=champgait.py

cd /d "%~dp0.."

echo === resetting simulation ===
pixi run python src\reset_sim.py
echo.
echo === running %SCRIPT% ===
pixi run python src\%SCRIPT%
echo.
echo === done ===
endlocal
pause
