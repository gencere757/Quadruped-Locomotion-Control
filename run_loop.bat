@echo off
REM ==================================================================================
REM Headless, unattended test loop for champgait_wave.py - no GUI, no clicking.
REM v2 FIX: v1 relaunched gz-sim every iteration and tried to taskkill it after -
REM that kill never actually worked (the gz CLI's dispatcher runs as ruby.exe under
REM the hood, not gz*.exe, so the old world process never died), which made every
REM iteration after the first fail with "Another world of the same name is running"
REM and silently talk to run #1's stale, already-collapsed world instead (that's why
REM roll=180deg / z=0.055 / zero displacement showed up on runs 2+ - not a physics
REM bug, this script's bug).
REM
REM v2 launches gz-sim ONCE, then just calls reset_sim.py (already resets the world
REM via its own gz-transport WorldControl call) between runs - exactly how the
REM original manual workflow worked (open gazebo once, click reset+run repeatedly).
REM No process-killing, no guessing binary names.
REM
REM Stop it any time with Ctrl+C in this window (then close the gz-sim window/task
REM manually if you want the server gone too).
REM
REM FIRST RUN CAUTION: the "gz sim -s -r ..." line below is my best guess at the
REM headless-launch syntax for your gz-sim=10.* install - I can't test it myself, so
REM please watch the first few lines of output. If IT errors (bad flag, binary not
REM found, etc.) paste the error back and I'll fix the command.
REM ==================================================================================

cd /d C:\gz-ws

echo ============================================================
echo === launching gz-sim headless (once) ===
echo ============================================================
start "gz-sim-headless" /B pixi run gz sim -s -r --headless-rendering world1.sdf

REM give gz-sim time to finish loading the world/models before anything talks to it
timeout /t 8 /nobreak >nul

:loop
echo.
echo ============================================================
echo === %DATE% %TIME% - resetting world ===
echo ============================================================
pixi run python reset_sim.py

echo === running champgait_wave.py ===
pixi run python champgait_wave.py

REM brief pause so Claude has a moment to read the log / ship an edit before the
REM next iteration picks up whatever is currently on disk
timeout /t 3 /nobreak >nul

goto loop
