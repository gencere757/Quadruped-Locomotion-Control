@echo off
REM ============================================================
REM Runs the Gazebo sim exactly ONCE: launches gz-sim headless, waits
REM for it to load, resets the world, runs turn_test.py a single time
REM (scripted forward walk -> in-place rotation -> back walk -> combined
REM linear+angular walk, no keyboard needed), then kills every gz-sim-
REM related process and exits. Modeled directly on run_once.bat (which
REM does the same thing for trot_demo.py) - see that file for the fuller
REM history/reasoning behind the kill-before-and-after pattern.
REM
REM Do not run this at the same time as run_loop.bat / run_once.bat /
REM notify_loop.bat - they'd launch a second gz-sim instance against the
REM same world1.sdf and reproduce the collision/corruption kill_gz.ps1
REM was built to prevent.
REM
REM Output: run_log_turn_test.txt in this folder has the full tee'd
REM log (per-phase status lines plus a dx/dy/net-yaw/avg-speed summary
REM printed at the end of each phase) for after-the-fact review.
REM
REM Requires kill_gz.ps1 in the same folder (C:\gz-ws).
REM ============================================================

cd /d C:\gz-ws

echo ============================================================
echo === clearing any leftover gz-sim processes before starting ===
echo ============================================================
powershell -NoProfile -ExecutionPolicy Bypass -File "C:\gz-ws\kill_gz.ps1"

echo ============================================================
echo === launching gz-sim headless ===
echo ============================================================
start "gz-sim-headless" /B pixi run gz sim -s -r --headless-rendering world1.sdf

REM give gz-sim time to finish loading the world/models before anything talks to it
timeout /t 8 /nobreak >nul

echo === resetting world ===
pixi run python reset_sim.py

echo === running turn_test.py (single run: forward / rotate / back / combined) ===
pixi run python turn_test.py

echo === stopping gz-sim ===
powershell -NoProfile -ExecutionPolicy Bypass -File "C:\gz-ws\kill_gz.ps1"

echo === run_turn_test_once.bat complete ===
