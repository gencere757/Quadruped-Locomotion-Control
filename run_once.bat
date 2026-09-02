@echo off
REM ============================================================
REM Runs the Gazebo sim exactly ONCE: launches gz-sim headless, waits
REM for it to load, resets the world, runs trot_demo.py a single
REM time, then kills every gz-sim-related process and exits.
REM
REM v5: back to trot_demo.py - v4 (switching to champgait_wave.py) was reverted within minutes, before
REM a single run even happened. Do NOT run notify_loop_turn.bat at the same time as this - two gz-sim
REM instances against the same world1.sdf collide/corrupt each other.
REM
REM v4: switched target back to champgait_wave.py - going back to the statically-stable wave gait as
REM the first thing to get walking on the new (much heavier) CAD export, instead of continuing on the
REM dynamic trot gait (manual_control.py/turn_test.py) this whole debugging thread had moved to.
REM
REM v3: switched target script to trot_demo.py (the crawl gait's own AUTOPING loop was paused
REM while that one ran instead - do not run both loops at once, they'd launch two gz-sim instances
REM against the same world1.sdf and reproduce the exact collision/corruption kill_gz.ps1 was built
REM to prevent).
REM
REM v2: kill step now matches by command line (world1.sdf) instead of
REM walking a process tree from a captured PID - the PID-tree approach
REM missed the real gz-sim process when the pixi wrapper exited early
REM and orphaned it, leaving it running and colliding with the next
REM launch (this corrupted telemetry on a real run - vel/accel spiking
REM into the thousands). See kill_gz.ps1 for the full explanation.
REM
REM Unlike run_loop.bat (which keeps gz-sim alive across iterations
REM for speed), this pays the ~8s gz-sim startup cost every call - a
REM deliberate trade-off for a clean "one run, fully closed" unit that
REM notify_loop.bat can call repeatedly without any leftover state.
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

echo === running trot_demo.py (single run) ===
pixi run python trot_demo.py

echo === stopping gz-sim ===
powershell -NoProfile -ExecutionPolicy Bypass -File "C:\gz-ws\kill_gz.ps1"

echo === run_once.bat complete ===
