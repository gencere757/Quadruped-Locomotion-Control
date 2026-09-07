@echo off
REM ============================================================
REM Runs the Gazebo sim exactly ONCE: launches gz-sim headless, waits
REM for it to load, resets the world, runs champgait_wave.py a single
REM time, then kills every gz-sim-related process and exits.
REM
REM v7: switched target back to trot_demo.py. champgait_wave.py's wave gait is now genuinely solid -
REM v16-v18 tracked down and fixed the pitch-correction gain mismatch (joints are 5.3x stiffer than
REM this whole codebase's control loops were tuned for) and an under-powered yaw correction; the wave
REM gait now completes its full sequence cleanly, repeatably, with yaw drift converging to ~5deg net
REM instead of climbing unbounded. Moving on to give the trot gait the same treatment - trot_demo.py's
REM own CORRECTION_FRACTION comment already independently diagnosed the identical symptom (theta
REM ringing between -20/+18deg, pitch oscillating with no sign of decaying) and even correctly
REM identified the mechanism (heavier/stiffer joints -> more effective loop delay -> P-gain that's
REM fine on paper rings in practice) - same root cause as the wave gait, not yet fixed here. UNLIKE the
REM wave gait, trot only has 2 feet planted at a time and, per this file's own v6 note, has NO real
REM foot-placement balance control - the reactive tilt correction IS the only thing keeping it upright.
REM So the wave gait's fix (cut the gain ~5.3x) is not a given win here: cutting too far could remove
REM the only thing holding the trot up rather than just quieting an oscillation on top of otherwise-
REM adequate static stability. Watching fresh telemetry before touching CORRECTION_FRACTION/
REM PITCH_RATE_DAMPING here, not assuming the wave gait's numbers transfer.
REM
REM v6: switched target to champgait_wave.py - trot_demo.py's post-gait-fall bugs got fixed (speed-
REM gated deceleration/handoff, flat-foot planting, speed-gated final wait), but the underlying trot
REM gait itself still walks unsteadily on this much-heavier CAD export because the only thing keeping
REM it upright is a reactive whole-body-tilt correction loop, not real foot-placement balance control.
REM champgait_wave.py's statically-stable wave gait (always >=3 feet planted) sidesteps that problem
REM entirely - one earlier test run showed pitch/roll both under ~2deg for 25+ continuous seconds,
REM dramatically steadier than anything the trot scripts produced. Ported the same class of end-of-run
REM fixes into it (ease_out_of_gait, speed-gated final tail) before making it the primary target.
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
