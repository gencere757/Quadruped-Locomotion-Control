@echo off
REM ============================================================
REM Runs run_turn_test_once.bat repeatedly. After each completed sim run,
REM clicks the chat input box at CHAT_X,CHAT_Y and types MESSAGE + Enter -
REM so Claude gets pinged with a real message every time a turn_test run
REM finishes, with no scheduled check-in needed on Claude's side. This is
REM the same mechanism as notify_loop.bat (which does this for trot_demo.py
REM via run_once.bat) - see that file for the fuller original notes.
REM
REM IMPORTANT: this loop does NOT wait for Claude to push a fix between
REM iterations - it goes straight back to :loop and starts the next run
REM immediately. That's fine (each run is a fresh `pixi run python
REM turn_test.py` process, so it picks up whatever's on disk the moment
REM it starts), but it means a fix pushed *while* a run is already in
REM progress won't apply until the run after that. If you want a fix to
REM apply to the very next run, it helps to leave a few seconds of gap
REM (the loop already has one - see the timeout below) or just watch for
REM the "logging this run to run_log_turn_test.txt" line before assuming
REM a specific run used the new values.
REM
REM SETUP, IN ORDER (skip re-checking if you already verified these
REM recently and haven't moved/resized the chat window since):
REM   1. Run find_coords.bat, hover your mouse over the chat input box,
REM      and read the X,Y off the console. Fill those into CHAT_X/CHAT_Y
REM      below if they've changed.
REM   2. Test click_and_type.ps1 BY ITSELF first before trusting this loop:
REM        powershell -ExecutionPolicy Bypass -File click_and_type.ps1 -X 800 -Y 900 -Message "test ping"
REM   3. Keep the chat window visible and unobstructed while this loop
REM      runs - it clicks a fixed screen coordinate, it does not find or
REM      focus the window first. Resizing/moving the window or changing
REM      display scaling makes the coordinates stale.
REM
REM Only run ONE gz-sim loop at a time - do not run this alongside
REM notify_loop.bat / run_loop.bat / run_once.bat, they'd all launch a
REM second gz-sim instance against the same world1.sdf.
REM
REM Stop the loop any time with Ctrl+C in this window.
REM ============================================================

set CHAT_X=158
set CHAT_Y=723
set MESSAGE=AUTOPING: turn_test run finished, please check run_log_turn_test.txt

:loop
call C:\gz-ws\run_turn_test_once.bat

echo === pinging chat window ===
powershell -NoProfile -ExecutionPolicy Bypass -File "C:\gz-ws\click_and_type.ps1" -X %CHAT_X% -Y %CHAT_Y% -Message "%MESSAGE%"

REM Widened from 3s to 10s: 2 of the first 3 automated runs died abnormally (one stubbed out after
REM 2 log lines, one cut off mid-phase with no abort/exit message at all) rather than failing the
REM gait cleanly - looks like back-to-back gz-sim launches not leaving enough gap for the previous
REM instance to fully release before the next one starts, which is exactly the collision run_once.bat's
REM own header comment warns about. If runs still die abnormally with this wider gap, that's the next
REM thing to widen further (or check Task Manager for lingering gz-sim/pixi processes after kill_gz.ps1
REM runs - it may not be catching everything).
timeout /t 10 /nobreak >nul
goto loop
