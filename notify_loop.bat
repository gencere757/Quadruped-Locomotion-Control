@echo off
REM ============================================================
REM Runs run_once.bat repeatedly. After each completed sim run, clicks
REM the chat input box at CHAT_X,CHAT_Y and types MESSAGE + Enter - so
REM Claude gets pinged with a real message every time a run finishes,
REM with no scheduled check-in needed on Claude's side.
REM
REM SETUP, IN ORDER:
REM   1. Run find_coords.bat, hover your mouse over the chat input box
REM      (where you type messages to Claude), and read the X,Y off the
REM      console. Fill those two numbers in below (CHAT_X / CHAT_Y).
REM   2. Test click_and_type.ps1 BY ITSELF first (see example below)
REM      before wiring it into this loop, to confirm the click lands in
REM      the right spot and the message actually sends:
REM        powershell -ExecutionPolicy Bypass -File click_and_type.ps1 -X 800 -Y 900 -Message "test ping"
REM   3. Keep the chat window visible and unobstructed while this loop
REM      runs - it clicks a fixed screen coordinate, it does not find
REM      or focus the window first.
REM   4. If you resize/move the window or change display scaling, the
REM      coordinates go stale - rerun find_coords.bat to get new ones.
REM
REM v7: switched target back to trot_demo.py (run_once.bat's own v7 note has the full reasoning) -
REM message text updated to match. Kept the 20s gap from v6 regardless of which script is the target.
REM
REM v6: widened the post-run gap again, 10s -> 20s. Evidence: a burst of 3 back-to-back runs all
REM aborted fast and hard (t=6-16s into gait, sharp pitch dips, one with an implausible +66.8deg yaw
REM drift) - a cluster of differently-shaped failures all at once looks less like the gait's own
REM marginal stability and more like a repeat of the exact corrupted-relaunch pattern v3's 3s->10s
REM widening was already built to prevent (run_once.bat's own v2 note: an insufficiently-recovered
REM gz-sim relaunch corrupts telemetry - vel/accel spiking, in that case). A fast-failing run makes
REM each run_once.bat cycle SHORTER, so gz-sim gets relaunched more often in a burst right when this
REM is most likely to bite - a plausible self-reinforcing cascade. 20s costs little against a run that
REM otherwise takes over a minute, and directly attacks the one hypothesis with real precedent here.
REM
REM v5: switched to run_once.bat's new target (champgait_wave.py) - message text updated to match.
REM This is a MUCH longer run than trot_demo.py's (N_CYCLES=3 * GAIT_PERIOD=24s = 72s of gait alone,
REM plus drop-settle/crouch/ease-in/ease-out - expect well over a minute per run before the ping fires).
REM
REM v4: back to trot_demo.py - v3 (switching to champgait_wave.py) was reverted within minutes,
REM before a single run even happened. Kept the widened 10s gap from v3 (was 3s originally) since
REM that fix is worth keeping regardless of which script is the target.
REM
REM v3: switched to run_once.bat's then-target (champgait_wave.py) - message text updated to
REM match. Only run ONE of the gz-sim loops at a time (this vs. notify_loop_turn.bat) - see
REM run_once.bat's own notes. Also widened the gap below from 3s to 10s (matches
REM notify_loop_turn.bat's own fix) - back-to-back gz-sim launches with too little gap between them
REM caused abnormal early deaths there.
REM
REM v2: pointed at run_once.bat's previous target (trot_demo.py) - message text updated to match.
REM Only run ONE of these loops at a time (trot vs. the crawl gait) - see run_once.bat's v3 note.
REM
REM Stop the loop any time with Ctrl+C in this window.
REM ============================================================

set CHAT_X=158
set CHAT_Y=723
set MESSAGE=AUTOPING: trot run finished, please check run_log_trot.txt

:loop
call C:\gz-ws\run_once.bat

echo === pinging chat window ===
powershell -NoProfile -ExecutionPolicy Bypass -File "C:\gz-ws\click_and_type.ps1" -X %CHAT_X% -Y %CHAT_Y% -Message "%MESSAGE%"

timeout /t 20 /nobreak >nul
goto loop
