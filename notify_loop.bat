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

timeout /t 10 /nobreak >nul
goto loop
