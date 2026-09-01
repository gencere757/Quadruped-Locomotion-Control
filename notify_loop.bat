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
REM v2: pointed at run_once.bat's new target (trot_demo.py) - message text updated to match.
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

timeout /t 3 /nobreak >nul
goto loop
