@echo off
REM ============================================================
REM Helper: prints your live mouse position every half second so you
REM can hover over the chat input box (in the window where you send
REM messages to Claude) and read off the X,Y to use in notify_loop.bat.
REM
REM Press Ctrl+C to stop once you have the numbers.
REM ============================================================
echo Hover your mouse over the chat input box. Reading position every 0.5s...
echo Press Ctrl+C to stop.
echo.
powershell -NoProfile -Command "Add-Type -AssemblyName System.Windows.Forms; while ($true) { $p = [System.Windows.Forms.Cursor]::Position; Write-Host ('X=' + $p.X + '   Y=' + $p.Y); Start-Sleep -Milliseconds 500 }"
