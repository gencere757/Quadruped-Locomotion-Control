@echo off
cd /d C:\gz-ws
echo === resetting simulation ===
pixi run python reset_sim.py
echo.
echo === running champgait.py ===
pixi run python champgait.py
echo.
echo === done ===
pause
