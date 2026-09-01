# click_and_type.ps1
# Moves the mouse to (X, Y), left-clicks there, then types Message and presses Enter.
# Used by notify_loop.bat to "type a message to Claude" after each sim run finishes,
# by literally clicking into the chat input box and sending keystrokes at the OS level.
#
# CAVEATS (read before wiring this into the loop):
#  - Coordinates are absolute screen pixels. If the window moves, resizes, or the
#    display scaling/resolution changes, these will be wrong and the click will land
#    somewhere else - re-run find_coords.bat to get fresh numbers whenever that happens.
#  - Keep the chat window visible and not covered by other windows while this runs -
#    this does NOT bring the window to the foreground first, it just clicks at a
#    screen coordinate.
#  - SendKeys treats + ^ % ~ ( ) { } [ ] as special characters. Keep Message plain
#    alphanumeric text (the default in notify_loop.bat is safe) - if you want to
#    customize it and need one of those characters, wrap it in {} eg "{+}" for a
#    literal plus sign, or avoid it.

param(
    [Parameter(Mandatory=$true)][int]$X,
    [Parameter(Mandatory=$true)][int]$Y,
    [Parameter(Mandatory=$true)][string]$Message
)

Add-Type @"
using System;
using System.Runtime.InteropServices;
public class MouseSim {
    [DllImport("user32.dll")]
    public static extern bool SetCursorPos(int X, int Y);
    [DllImport("user32.dll")]
    public static extern void mouse_event(uint dwFlags, uint dx, uint dy, uint dwData, UIntPtr dwExtraInfo);
    public const uint MOUSEEVENTF_LEFTDOWN = 0x0002;
    public const uint MOUSEEVENTF_LEFTUP = 0x0004;
}
"@

[MouseSim]::SetCursorPos($X, $Y)
Start-Sleep -Milliseconds 300
[MouseSim]::mouse_event([MouseSim]::MOUSEEVENTF_LEFTDOWN, 0, 0, 0, [UIntPtr]::Zero)
Start-Sleep -Milliseconds 50
[MouseSim]::mouse_event([MouseSim]::MOUSEEVENTF_LEFTUP, 0, 0, 0, [UIntPtr]::Zero)

# give the click a moment to focus the input box before typing
Start-Sleep -Milliseconds 400

Add-Type -AssemblyName System.Windows.Forms
[System.Windows.Forms.SendKeys]::SendWait($Message)
Start-Sleep -Milliseconds 200
[System.Windows.Forms.SendKeys]::SendWait("{ENTER}")
