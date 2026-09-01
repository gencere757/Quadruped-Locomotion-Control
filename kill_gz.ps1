# kill_gz.ps1
# Kills every process whose command line references world1.sdf - covers the pixi
# wrapper, the ruby.exe gz-tools dispatcher, and the actual gz-sim server process,
# regardless of parent/child relationships.
#
# Replaces the earlier PID-tree-based kill (kill_tree.ps1): that approach captured
# the PID of the "pixi" wrapper process and walked its children, but if pixi exits
# shortly after spawning the real gz-sim process (common for wrapper launchers), the
# real process gets orphaned/reparented and stops showing up as a "child" of the
# tracked PID - so it never actually got killed. The next loop iteration then
# launched a SECOND gz-sim instance while the first was still alive, and the two
# colliding physics servers is the likely cause of the corrupted telemetry seen
# (velocities in the tens of m/s, accelerations in the thousands).

$procs = Get-CimInstance Win32_Process | Where-Object { $_.CommandLine -and $_.CommandLine -like '*world1.sdf*' }

if ($procs) {
    foreach ($p in $procs) {
        Write-Host "Killing PID $($p.ProcessId): $($p.CommandLine)"
        Stop-Process -Id $p.ProcessId -Force -ErrorAction SilentlyContinue
    }
    # give Windows a moment to actually release the process/port before the next launch
    Start-Sleep -Milliseconds 1500
} else {
    Write-Host "No matching gz-sim processes found."
}
