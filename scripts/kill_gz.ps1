# kill_gz.ps1
# Stops every process whose command line references world1.sdf: the pixi wrapper,
# the ruby.exe gz-tools dispatcher and the gz-sim server itself. Matching on the
# command line (rather than walking a process tree from a captured PID) also catches
# gz-sim processes that were orphaned when the pixi wrapper exited early. Leaving one
# alive and launching a second instance against the same world corrupts telemetry.

$procs = Get-CimInstance Win32_Process | Where-Object { $_.CommandLine -and $_.CommandLine -like '*world1.sdf*' }

if ($procs) {
    foreach ($p in $procs) {
        Write-Host "Killing PID $($p.ProcessId): $($p.CommandLine)"
        Stop-Process -Id $p.ProcessId -Force -ErrorAction SilentlyContinue
    }
    # give Windows a moment to release the process/port before the next launch
    Start-Sleep -Milliseconds 1500
} else {
    Write-Host "No matching gz-sim processes found."
}
