# kill_tree.ps1
# Recursively kills a process and all of its descendants, given the root PID.
# Needed because "gz sim" is launched via pixi -> a ruby.exe dispatcher -> the
# actual sim process, and none of those match a simple "taskkill /IM gz*.exe" -
# killing by the whole process tree (starting from the PID we captured when we
# launched it) is the only reliable way to make sure nothing lingers.

param(
    [Parameter(Mandatory=$true)][int]$RootProcessId
)

function Stop-ProcessTree {
    param([int]$TargetId)
    $children = Get-CimInstance Win32_Process -Filter "ParentProcessId=$TargetId" -ErrorAction SilentlyContinue
    foreach ($child in $children) {
        Stop-ProcessTree -TargetId $child.ProcessId
    }
    Stop-Process -Id $TargetId -Force -ErrorAction SilentlyContinue
}

Stop-ProcessTree -TargetId $RootProcessId
