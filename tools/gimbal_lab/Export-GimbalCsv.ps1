param(
    [ValidateSet('yaw','pitch')][string]$Axis = 'yaw',
    [ValidateRange(1,3600)][int]$LastSeconds = 30,
    [string]$Output = ''
)
$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path -Parent (Split-Path -Parent $PSScriptRoot)
$workspace = Split-Path -Parent $projectRoot
$reader = Join-Path $workspace '.agents/skills/mcu-variables/scripts/read.js'
if (-not (Test-Path -LiteralPath $reader -PathType Leaf)) {
    throw 'Open OMNI_freeRTOS in VS Code and enable EmberProbe Agent Skills first.'
}
if (-not $Output) {
    $runName = Get-Date -Format 'yyyyMMdd_HHmmss'
    $Output = "OMNI_jpl/tools/gimbal_lab/data/${Axis}_${runName}.csv"
}
$motorIndex = if ($Axis -eq 'yaw') { 4 } else { 5 }
$variables = "debug_gimbal.${Axis}_target_deg,debug_ins.${Axis}_deg,debug_gimbal.${Axis}_output,debug_motor[$motorIndex].current_raw,debug_gimbal.mode,debug_gimbal.faults"
& node $reader --workspace $workspace --export-csv --variables $variables --last $LastSeconds --output $Output
if ($LASTEXITCODE -ne 0) { throw 'Export failed. Read the EmberProbe JSON diagnostic above.' }
