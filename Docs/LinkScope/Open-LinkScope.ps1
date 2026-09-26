param(
    [ValidateSet('', 'overview', 'remote', 'imu', 'gimbal', 'shoot', 'motors', 'links', 'shoot_stop', 'shoot_stop_baseline')]
    [string]$Preset = '',
    [string]$AppDirectory = 'E:/FOQQ/Linkscope/LinkScope-x86-V1.3.0',
    [string]$ElfPath = '',
    [switch]$CheckOnly
)

# This launcher runs only when the developer opens it. It never flashes a board.
# Settings are copied only after all paths and variable sections have been checked.
$ErrorActionPreference = 'Stop'
try {
    $choices = @('overview', 'remote', 'imu', 'gimbal', 'shoot', 'motors', 'links', 'shoot_stop')
    if ([string]::IsNullOrWhiteSpace($Preset)) {
        Write-Host 'Choose the signals to preselect in LinkScope:'
        Write-Host '  1. overview - start here: command / IMU / gimbal / shoot'
        Write-Host '  2. remote   - both switches, four stick axes and wheel'
        Write-Host '  3. imu      - BMI088 startup and stationary calibration'
        Write-Host '  4. gimbal   - feedback, permit and actual output'
        Write-Host '  5. shoot    - friction / trigger gates and output'
        Write-Host '  6. motors   - CAN buses and the five upper motors'
        Write-Host '  7. links    - NUC and referee receive status'
        Write-Host '  8. shoot_stop - switch / zero output / CAN queue / actual RPM'
        $answer = Read-Host 'Number [1]'
        if ([string]::IsNullOrWhiteSpace($answer)) { $answer = '1' }
        $choice = 0
        if (-not [int]::TryParse($answer, [ref]$choice) -or $choice -lt 1 -or $choice -gt $choices.Count) {
            throw 'Choose one of the displayed numbers.'
        }
        $Preset = $choices[$choice - 1]
    }

    $appRoot = (Resolve-Path -LiteralPath $AppDirectory).Path
    $appExe = Join-Path $appRoot 'LinkScope.exe'
    $presetPath = Join-Path $PSScriptRoot ($Preset + '.ini')
    if (-not (Test-Path -LiteralPath $appExe -PathType Leaf)) {
        throw ('LinkScope.exe was not found: ' + $appExe)
    }
    if (-not (Test-Path -LiteralPath $presetPath -PathType Leaf)) {
        throw ('Preset was not found: ' + $presetPath)
    }
    if ([string]::IsNullOrWhiteSpace($ElfPath)) {
        $projectRoot = Split-Path -Parent (Split-Path -Parent $PSScriptRoot)
        $ElfPath = Join-Path $projectRoot 'build-arm-debug/OMNI_jpl.elf'
    }
    if (-not (Test-Path -LiteralPath $ElfPath -PathType Leaf)) {
        throw ('Build the Debug ELF first, or supply -ElfPath: ' + $ElfPath)
    }
    $resolvedElf = (Resolve-Path -LiteralPath $ElfPath).Path.Replace('\', '/')
    $settings = [System.IO.File]::ReadAllText($presetPath)
    $countMatch = [regex]::Match($settings, '(?m)^VarNum=(\d+)\r?$')
    $sections = [regex]::Matches($settings, '(?m)^\[VarInfo(\d+)\]\r?$')
    if (-not $countMatch.Success -or [int]$countMatch.Groups[1].Value -ne $sections.Count) {
        throw 'VarNum does not match the number of variable sections.'
    }
    for ($i = 0; $i -lt $sections.Count; $i++) {
        if ([int]$sections[$i].Groups[1].Value -ne $i) {
            throw 'Variable section numbers must start at zero and be consecutive.'
        }
    }
    $settings = [regex]::Replace($settings, '(?m)^AxfPath=.*$', [System.Text.RegularExpressions.MatchEvaluator]{ param($match) 'AxfPath=' + $resolvedElf })
    Write-Host ('Preset: ' + $Preset + '; preselected signals: ' + $sections.Count)
    Write-Host ('ELF: ' + $resolvedElf)
    if ($CheckOnly) {
        Write-Host 'CheckOnly: no settings changed and no application started.'
        exit 0
    }

    # An open GUI would overwrite autosave.ini on exit. Ask the user to close it.
    # Do not kill its processes or interfere with any other debugger.
    if (@(Get-Process -Name 'LinkScope' -ErrorAction SilentlyContinue).Count -gt 0) {
        throw 'LinkScope is already running. Close it normally, then run this launcher again.'
    }
    $autosave = Join-Path $appRoot 'autosave.ini'
    $backup = $null
    if (Test-Path -LiteralPath $autosave -PathType Leaf) {
        $backup = Join-Path $appRoot ('autosave.before-preset-' + (Get-Date -Format 'yyyyMMdd-HHmmssfff') + '.ini')
        Copy-Item -LiteralPath $autosave -Destination $backup -ErrorAction Stop
    }
    try {
        [System.IO.File]::WriteAllText($autosave, $settings, [System.Text.UTF8Encoding]::new($false))
        # The user explicitly launches an interactive application here.
        Start-Process -FilePath $appExe -WorkingDirectory $appRoot -WindowStyle Normal -ErrorAction Stop | Out-Null
    }
    catch {
        if ($backup) { Copy-Item -LiteralPath $backup -Destination $autosave -ErrorAction Stop }
        throw
    }
    if ($backup) { Write-Host ('Previous settings: ' + $backup) }
    Write-Host 'Signals are preselected. Check the ELF matches the flashed firmware before sampling.'
}
catch {
    Write-Host ('ERROR: ' + $_.Exception.Message) -ForegroundColor Red
    exit 1
}
