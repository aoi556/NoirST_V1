$ErrorActionPreference = 'Stop'
# Parameter/behaviour reference checks. Does not execute the target C binary.
$root = Join-Path $PSScriptRoot '..'
$params = Get-Content -Raw (Join-Path $root 'Core/Inc/params.h')
function Number([string]$name) {
    $m = [regex]::Match($params, "(?m)^#define\s+$name\s+\(?([0-9.]+)")
    if (!$m.Success) { throw "Missing parameter $name" }
    [double]::Parse($m.Groups[1].Value, [Globalization.CultureInfo]::InvariantCulture)
}
$leftTarget = Number 'WALL_TARGET_L_MM'
$rightTarget = Number 'WALL_TARGET_R_MM'
$maximum = Number 'WALL_FOLLOW_MAX_MM'
$deadband = Number 'WALL_FOLLOW_DEADBAND_MM'
$kp = Number 'WALL_FOLLOW_KP_DEG_S_PER_MM'
$cap = Number 'WALL_FOLLOW_MAX_DEG_S'
$minWheel = Number 'WALL_FOLLOW_MIN_WHEEL_MM_S'
$tread = Number 'TREAD_MM'
foreach ($name in @('L','FL','FR','R')) {
    $limit = Number "WALL_BASE_${name}"
    if ($limit -lt 0 -or $limit -gt 4095) { throw 'Wall threshold outside ADC range' }
}
if ([Math]::Abs($leftTarget - (30 + (1286-960)*10/340)) -gt 0.001 -or
    [Math]::Abs($rightTarget - (40 + (555-550)*10/102)) -gt 0.001) { throw 'Centre conversion mismatch' }
if ($maximum -gt 90 -or $maximum -lt [Math]::Max($leftTarget,$rightTarget) -or
    $deadband -lt 0 -or $kp -le 0 -or $cap -le 0 -or $minWheel -le 0) { throw 'Invalid control parameters' }
function Omega([double]$left, [double]$right, [double]$speed, [bool]$leftValid=$true, [bool]$rightValid=$true) {
    $l = $leftValid -and $left -le $maximum
    $r = $rightValid -and $right -le $maximum
    $errorMm = 0.0
    if ($l) { $errorMm += $left - $leftTarget }
    if ($r) { $errorMm -= $right - $rightTarget }
    if ($l -and $r) { $errorMm /= 2 }
    if ($errorMm -gt $deadband) { $errorMm -= $deadband }
    elseif ($errorMm -lt -$deadband) { $errorMm += $deadband }
    else { $errorMm = 0 }
    $limit = [Math]::Min($cap, [Math]::Max(0, $speed-$minWheel)*360/($tread*[Math]::PI))
    [Math]::Max(-$limit, [Math]::Min($limit, $errorMm*$kp))
}
if ((Omega $leftTarget $rightTarget 200) -ne 0) { throw 'Centred robot must go straight' }
if ((Omega ($leftTarget-5) ($rightTarget+5) 200) -ge 0) { throw 'Near left wall must turn right' }
if ((Omega ($leftTarget+5) ($rightTarget-5) 200) -le 0) { throw 'Near right wall must turn left' }
if ((Omega ($leftTarget-5) 90 200 $true $false) -ge 0) { throw 'Left-only wall sign' }
if ((Omega 90 ($rightTarget-5) 200 $false $true) -le 0) { throw 'Right-only wall sign' }
if ((Omega 10 10 200 $false $false) -ne 0) { throw 'Clamped invalid samples must not steer' }
if ((Omega 90 90 200) -ne 0) { throw 'Distant walls must not steer' }
if ((Omega ($leftTarget+0.5) $rightTarget 200) -ne 0) { throw 'Deadband' }
foreach ($speed in @(-30,0,20,30,31,50,200,525)) {
    foreach ($left in @(10,20,40,60,70,90)) {
        foreach ($right in @(10,20,40,60,70,90)) {
            $omega = Omega $left $right $speed
            if ([Math]::Abs($omega) -gt $cap+0.0001) { throw 'Angular cap exceeded' }
            if ($speed -le $minWheel -and $omega -ne 0) { throw 'Low speed steering' }
            if ($speed -gt $minWheel) {
                $slowWheel = $speed - [Math]::Abs($omega)*$tread*[Math]::PI/360
                if ($slowWheel -lt $minWheel-0.0001) { throw 'Wheel speed below limit' }
            }
        }
    }
}
# Integration guard: raw wall channels may only be used inside the sensor layer.
foreach ($file in Get-ChildItem (Join-Path $root 'Core/Src') -Filter '*.c') {
    if ($file.Name -eq 'sensor.c') { continue }
    $source = Get-Content -Raw $file.FullName
    if ($source -match '(?:\.|->)ad_(?:fl|fr|l|r)\b|\b(?:WALL_BASE_\w+|CTRL_BASE_\w+|CTRL_CONT|BASE_L|BASE_R)\b') {
        throw "Raw wall control reference in $($file.Name)"
    }
}
Write-Output 'PASS: centre conversion, steering signs, single/no wall, deadband, angular/wheel limits (reference calculation), no raw wall consumers outside sensor.c.'
