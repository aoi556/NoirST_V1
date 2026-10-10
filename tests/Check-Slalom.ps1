$ErrorActionPreference = 'Stop'
$params = Get-Content -Raw -LiteralPath (Join-Path $PSScriptRoot '../Core/Inc/params.h')
function Read-Number([string]$name) {
    $match = [regex]::Match($params, "(?m)^#define\s+$name\s+\(?([0-9.]+)")
    if (!$match.Success) { throw "Missing numeric parameter: $name" }
    return [double]::Parse($match.Groups[1].Value, [Globalization.CultureInfo]::InvariantCulture)
}
foreach ($prefix in @("SLALOM", "EXPLORE_SLALOM")) {
Write-Output $prefix
$speed = Read-Number "${prefix}_SPEED_MM_S"
$minimum = Read-Number 'SLALOM_MIN_SPEED_MM_S'
$targetAngle = Read-Number "${prefix}_TURN_ANGLE_DEG"
$hold = Read-Number "${prefix}_HOLD_MS"
$ramp = 2 * $hold
$tread = Read-Number 'TREAD_MM'
$halfCell = Read-Number 'HALF_CELL_MM'
$pulseDistance = (Read-Number 'WHEEL_DIAMETER_MM') * [Math]::PI / (Read-Number 'PULSE_PER_REV')
if ($ramp -le 0 -or $hold -lt 0 -or $speed -lt $minimum) { throw 'Invalid motion parameters' }
# Verify the geometry, not just the angular area: a 90-degree command must
# also join the perpendicular cell boundary at its centre.
foreach ($side in @('R', 'L')) {
    $entry = Read-Number "${prefix}_${side}_ENTRY_MM"
    $exit = Read-Number "${prefix}_${side}_EXIT_MM"
    $gain = Read-Number "${prefix}_${side}_ANGLE_GAIN"
    $sign = if ($side -eq 'R') { 1.0 } else { -1.0 }
    $x = 0.0; $y = $entry; $angle = 0.0
    $minWheel = $speed; $maxWheel = $speed
    $total = 2 * $ramp + $hold
    for ($tick = 0; $tick -lt $total; ++$tick) {
        $t = $tick + 0.5
        $shape = [Math]::Min(1.0, [Math]::Min($t / $ramp, ($total - $t) / $ramp))
        $omega = $gain * $targetAngle * 1000.0 / ($ramp + $hold) * $shape * [Math]::PI / 180.0
        $minWheel = [Math]::Min($minWheel, $speed - $omega * $tread / 2)
        $maxWheel = [Math]::Max($maxWheel, $speed + $omega * $tread / 2)
        $da = $sign * $omega * 0.001
        $x += $speed * 0.001 * [Math]::Sin($angle + $da / 2)
        $y += $speed * 0.001 * [Math]::Cos($angle + $da / 2)
        $angle += $da
        if ($tick + 1 -eq $ramp -or $tick + 1 -eq $ramp + $hold) {
            $expected = if ($tick + 1 -eq $ramp) { $targetAngle / 3 } else { 2 * $targetAngle / 3 }
            if ([Math]::Abs($angle * 180 / [Math]::PI - $sign * $gain * $expected) -gt 0.001) { throw "Phase angle mismatch" }
        }
    }
    $x += $exit * [Math]::Sin($angle)
    $y += $exit * [Math]::Cos($angle)
    $degrees = $angle * 180.0 / [Math]::PI
    if ([Math]::Abs($degrees - $sign * $targetAngle) -gt 0.01) { throw "$side angle mismatch: $degrees" }
    if ($targetAngle -eq 90 -and ([Math]::Abs($x - $sign * $halfCell) -gt 1 -or [Math]::Abs($y - $halfCell) -gt 1)) {
        throw "$side endpoint mismatch: ($x, $y). Recalculate entry/exit when changing speed or timing."
    }
    if ($minWheel -lt $minimum -or 1e6 * $pulseDistance / $minWheel -gt 65535) {
        throw "$side wheel speed outside supported timer range"
    }
    Write-Output ('{0}: angle={1:F3} deg, endpoint=({2:F3},{3:F3}) mm, wheels={4:F1}..{5:F1} mm/s' -f $side,$degrees,$x,$y,$minWheel,$maxWheel)
}
}
Write-Output 'PASS: nominal no-slip geometry and forward wheel/timer limits. Physical clearance and slip require robot tests.'
