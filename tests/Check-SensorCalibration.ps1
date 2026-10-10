$ErrorActionPreference = 'Stop'
# Numerical/table validation, not execution of the compiled STM32 C function.
$source = Get-Content -Raw (Join-Path $PSScriptRoot '../Core/Src/sensor.c')
$expected = @{
    L = @(2599,1880,1286,946,740,606,516,456,417)
    FL = @(771,547,510,497,464,412,336,330,298)
    FR = @(2647,2528,2516,1715,1311,1047,856,704,581)
    R = @(950,798,670,555,453,347,280,230,194)
}
function Convert-Reference($points, [uint32]$raw) {
    if ($raw -gt $points[0].raw) { return @(10.0, 'high') }
    if ($raw -lt $points[8].raw) { return @(90.0, 'low') }
    for ($i = 0; $i -lt 8; ++$i) {
        if ($raw -ge $points[$i+1].raw) {
            return @(($points[$i].mm + ($points[$i].raw - [double]$raw) *
                ($points[$i+1].mm - $points[$i].mm) /
                ($points[$i].raw - $points[$i+1].raw)), 'ok')
        }
    }
    throw 'No interval'
}
foreach ($name in @('L','FL','FR','R')) {
    $match = [regex]::Match($source, "(?s)\[SENSOR_$name\]\s*=\s*\{((?:\s*\{\d+,\d+\}\s*,?)+)\}")
    $pairs = [regex]::Matches($match.Groups[1].Value, '\{\s*(\d+)\s*,\s*(\d+)\s*\}')
    if ($pairs.Count -ne 9) { throw "${name}: expected 9 calibration points" }
    $points = @($pairs | ForEach-Object { @{raw=[int]$_.Groups[1].Value; mm=[int]$_.Groups[2].Value} })
    for ($i=0; $i -lt 9; ++$i) {
        if ($points[$i].raw -ne $expected[$name][$i] -or $points[$i].mm -ne 10*($i+1)) {
            throw "${name}: calibration mismatch at $i"
        }
        $result = Convert-Reference $points $points[$i].raw
        if ([Math]::Abs($result[0] - 10*($i+1)) -gt 0.0001 -or $result[1] -ne 'ok') { throw 'Knot mismatch' }
        if ($i -lt 8) {
            $gap = $points[$i].raw - $points[$i+1].raw
            if ($gap -le 0) { throw 'Non-monotonic calibration' }
            $mid = [uint32][Math]::Floor(($points[$i].raw + $points[$i+1].raw)/2.0)
            $result = Convert-Reference $points $mid
            if ($result[0] -le $points[$i].mm -or $result[0] -ge $points[$i+1].mm) { throw 'Interior mismatch' }
        }
    }
    $previous = 90.0
    for ($raw=0; $raw -le 4095; ++$raw) {
        $result = Convert-Reference $points $raw
        if ($result[0] -lt 10 -or $result[0] -gt 90 -or $result[0] -gt $previous) { throw 'Bound/monotonicity mismatch' }
        $previous = $result[0]
    }
    foreach ($raw in @(0, ($points[8].raw-1))) {
        $result = Convert-Reference $points $raw
        if ($result[0] -ne 90 -or $result[1] -ne 'low') { throw 'Low clamp mismatch' }
    }
    foreach ($raw in @(($points[0].raw+1), 4095, [uint32]::MaxValue)) {
        $result = Convert-Reference $points $raw
        if ($result[0] -ne 10 -or $result[1] -ne 'high') { throw 'High clamp mismatch' }
    }
    Write-Output "${name}: PASS table, knots, interiors, all 12-bit inputs and clamps (reference calculation)"
}
