param(
    [ValidateSet('Build', 'Rebuild')]
    [string]$Mode = 'Rebuild'
)

$ErrorActionPreference = 'Stop'
$scriptRoot = Split-Path -Parent $MyInvocation.MyCommand.Path
$repositoryRoot = (Resolve-Path -LiteralPath (Join-Path $scriptRoot '..')).Path
. (Join-Path $scriptRoot 'resolve_keil.ps1')

$uv4 = Resolve-KeilUV4
$project = Join-Path $repositoryRoot 'keil\empty_LP_MSPM0G3507_nortos_keil.uvprojx'
$buildLog = Join-Path $repositoryRoot 'keil\build.log'
$objects = Join-Path $repositoryRoot 'keil\Objects'
$axf = Join-Path $objects 'empty_LP_MSPM0G3507_nortos_keil.axf'
$hex = Join-Path $objects 'empty_LP_MSPM0G3507_nortos_keil.hex'
$operation = if ($Mode -eq 'Rebuild') { '-r' } else { '-b' }

Write-Output "Keil: $uv4"
Write-Output "Project: $project"
Write-Output "Mode: $Mode"

if (Test-Path -LiteralPath $buildLog -PathType Leaf) {
    Remove-Item -LiteralPath $buildLog -Force
}
$keilProcess = Start-Process -FilePath $uv4 -ArgumentList @(
    $operation, ('"' + $project + '"'), '-j0', '-o', ('"' + $buildLog + '"')
) -WindowStyle Hidden -Wait -PassThru
$uvExitCode = $keilProcess.ExitCode

if (-not (Test-Path -LiteralPath $buildLog -PathType Leaf)) {
    throw "Keil did not create build log (exit $uvExitCode): $buildLog"
}
$logText = Get-Content -LiteralPath $buildLog -Raw
if (($null -ne $uvExitCode -and $uvExitCode -ne 0) -or
    $logText -notmatch '0 Error\(s\), 0 Warning\(s\)') {
    Get-Content -LiteralPath $buildLog | Write-Output
    throw "Keil build failed (exit $uvExitCode)."
}
if (-not (Test-Path -LiteralPath $axf -PathType Leaf)) { throw "Missing AXF: $axf" }
if (-not (Test-Path -LiteralPath $hex -PathType Leaf)) { throw "Missing HEX: $hex" }

Get-Content -LiteralPath $buildLog | Select-Object -Last 12
Write-Output "AXF bytes: $((Get-Item -LiteralPath $axf).Length)"
Write-Output "HEX bytes: $((Get-Item -LiteralPath $hex).Length)"
