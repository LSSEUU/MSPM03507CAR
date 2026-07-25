$ErrorActionPreference = 'Stop'
$scriptRoot = Split-Path -Parent $MyInvocation.MyCommand.Path
$repositoryRoot = (Resolve-Path -LiteralPath (Join-Path $scriptRoot '..')).Path
. (Join-Path $scriptRoot 'resolve_keil.ps1')

$uv4 = Resolve-KeilUV4
$project = Join-Path $repositoryRoot 'keil\empty_LP_MSPM0G3507_nortos_keil.uvprojx'
$flashLog = Join-Path $repositoryRoot 'keil\flash.log'

if (Test-Path -LiteralPath $flashLog -PathType Leaf) {
    Remove-Item -LiteralPath $flashLog -Force
}

Write-Output "Keil: $uv4"
Write-Output "Project: $project"
$flashProcess = Start-Process -FilePath $uv4 -ArgumentList @(
    '-f', ('"' + $project + '"'), '-o', ('"' + $flashLog + '"')
) -WindowStyle Hidden -Wait -PassThru

if (-not (Test-Path -LiteralPath $flashLog -PathType Leaf)) {
    throw "Keil did not create flash log (exit $($flashProcess.ExitCode))."
}
$flashText = Get-Content -LiteralPath $flashLog -Raw
Get-Content -LiteralPath $flashLog -Tail 30
if (($flashProcess.ExitCode -ne 0) -or ($flashText -notmatch 'Verify OK')) {
    throw "Keil flash failed (exit $($flashProcess.ExitCode))."
}
