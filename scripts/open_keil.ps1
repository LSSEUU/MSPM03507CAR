$ErrorActionPreference = 'Stop'
$scriptRoot = Split-Path -Parent $MyInvocation.MyCommand.Path
$repositoryRoot = (Resolve-Path -LiteralPath (Join-Path $scriptRoot '..')).Path
. (Join-Path $scriptRoot 'resolve_keil.ps1')
$uv4 = Resolve-KeilUV4
$project = Join-Path $repositoryRoot 'keil\empty_LP_MSPM0G3507_nortos_keil.uvprojx'
Start-Process -FilePath $uv4 -ArgumentList $project
Write-Output "Opened: $project"
