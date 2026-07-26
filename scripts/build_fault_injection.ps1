param(
    [string]$ProbeSerial = '4041840e'
)

$ErrorActionPreference = 'Stop'
$scriptRoot = Split-Path -Parent $MyInvocation.MyCommand.Path
$repositoryRoot = (Resolve-Path -LiteralPath (Join-Path $scriptRoot '..')).Path
$m6FaultInjectionProbeSerial = $ProbeSerial
. (Join-Path $scriptRoot 'resolve_keil.ps1')
$m6PreviousFlashLibraryOnly = $env:M6_FLASH_LIBRARY_ONLY
try {
    $env:M6_FLASH_LIBRARY_ONLY = '1'
    . (Join-Path $scriptRoot 'flash.ps1')
}
finally {
    if ($null -eq $m6PreviousFlashLibraryOnly) {
        Remove-Item Env:M6_FLASH_LIBRARY_ONLY -ErrorAction SilentlyContinue
    } else {
        $env:M6_FLASH_LIBRARY_ONLY = $m6PreviousFlashLibraryOnly
    }
}

$uv4 = Resolve-KeilUV4
$projectPath = Join-Path $repositoryRoot 'keil\empty_LP_MSPM0G3507_nortos_keil.uvprojx'
$optionsPath = [System.IO.Path]::ChangeExtension($projectPath, '.uvoptx')
$generatedProject = Join-Path $repositoryRoot 'keil\fault_injection.generated.uvprojx'
$generatedOptions = Join-Path $repositoryRoot 'keil\fault_injection.generated.uvoptx'
$buildLog = Join-Path $repositoryRoot 'keil\fault_injection_build.log'
$objects = Join-Path $repositoryRoot 'keil\ObjectsFaultInjection'
$axf = Join-Path $objects 'm2_fault_injection.axf'
$hex = Join-Path $objects 'm2_fault_injection.hex'

$projectText = Get-Content -LiteralPath $projectPath -Raw
$projectText = $projectText.Replace(
    '<TargetName>empty_LP_MSPM0G3507_nortos_keil</TargetName>',
    '<TargetName>m2_fault_injection</TargetName>')
$projectText = $projectText.Replace(
    '<OutputDirectory>.\Objects\</OutputDirectory>',
    '<OutputDirectory>.\ObjectsFaultInjection\</OutputDirectory>')
$projectText = $projectText.Replace(
    '<OutputName>empty_LP_MSPM0G3507_nortos_keil</OutputName>',
    '<OutputName>m2_fault_injection</OutputName>')
$projectText = $projectText.Replace(
    '<Define>__MSPM0G3507__</Define>',
    '<Define>__MSPM0G3507__,M2_FAULT_INJECTION_TEST</Define>')
if (($projectText -notmatch 'M2_FAULT_INJECTION_TEST') -or
    ($projectText -notmatch 'ObjectsFaultInjection') -or
    ($projectText -notmatch '<OutputName>m2_fault_injection</OutputName>')) {
    throw 'Failed to isolate the generated fault-injection project.'
}
[System.IO.File]::WriteAllText($generatedProject, $projectText,
    [System.Text.UTF8Encoding]::new($false))

if (Test-Path -LiteralPath $generatedOptions -PathType Leaf) {
    Remove-Item -LiteralPath $generatedOptions -Force
}
if (Test-Path -LiteralPath $optionsPath -PathType Leaf) {
    $optionsText = Get-Content -LiteralPath $optionsPath -Raw
    $optionsText = $optionsText.Replace(
        '<TargetName>empty_LP_MSPM0G3507_nortos_keil</TargetName>',
        '<TargetName>m2_fault_injection</TargetName>')
    if ($optionsText -notmatch
            '<TargetName>m2_fault_injection</TargetName>') {
        throw 'Failed to retarget the fault-injection debug options.'
    }
    [System.IO.File]::WriteAllText($generatedOptions, $optionsText,
        [System.Text.UTF8Encoding]::new($false))
    $null = Assert-KeilDebugConfiguration -ProjectPath $generatedProject `
        -TargetName 'm2_fault_injection' `
        -SerialNumber $m6FaultInjectionProbeSerial
}

if (Test-Path -LiteralPath $buildLog -PathType Leaf) {
    Remove-Item -LiteralPath $buildLog -Force
}
$buildProcess = Start-Process -FilePath $uv4 -ArgumentList @(
    '-r', ('"' + $generatedProject + '"'), '-j0', '-o', ('"' + $buildLog + '"')
) -WindowStyle Hidden -Wait -PassThru
if (-not (Test-Path -LiteralPath $buildLog -PathType Leaf)) {
    throw "Keil did not create fault-injection build log (exit $($buildProcess.ExitCode))."
}
$buildText = Get-Content -LiteralPath $buildLog -Raw
if (($buildProcess.ExitCode -ne 0) -or
    $buildText -notmatch '0 Error\(s\), 0 Warning\(s\)') {
    Get-Content -LiteralPath $buildLog | Write-Output
    throw "Fault-injection build failed (exit $($buildProcess.ExitCode))."
}
if (-not (Test-Path -LiteralPath $axf -PathType Leaf)) { throw "Missing AXF: $axf" }
if (-not (Test-Path -LiteralPath $hex -PathType Leaf)) { throw "Missing HEX: $hex" }

Get-Content -LiteralPath $buildLog | Select-Object -Last 12
Write-Output "Fault-injection AXF bytes: $((Get-Item -LiteralPath $axf).Length)"
Write-Output "Fault-injection HEX bytes: $((Get-Item -LiteralPath $hex).Length)"
Write-Output "Fault-injection project: $generatedProject"
if (Test-Path -LiteralPath $generatedOptions -PathType Leaf) {
    Write-Output "Fault-injection debug options: $generatedOptions"
}
