#Requires -Version 5.1

param(
    [string]$ProbeSerial = '4041840e',
    [string]$ExpectedFirmwarePrefix = 'M6',
    [string]$ExpectedFirmwareId = 'M6-runtime-20260731-r45',
    [ValidateRange(1, 65535)]
    [int]$ExpectedSchema = 1,
    [string]$Port,
    [switch]$PreflightOnly
)

$ErrorActionPreference = 'Stop'

function Assert-KeilProjectConfiguration {
    param(
        [Parameter(Mandatory = $true)]
        [string]$ProjectPath
    )

    if (-not (Test-Path -LiteralPath $ProjectPath -PathType Leaf)) {
        throw "Production Keil project not found: $ProjectPath"
    }
    [xml]$project = Get-Content -LiteralPath $ProjectPath -Raw
    $target = $project.Project.Targets.Target
    if ($null -eq $target) { throw 'Keil project has no target.' }
    if ($target.TargetName -ne 'empty_LP_MSPM0G3507_nortos_keil') {
        throw "Unexpected Keil target: $($target.TargetName)"
    }
    $common = $target.TargetOption.TargetCommonOption
    if ($common.Device -ne 'MSPM0G3507') {
        throw "Unexpected target device: $($common.Device)"
    }
    if ([string]$target.pCCUsed -notmatch '6220000::V6\.22::ARMCLANG') {
        throw "The production target is not configured for ARMCLANG 6.22: $($target.pCCUsed)"
    }
    if ([string]$common.PackID -notmatch '^TexasInstruments\.MSPM0G1X0X_G3X0X_DFP\.') {
        throw "Unexpected TI device pack: $($common.PackID)"
    }
    $utilities = $target.TargetOption.Utilities
    if ([string]$utilities.Flash2 -notmatch 'UL2CM3\.DLL') {
        throw "CMSIS-AGDI flash driver is not configured: $($utilities.Flash2)"
    }
    if ([string]$utilities.Flash1.UseTargetDll -ne '1') {
        throw 'The production target is not configured to use its target flash driver.'
    }
    if ([string]$utilities.Flash1.UpdateFlashBeforeDebugging -ne '1') {
        throw 'The production target is not configured to update flash before debugging.'
    }
    return $target
}

function Assert-KeilDebugConfiguration {
    param(
        [Parameter(Mandatory = $true)]
        [string]$ProjectPath,
        [Parameter(Mandatory = $true)]
        [string]$TargetName,
        [Parameter(Mandatory = $true)]
        [string]$SerialNumber
    )

    $optionsPath = [System.IO.Path]::ChangeExtension($ProjectPath, '.uvoptx')
    if (-not (Test-Path -LiteralPath $optionsPath -PathType Leaf)) {
        throw ("Keil debug options are missing: $optionsPath. Open this exact " +
               'project and select Use Target / CMSIS-DAP before downloading.')
    }
    [xml]$options = Get-Content -LiteralPath $optionsPath -Raw
    $debugTarget = @($options.ProjectOpt.Target | Where-Object {
        [string]$_.TargetName -eq $TargetName
    }) | Select-Object -First 1
    if ($null -eq $debugTarget) {
        throw "Keil debug options do not contain target ${TargetName}: $optionsPath"
    }
    $targetOptions = $debugTarget.TargetOption
    if (([string]$targetOptions.OPTTT.RunTarget -ne '1') -or
        ([string]$targetOptions.OPTFL.IsCurrentTarget -ne '1')) {
        throw 'Keil options do not select this target as the current hardware target.'
    }
    if (([string]$targetOptions.DebugOpt.uTrg -ne '1') -or
        ([string]$targetOptions.DebugOpt.uSim -ne '0')) {
        throw 'Keil Debug is not set to Use Target.'
    }
    if ([string]$targetOptions.DebugOpt.pMon -notmatch '(?i)CMSIS_AGDI\.dll$') {
        throw 'Keil Debug is not configured for CMSIS-AGDI/CMSIS-DAP.'
    }
    if (([string]$targetOptions.DebugDescription.Enable -ne '1') -or
        ([string]$targetOptions.DebugDescription.EnableFlashSeq -ne '1')) {
        throw 'Keil target debugging or its flash sequence is disabled.'
    }
    if (([string]$targetOptions.DebugDescription.Protocol -ne '2') -or
        ([string]$targetOptions.DebugDescription.DbgClock -ne '1000000')) {
        throw 'Keil Debug is not configured for the verified 1 MHz SWD link.'
    }

    $serialPattern = [regex]::Escape($SerialNumber)
    $matchingRegistry = @(
        $targetOptions.TargetDriverDllRegistry.SetRegEntry | Where-Object {
            ([string]$_.Key -eq 'CMSIS_AGDI') -and
            ([string]$_.Name -match '(?i)Horco CMSIS-DAP') -and
            ([string]$_.Name -match "(?i)-U$serialPattern(?:\s|$)") -and
            ([string]$_.Name -match
             '(?i)MSPM0G1X0X_G3X0X_MAIN_128KB\.FLM')
        }
    )
    if ($matchingRegistry.Count -eq 0) {
        throw ("Keil Debug does not select Horco CMSIS-DAP serial " +
               "$SerialNumber in $optionsPath")
    }
    return $debugTarget
}

function Assert-HorcoCmsisDap {
    param(
        [Parameter(Mandatory = $true)]
        [string]$SerialNumber,
        [scriptblock]$DeviceProvider
    )

    if ($null -eq $DeviceProvider) {
        $DeviceProvider = {
            Get-PnpDevice -PresentOnly -ErrorAction Stop
        }
    }
    $devices = @(& $DeviceProvider)
    $serialPattern = [regex]::Escape($SerialNumber)
    $matchingSerial = @($devices | Where-Object {
        ([string]$_.InstanceId -match "(?i)$serialPattern") -or
        ([string]$_.SerialNumber -eq $SerialNumber)
    })
    if ($matchingSerial.Count -eq 0) {
        throw "Horco CMSIS-DAP serial $SerialNumber is not present; no download was attempted."
    }
    $serialInstanceIds = @($matchingSerial | ForEach-Object {
        [string]$_.InstanceId
    })
    $dap = @($devices | Where-Object {
        $isDap =
        ([string]$_.FriendlyName -match '(?i)(horco|cmsis.?dap)') -or
        ([string]$_.Name -match '(?i)(horco|cmsis.?dap)')
        if (-not $isDap) { return $false }

        $directSerialMatch =
            ([string]$_.InstanceId -match "(?i)$serialPattern") -or
            ([string]$_.SerialNumber -eq $SerialNumber)
        if ($directSerialMatch) { return $true }

        $parentInstanceId = ''
        if ($_.PSObject.Properties.Name -contains 'ParentInstanceId') {
            $parentInstanceId = [string]$_.ParentInstanceId
        } elseif ($_.PSObject.Properties.Name -contains 'Parent') {
            $parentInstanceId = [string]$_.Parent
        } elseif ([string]$_.InstanceId -ne '') {
            try {
                $parentInstanceId = [string](Get-PnpDeviceProperty `
                    -InstanceId ([string]$_.InstanceId) `
                    -KeyName 'DEVPKEY_Device_Parent' `
                    -ErrorAction Stop).Data
            } catch {
                $parentInstanceId = ''
            }
        }
        return ($parentInstanceId -ne '') -and
            ($serialInstanceIds -icontains $parentInstanceId)
    })
    if ($dap.Count -eq 0) {
        throw "USB device serial $SerialNumber is present but is not identified as Horco CMSIS-DAP."
    }
    $notOk = @($dap | Where-Object {
        $_.PSObject.Properties.Name -contains 'Status' -and
        [string]$_.Status -notin @('OK', 'Unknown', '')
    })
    if ($notOk.Count -ne 0) {
        throw "Horco CMSIS-DAP serial $SerialNumber is present but not healthy."
    }
    return $dap[0]
}

function Assert-ProductionBuildEvidence {
    param(
        [Parameter(Mandatory = $true)]
        [string]$RepositoryRoot
    )

    $buildLog = Join-Path $RepositoryRoot 'keil\build.log'
    $objects = Join-Path $RepositoryRoot 'keil\Objects'
    $axf = Join-Path $objects 'empty_LP_MSPM0G3507_nortos_keil.axf'
    $hex = Join-Path $objects 'empty_LP_MSPM0G3507_nortos_keil.hex'
    foreach ($path in @($buildLog, $axf, $hex)) {
        if (-not (Test-Path -LiteralPath $path -PathType Leaf)) {
            throw "Missing successful production rebuild evidence: $path"
        }
    }
    $buildText = Get-Content -LiteralPath $buildLog -Raw
    if ($buildText -notmatch "Rebuild target 'empty_LP_MSPM0G3507_nortos_keil'" -or
        $buildText -notmatch 'Using Compiler.*V6\.22' -or
        $buildText -notmatch '0 Error\(s\), 0 Warning\(s\)') {
        throw 'build.log does not prove a clean Keil 6.22 production Rebuild.'
    }

    $inputRoots = @('Source', 'System', 'Control', 'App', 'Hardware', 'Config', 'include', 'keil')
    $inputs = foreach ($relativeRoot in $inputRoots) {
        $inputRoot = Join-Path $RepositoryRoot $relativeRoot
        if (Test-Path -LiteralPath $inputRoot) {
            Get-ChildItem -LiteralPath $inputRoot -Recurse -File | Where-Object {
                $_.Extension -in @('.c', '.h', '.s', '.sct', '.uvprojx')
            }
        }
    }
    $newestInput = $inputs | Sort-Object LastWriteTimeUtc -Descending | Select-Object -First 1
    $axfItem = Get-Item -LiteralPath $axf
    $hexItem = Get-Item -LiteralPath $hex
    if ($null -ne $newestInput -and
        ($axfItem.LastWriteTimeUtc -lt $newestInput.LastWriteTimeUtc -or
         $hexItem.LastWriteTimeUtc -lt $newestInput.LastWriteTimeUtc)) {
        throw "Production rebuild is stale relative to $($newestInput.FullName)."
    }
    return [pscustomobject]@{
        BuildLog = $buildLog
        AxF = $axf
        Hex = $hex
    }
}

function Test-KeilDownloadResult {
    param(
        [Parameter(Mandatory = $true)]
        [int]$ExitCode,
        [Parameter(Mandatory = $true)]
        [string]$LogPath
    )

    if ($ExitCode -ne 0 -or -not (Test-Path -LiteralPath $LogPath -PathType Leaf)) {
        return $false
    }
    return (Get-Content -LiteralPath $LogPath -Raw) -match 'Verify OK'
}

function Invoke-KeilDownloadWithRetry {
    param(
        [Parameter(Mandatory = $true)]
        [string]$UV4Path,
        [Parameter(Mandatory = $true)]
        [string[]]$ArgumentList,
        [Parameter(Mandatory = $true)]
        [string]$LogPath,
        [scriptblock]$ProcessRunner
    )

    if ($null -eq $ProcessRunner) {
        $ProcessRunner = {
            param($Executable, $Arguments)
            Start-Process -FilePath $Executable -ArgumentList $Arguments `
                -WindowStyle Hidden -Wait -PassThru
        }
    }
    if (Test-Path -LiteralPath $LogPath -PathType Leaf) {
        Remove-Item -LiteralPath $LogPath -Force
    }

    # The same executable and immutable argument array are used on both calls.
    # Do not add a third attempt or mutate probe/target/transport configuration.
    $commandArguments = @($ArgumentList)
    $lastFailure = ''
    for ($attempt = 1; $attempt -le 2; $attempt++) {
        if (Test-Path -LiteralPath $LogPath -PathType Leaf) {
            Remove-Item -LiteralPath $LogPath -Force
        }
        try {
            $process = & $ProcessRunner $UV4Path $commandArguments
            $exitCode = [int]$process.ExitCode
            $lastFailure = "exit=$exitCode or Verify OK missing"
        } catch {
            $exitCode = -1
            $lastFailure = $_.Exception.Message
        }
        if (Test-KeilDownloadResult -ExitCode $exitCode -LogPath $LogPath) {
            return [pscustomobject]@{
                Attempts = $attempt
                ExitCode = $exitCode
                LogPath = $LogPath
            }
        }
        if ($attempt -eq 1) {
            Write-Warning 'First command-line download failed; retrying the identical command once.'
        }
    }
    throw ("Command-line download failed twice ($lastFailure). Stop here and " +
           'download manually; do not switch to GUI automation, BSL, or another command variant.')
}

function Invoke-M6ApplicationProbe {
    param(
        [Parameter(Mandatory = $true)]
        [string]$RepositoryRoot,
        [Parameter(Mandatory = $true)]
        [string]$FirmwarePrefix,
        [string]$FirmwareId,
        [Parameter(Mandatory = $true)]
        [int]$Schema,
        [string]$SerialPort,
        [scriptblock]$PythonRunner
    )

    $tool = Join-Path $RepositoryRoot 'scripts\m6_tune.py'
    if (-not (Test-Path -LiteralPath $tool -PathType Leaf)) {
        throw "M6 probe tool is missing: $tool"
    }
    $probeArguments = @(
        $tool,
        '--firmware-prefix', $FirmwarePrefix,
        '--schema', [string]$Schema
    )
    if ($SerialPort) { $probeArguments += @('--port', $SerialPort) }
    if ($FirmwareId) { $probeArguments += @('--firmware-id', $FirmwareId) }
    $probeArguments += @('probe', '--json')
    if ($null -eq $PythonRunner) {
        $PythonRunner = {
            param($Arguments)
            $lines = & python @Arguments 2>&1
            return [pscustomobject]@{
                ExitCode = $LASTEXITCODE
                Output = @($lines)
            }
        }
    }
    $run = & $PythonRunner $probeArguments
    if ([int]$run.ExitCode -ne 0) {
        throw "Flash Verify OK, but application startup probe failed: $($run.Output -join ' ')"
    }
    $jsonLine = @($run.Output | Where-Object {
        ([string]$_).TrimStart().StartsWith('{')
    }) | Select-Object -Last 1
    if ($null -eq $jsonLine) {
        throw 'Flash Verify OK, but the application startup probe returned no JSON evidence.'
    }
    try {
        $identity = ([string]$jsonLine | ConvertFrom-Json)
    } catch {
        throw "Flash Verify OK, but startup identity JSON is invalid: $jsonLine"
    }
    if ([string]$identity.firmware -notlike "$FirmwarePrefix*") {
        throw "Flash Verify OK, but firmware identity is $($identity.firmware)."
    }
    if ($FirmwareId -and [string]$identity.firmware -cne $FirmwareId) {
        throw "Flash Verify OK, but firmware identity is $($identity.firmware), expected $FirmwareId."
    }
    if ([int]$identity.schema -ne $Schema) {
        throw "Flash Verify OK, but parameter schema is $($identity.schema)."
    }
    if ($null -eq $identity.revision -or [int]$identity.revision -lt 0) {
        throw 'Flash Verify OK, but the parameter revision is missing.'
    }
    if ([string]$identity.source -notmatch '^(defaults|persisted|flash[-_]a|flash[-_]b)$') {
        throw "Flash Verify OK, but parameter load source is invalid: $($identity.source)"
    }
    return $identity
}

function Invoke-M6Flash {
    param(
        [string]$SerialNumber = '4041840e',
        [string]$FirmwarePrefix = 'M6',
        [string]$FirmwareId = 'M6-runtime-20260731-r45',
        [int]$Schema = 1,
        [string]$SerialPort,
        [switch]$OnlyPreflight
    )

    $repositoryRoot = (Resolve-Path -LiteralPath (Join-Path $PSScriptRoot '..')).Path
    . (Join-Path $PSScriptRoot 'resolve_keil.ps1')
    $uv4 = Resolve-KeilUV4
    $project = Join-Path $repositoryRoot 'keil\empty_LP_MSPM0G3507_nortos_keil.uvprojx'

    $target = Assert-KeilProjectConfiguration -ProjectPath $project
    $null = Assert-KeilDebugConfiguration -ProjectPath $project `
        -TargetName $target.TargetName -SerialNumber $SerialNumber
    $probe = Assert-HorcoCmsisDap -SerialNumber $SerialNumber
    $build = Assert-ProductionBuildEvidence -RepositoryRoot $repositoryRoot
    Write-Output "Keil: $uv4"
    Write-Output "Target: $($target.TargetName), ARMCLANG 6.22, MSPM0G3507"
    Write-Output "Debug: Use Target, CMSIS-AGDI, Horco serial=$SerialNumber"
    Write-Output "Probe: $($probe.FriendlyName), serial=$SerialNumber"
    Write-Output "Build evidence: $($build.AxF)"
    if ($OnlyPreflight) {
        Write-Output 'Preflight passed; no erase or programming command was started.'
        return
    }

    $timestamp = Get-Date -Format 'yyyyMMdd-HHmmss'
    $flashLog = Join-Path $repositoryRoot "keil\flash-$timestamp.log"
    $downloadArguments = @(
        '-f', ('"' + $project + '"'),
        '-o', ('"' + $flashLog + '"')
    )
    $download = Invoke-KeilDownloadWithRetry -UV4Path $uv4 `
        -ArgumentList $downloadArguments -LogPath $flashLog
    Get-Content -LiteralPath $flashLog | Select-Object -Last 30
    Write-Output "SWD programming verified after $($download.Attempts) attempt(s)."

    $identity = Invoke-M6ApplicationProbe -RepositoryRoot $repositoryRoot `
        -FirmwarePrefix $FirmwarePrefix -FirmwareId $FirmwareId `
        -Schema $Schema -SerialPort $SerialPort
    Write-Output ("Confirmed running firmware={0}, schema={1}, revision={2}, source={3}" -f `
        $identity.firmware, $identity.schema, $identity.revision, $identity.source)
    Write-Output "Download evidence: $flashLog"
}

if ($env:M6_FLASH_LIBRARY_ONLY -ne '1') {
    Invoke-M6Flash -SerialNumber $ProbeSerial `
        -FirmwarePrefix $ExpectedFirmwarePrefix -FirmwareId $ExpectedFirmwareId `
        -Schema $ExpectedSchema `
        -SerialPort $Port -OnlyPreflight:$PreflightOnly
}
