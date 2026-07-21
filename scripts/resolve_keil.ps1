function Resolve-KeilUV4 {
    $candidates = [System.Collections.Generic.List[string]]::new()
    if ($env:KEIL_UV4) { $candidates.Add($env:KEIL_UV4) }
    if ($env:LOCALAPPDATA) {
        $candidates.Add((Join-Path $env:LOCALAPPDATA 'Keil_v5\UV4\UV4.exe'))
    }
    if ($env:ProgramFiles) {
        $candidates.Add((Join-Path $env:ProgramFiles 'Keil_v5\UV4\UV4.exe'))
    }
    $command = Get-Command UV4.exe -ErrorAction SilentlyContinue
    if ($command) { $candidates.Add($command.Source) }

    foreach ($candidate in $candidates) {
        if ($candidate -and (Test-Path -LiteralPath $candidate -PathType Leaf)) {
            return (Resolve-Path -LiteralPath $candidate).Path
        }
    }
    throw 'Keil UV4.exe not found. Set KEIL_UV4 to its absolute path.'
}
