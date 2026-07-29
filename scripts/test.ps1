$ErrorActionPreference = 'Stop'
$repositoryRoot = (Resolve-Path -LiteralPath (Join-Path $PSScriptRoot '..')).Path
$pythonSource = Join-Path $repositoryRoot 'python\src'
$previousPythonPath = $env:PYTHONPATH
$env:PYTHONPATH = if ([string]::IsNullOrEmpty($previousPythonPath)) {
    $pythonSource
} else {
    $pythonSource + [IO.Path]::PathSeparator + $previousPythonPath
}
Push-Location $repositoryRoot
try {
    python -m unittest discover -s Tests -p 'test_*.py' -v
    if ($LASTEXITCODE -ne 0) { throw 'Host regression tests failed.' }
} finally {
    Pop-Location
    $env:PYTHONPATH = $previousPythonPath
}
