$ErrorActionPreference = 'Stop'
$repositoryRoot = (Resolve-Path -LiteralPath (Join-Path $PSScriptRoot '..')).Path
Push-Location $repositoryRoot
try {
    python -m unittest discover -s Tests -p 'test_*.py' -v
    if ($LASTEXITCODE -ne 0) { throw 'Host regression tests failed.' }
} finally {
    Pop-Location
}
