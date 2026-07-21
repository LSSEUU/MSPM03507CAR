$ErrorActionPreference = 'Stop'

$repositoryRootText = (& git rev-parse --show-toplevel 2>$null).Trim()
if (-not $repositoryRootText) {
    throw 'Not inside a Git repository.'
}

$repositoryRoot = (Resolve-Path -LiteralPath $repositoryRootText).Path
$currentDirectory = (Resolve-Path -LiteralPath (Get-Location).Path).Path
if ($currentDirectory -ne $repositoryRoot) {
    throw "Run this script from the repository root: $repositoryRoot"
}

$hookPath = Join-Path $repositoryRoot '.githooks\pre-commit'
if (-not (Test-Path -LiteralPath $hookPath -PathType Leaf)) {
    throw "Missing hook: $hookPath"
}

& git config --local core.hooksPath .githooks
if ($LASTEXITCODE -ne 0) {
    throw 'Failed to configure core.hooksPath.'
}

$configuredPath = (& git config --local --get core.hooksPath).Trim()
if ($configuredPath -ne '.githooks') {
    throw "Unexpected core.hooksPath: $configuredPath"
}

Write-Output 'Git hooks enabled: core.hooksPath=.githooks'
