$ErrorActionPreference = "Stop"

$projectRoot = (Resolve-Path (Join-Path $PSScriptRoot "..\..")).Path
$dotnetRoot = Join-Path $projectRoot ".tools\dotnet"

if (-not (Test-Path (Join-Path $dotnetRoot "dotnet.exe"))) {
    throw "Missing local .NET 7 runtime: $dotnetRoot"
}

$env:DOTNET_ROOT = $dotnetRoot
$env:PATH = "$dotnetRoot;$env:PATH"

Push-Location $projectRoot
try {
    conda run -n k230 python vision\scripts\convert_k230_kmodel.py @args
    if ($LASTEXITCODE -ne 0) {
        throw "nncase conversion failed with exit code $LASTEXITCODE"
    }
}
finally {
    Pop-Location
}
