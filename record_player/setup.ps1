# Setup: powershell -ExecutionPolicy Bypass -File record_player/setup.ps1
# Setup and launch: powershell -ExecutionPolicy Bypass -File record_player/setup.ps1 -Start
# Explicit Python: ./record_player/setup.ps1 -Python C:/Python312/python.exe -Start
# Alternate port: ./record_player/setup.ps1 -Start -Port 4318
param([string]$Python = '', [switch]$Start, [int]$Port = 4317)
$ErrorActionPreference = 'Stop' # Abort setup on failed downloads or filesystem operations.
$projectDir = $PSScriptRoot # All environments/dependencies remain inside record_player.
$nodeVersion = (Get-Content "$projectDir/.node-version").Trim() # Pinned portable Node version.
$nodeDir = "$projectDir/.runtime/node-v$nodeVersion-win-x64" # Local Node installation.
$venvPython = "$projectDir/.venv/Scripts/python.exe" # Isolated Python for Parquet conversion.
if (-not (Test-Path "$nodeDir/node.exe")) {
    New-Item -ItemType Directory -Force "$projectDir/.runtime" | Out-Null
    $archiveName = "node-v$nodeVersion-win-x64.zip" # Official Windows x64 distribution.
    $archivePath = "$projectDir/.runtime/$archiveName" # Download destination.
    Invoke-WebRequest "https://nodejs.org/dist/v$nodeVersion/$archiveName" -OutFile $archivePath
    $checksums = (Invoke-WebRequest "https://nodejs.org/dist/v$nodeVersion/SHASUMS256.txt" -UseBasicParsing).Content # Official digest list.
    $expected = (($checksums -split "`n" | Where-Object { $_.Trim().EndsWith(" $archiveName") }) -split '\s+')[0] # Expected SHA256.
    if ((Get-FileHash $archivePath -Algorithm SHA256).Hash.ToLower() -ne $expected) { throw 'Node download checksum mismatch.' }
    Expand-Archive -LiteralPath $archivePath -DestinationPath "$projectDir/.runtime" -Force
}
if (-not (Test-Path $venvPython)) {
    if (-not $Python) {
        $candidates = @("$env:USERPROFILE/miniforge3/python.exe", "$env:USERPROFILE/miniconda3/python.exe", 'py', 'python', 'python3') # Existing interpreter candidates; override with -Python.
        foreach ($candidate in $candidates) {
            try {
                & $candidate -c 'import sys; assert (3,10) <= sys.version_info < (3,14)' 2>$null
                if ($LASTEXITCODE -eq 0) { $Python = $candidate; break }
            } catch { }
        }
    }
    if (-not $Python) { throw 'Python 3.10-3.13 is required. Install Python or supply -Python C:/path/to/python.exe, then rerun.' }
    & $Python -m venv "$projectDir/.venv"
    if ($LASTEXITCODE -ne 0) { throw 'Could not create Python venv.' }
}
$env:PATH = "$nodeDir;$env:PATH" # Applies only to this script process and its children.
Push-Location $projectDir
try {
    $dependencyHash = (Get-FileHash requirements.txt).Hash + (Get-FileHash package-lock.json).Hash # Reinstall only when pinned requirements change.
    $stamp = "$projectDir/.runtime/dependencies.sha256" # Successful-install marker for offline subsequent launches.
    $ready = (Test-Path $stamp) -and ((Get-Content $stamp -Raw).Trim() -eq $dependencyHash) -and (Test-Path node_modules/three/build/three.module.js) # Local dependency presence.
    & $venvPython -c 'import numpy, pyarrow' 2>$null
    if ($LASTEXITCODE -ne 0) { $ready = $false }
    if (-not $ready) {
        & $venvPython -m pip install -r requirements.txt
        if ($LASTEXITCODE -ne 0) { throw 'Python package installation failed.' }
        & "$nodeDir/node.exe" "$nodeDir/node_modules/npm/bin/npm-cli.js" ci
        if ($LASTEXITCODE -ne 0) { throw 'Node package installation failed.' }
        Set-Content -LiteralPath $stamp -Value $dependencyHash
    }
    if ($Start) { & "$nodeDir/node.exe" server.mjs --port $Port --open }
} finally { Pop-Location }
