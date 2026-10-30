$ErrorActionPreference = "Stop"

$env:PICO_SDK_PATH = "$env:USERPROFILE\.pico-sdk\sdk\2.3.1"
$env:PICO_TOOLCHAIN_PATH = "$env:USERPROFILE\.pico-sdk\toolchain\15_2_Rel1"
$env:Path = "$env:USERPROFILE\.pico-sdk\toolchain\15_2_Rel1\bin;$env:USERPROFILE\.pico-sdk\cmake\v4.3.4\bin;$env:USERPROFILE\.pico-sdk\ninja\v1.13.2;$env:Path"

Set-Location $PSScriptRoot

cmake -S . -B build -G Ninja
if ($LASTEXITCODE -ne 0) { throw "cmake configure failed" }

cmake --build build
if ($LASTEXITCODE -ne 0) { throw "cmake build failed" }

Write-Host "---- UF2 output ----"
Get-ChildItem build\*.uf2 | ForEach-Object { Write-Host ($_.FullName + "  (" + [math]::Round($_.Length/1KB) + " KB)") }
