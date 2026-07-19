[CmdletBinding()]
param(
    [ValidateSet("Debug", "Release")]
    [string]$Configuration = "Debug"
)

$ErrorActionPreference = "Stop"
$root = Split-Path -Parent $PSScriptRoot
$build = Join-Path $root "build"
$versionFile = Join-Path $root "config\vulkan-sdk-version.txt"

if (-not (Test-Path -LiteralPath $versionFile)) {
    throw "Missing Vulkan SDK version file: $versionFile"
}
$sdkVersion = (Get-Content -Raw -LiteralPath $versionFile).Trim()
$sdk = Join-Path $root ".tools\VulkanSDK\$sdkVersion"
$executable = Join-Path $build "bin\$Configuration\UAView Studio.exe"

& (Join-Path $PSScriptRoot "build.ps1") -Configuration $Configuration

$cleanPath = [System.Environment]::GetEnvironmentVariable("PATH")
Remove-Item Env:PATH -ErrorAction SilentlyContinue
$env:Path = "$(Join-Path $sdk "Bin");$cleanPath"
$env:VULKAN_SDK = $sdk
$env:VK_SDK_PATH = $sdk
$env:VK_LAYER_PATH = Join-Path $sdk "Bin"

ctest `
    --test-dir $build `
    -C $Configuration `
    --output-on-failure `
    --no-tests=error
if ($LASTEXITCODE -ne 0) {
    throw "UAView Studio tests failed."
}

$smokeProcess = Start-Process `
    -FilePath $executable `
    -ArgumentList @("--smoke-test") `
    -WorkingDirectory (Split-Path -Parent $executable) `
    -PassThru `
    -Wait
if ($smokeProcess.ExitCode -ne 0) {
    throw "Vulkan smoke test failed with exit code $($smokeProcess.ExitCode)."
}

Write-Host "UAView Studio verification passed."
