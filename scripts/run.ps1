[CmdletBinding()]
param(
    [ValidateSet("Debug", "Release")]
    [string]$Configuration = "Debug"
)

$ErrorActionPreference = "Stop"
$root = Split-Path -Parent $PSScriptRoot
$executable = Join-Path $root "build\bin\$Configuration\UAView Studio.exe"
$versionFile = Join-Path $root "config\vulkan-sdk-version.txt"

if (-not (Test-Path -LiteralPath $versionFile)) {
    throw "Missing Vulkan SDK version file: $versionFile"
}
$sdkVersion = (Get-Content -Raw -LiteralPath $versionFile).Trim()
$sdk = Join-Path $root ".tools\VulkanSDK\$sdkVersion"

& (Join-Path $PSScriptRoot "build.ps1") -Configuration $Configuration

$cleanPath = [System.Environment]::GetEnvironmentVariable("PATH")
Remove-Item Env:PATH -ErrorAction SilentlyContinue
$env:Path = "$(Join-Path $sdk "Bin");$cleanPath"
$env:VULKAN_SDK = $sdk
$env:VK_SDK_PATH = $sdk
$env:VK_LAYER_PATH = Join-Path $sdk "Bin"

$studioProcess = Start-Process `
    -FilePath $executable `
    -WorkingDirectory (Split-Path -Parent $executable) `
    -PassThru `
    -Wait
if ($studioProcess.ExitCode -ne 0) {
    throw "UAView Studio exited with code $($studioProcess.ExitCode)."
}
