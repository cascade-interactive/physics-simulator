[CmdletBinding()]
param(
    [ValidateSet("Debug", "Release")]
    [string]$Configuration = "Debug",
    [switch]$Clean
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

& (Join-Path $PSScriptRoot "bootstrap.ps1")

$cleanPath = [System.Environment]::GetEnvironmentVariable("PATH")
Remove-Item Env:PATH -ErrorAction SilentlyContinue
$env:Path = "$(Join-Path $sdk "Bin");$cleanPath"
$env:VULKAN_SDK = $sdk
$env:VK_SDK_PATH = $sdk

if ($Clean -and (Test-Path -LiteralPath $build)) {
    Remove-Item -Recurse -Force -LiteralPath $build
}

cmake `
    -S $root `
    -B $build `
    -G "Visual Studio 17 2022" `
    -A x64 `
    "-DUAVIEW_VULKAN_SDK=$sdk" `
    -DUAVIEW_ENABLE_VALIDATION=ON `
    -DUAVIEW_BUILD_TESTS=ON
if ($LASTEXITCODE -ne 0) {
    throw "CMake configure failed."
}

cmake --build $build --config $Configuration --parallel
if ($LASTEXITCODE -ne 0) {
    throw "Build failed."
}

Write-Host "UAView Studio $Configuration build completed."
