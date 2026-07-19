[CmdletBinding()]
param()

$ErrorActionPreference = "Stop"
$root = Split-Path -Parent $PSScriptRoot
$versionFile = Join-Path $root "config\vulkan-sdk-version.txt"
$hashFile = Join-Path $root "config\vulkan-sdk-sha256.txt"

if (-not (Test-Path -LiteralPath $versionFile)) {
    throw "Missing Vulkan SDK version file: $versionFile"
}
if (-not (Test-Path -LiteralPath $hashFile)) {
    throw "Missing Vulkan SDK checksum file: $hashFile"
}

$sdkVersion = (Get-Content -Raw -LiteralPath $versionFile).Trim()
$sdkSha256 = (Get-Content -Raw -LiteralPath $hashFile).Trim().ToUpperInvariant()

if ($sdkVersion -notmatch '^\d+\.\d+\.\d+\.\d+$') {
    throw "Invalid Vulkan SDK version in ${versionFile}: '$sdkVersion'"
}
if ($sdkSha256 -notmatch '^[0-9A-F]{64}$') {
    throw "Invalid SHA-256 checksum in ${hashFile}: '$sdkSha256'"
}

$cache = Join-Path $root ".cache"
$sdkRoot = Join-Path $root ".tools\VulkanSDK\$sdkVersion"
$installer = Join-Path $cache "vulkansdk-windows-X64-$sdkVersion.exe"
$partialInstaller = "$installer.part"
$requiredSdkFiles = @(
    "Include\vulkan\vulkan.h",
    "Bin\glslc.exe",
    "Lib\vulkan-1.lib",
    "Bin\VkLayer_khronos_validation.json",
    "Bin\VkLayer_khronos_validation.dll"
)

function Test-VulkanSdkReady {
    foreach ($relativePath in $requiredSdkFiles) {
        if (-not (Test-Path -LiteralPath (Join-Path $sdkRoot $relativePath) -PathType Leaf)) {
            return $false
        }
    }
    return $true
}

if (Test-VulkanSdkReady) {
    Write-Host "Vulkan SDK $sdkVersion is already ready at $sdkRoot"
    return
}

New-Item -ItemType Directory -Force -Path $cache | Out-Null
$downloadUrl = "https://sdk.lunarg.com/sdk/download/$sdkVersion/windows/vulkansdk-windows-X64-$sdkVersion.exe"

if (Test-Path -LiteralPath $installer -PathType Leaf) {
    $cachedHash = (Get-FileHash -Algorithm SHA256 -LiteralPath $installer).Hash
    if ($cachedHash -ne $sdkSha256) {
        Write-Warning "Discarding a cached Vulkan SDK installer with an invalid checksum."
        Remove-Item -Force -LiteralPath $installer
    }
}

if (-not (Test-Path -LiteralPath $installer -PathType Leaf)) {
    if (Test-Path -LiteralPath $partialInstaller) {
        Remove-Item -Force -LiteralPath $partialInstaller
    }

    Write-Host "Downloading Vulkan SDK $sdkVersion..."
    try {
        Invoke-WebRequest -UseBasicParsing -Uri $downloadUrl -OutFile $partialInstaller
        $downloadedHash = (Get-FileHash -Algorithm SHA256 -LiteralPath $partialInstaller).Hash

        if ($downloadedHash -ne $sdkSha256) {
            throw "Vulkan SDK checksum mismatch. Expected $sdkSha256, received $downloadedHash."
        }

        Move-Item -LiteralPath $partialInstaller -Destination $installer
    }
    catch {
        if (Test-Path -LiteralPath $partialInstaller) {
            Remove-Item -Force -LiteralPath $partialInstaller
        }
        throw
    }
}

Write-Host "Installing a project-local Vulkan SDK..."
& $installer `
    --root $sdkRoot `
    --accept-licenses `
    --default-answer `
    --confirm-command `
    install `
    copy_only=1

if ($LASTEXITCODE -ne 0) {
    throw "Vulkan SDK installer failed with exit code $LASTEXITCODE."
}
if (-not (Test-VulkanSdkReady)) {
    $missingFiles = $requiredSdkFiles |
        Where-Object { -not (Test-Path -LiteralPath (Join-Path $sdkRoot $_) -PathType Leaf) }
    throw "Vulkan SDK installation is incomplete. Missing: $($missingFiles -join ', ')"
}

Write-Host "Vulkan SDK $sdkVersion is ready at $sdkRoot"
