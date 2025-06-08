# Cursor Environment Refresh Script
# This script helps resolve common development environment issues

Write-Host "=== Cursor Environment Refresh Script ===" -ForegroundColor Green

# Check if we're in the project directory
if (-not (Test-Path "vcpkg.json")) {
    Write-Host "Error: Not in project root directory. Please run this script from the project root." -ForegroundColor Red
    exit 1
}

# Step 1: Ensure yaml-cpp is installed
Write-Host "Step 1: Checking yaml-cpp installation..." -ForegroundColor Yellow
$vcpkgPath = "C:\vcpkg\vcpkg.exe"
if (Test-Path $vcpkgPath) {
    Write-Host "Installing/updating yaml-cpp..." -ForegroundColor Blue
    try {
        & $vcpkgPath install yaml-cpp:x64-windows
        Write-Host "yaml-cpp installation completed" -ForegroundColor Green
    }
    catch {
        Write-Host "Failed to install yaml-cpp" -ForegroundColor Red
    }
}
else {
    Write-Host "vcpkg not found at $vcpkgPath" -ForegroundColor Red
}

# Step 2: Regenerate build files
Write-Host "Step 2: Regenerating build files..." -ForegroundColor Yellow
if (Test-Path "build") {
    Write-Host "Building project..." -ForegroundColor Blue
    cmake --build build --config Release
}
else {
    Write-Host "Configuring CMake..." -ForegroundColor Blue
    cmake -S agent_control_pkg -B build -DCMAKE_TOOLCHAIN_FILE=C:/vcpkg/scripts/buildsystems/vcpkg.cmake
    cmake --build build --config Release
}

# Step 3: Check compile_commands.json
Write-Host "Step 3: Checking compile_commands.json..." -ForegroundColor Yellow
if (Test-Path "build\compile_commands.json") {
    Write-Host "compile_commands.json exists" -ForegroundColor Green
}
else {
    Write-Host "compile_commands.json not found" -ForegroundColor Red
}

# Step 4: Instructions for Cursor restart
Write-Host "Step 4: Manual steps in Cursor:" -ForegroundColor Yellow
Write-Host "1. Press Ctrl+Shift+P and run 'clangd: Restart language server'" -ForegroundColor Cyan
Write-Host "2. Press Ctrl+Shift+P and run 'C/C++: Restart IntelliSense'" -ForegroundColor Cyan
Write-Host "3. Press Ctrl+Shift+P and run 'Developer: Reload Window'" -ForegroundColor Cyan

Write-Host "=== Refresh Complete ===" -ForegroundColor Green
Write-Host "If issues persist, check the CURSOR_SETUP.md file for additional troubleshooting steps." -ForegroundColor White 