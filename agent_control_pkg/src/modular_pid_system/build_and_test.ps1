# PowerShell script to build and test the modular PID system
# Usage: .\build_and_test.ps1 [test_name]
# Example: .\build_and_test.ps1 pid_only

param(
    [string]$TestName = "all",
    [switch]$Clean = $false,
    [switch]$Verbose = $false
)

Write-Host "=== Modular PID System Build and Test Script ===" -ForegroundColor Green

# Get the script directory and project root
$ScriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$ProjectRoot = Split-Path -Parent (Split-Path -Parent $ScriptDir)

Write-Host "Script Directory: $ScriptDir" -ForegroundColor Yellow
Write-Host "Project Root: $ProjectRoot" -ForegroundColor Yellow

# Set build directory
$BuildDir = Join-Path $ProjectRoot "build"

# Clean build if requested
if ($Clean) {
    Write-Host "Cleaning build directory..." -ForegroundColor Yellow
    if (Test-Path $BuildDir) {
        Remove-Item -Recurse -Force $BuildDir
    }
}

# Create build directory
if (-not (Test-Path $BuildDir)) {
    Write-Host "Creating build directory..." -ForegroundColor Yellow
    New-Item -ItemType Directory -Path $BuildDir | Out-Null
}

# Change to build directory
Set-Location $BuildDir

try {
    # Configure with CMake
    Write-Host "Configuring with CMake..." -ForegroundColor Yellow
    if ($Verbose) {
        cmake .. -DCMAKE_BUILD_TYPE=Release
    }
    else {
        cmake .. -DCMAKE_BUILD_TYPE=Release | Out-Null
    }
    
    if ($LASTEXITCODE -ne 0) {
        throw "CMake configuration failed"
    }

    # Build the project
    Write-Host "Building project..." -ForegroundColor Yellow
    if ($Verbose) {
        cmake --build . --config Release
    }
    else {
        cmake --build . --config Release | Out-Null
    }
    
    if ($LASTEXITCODE -ne 0) {
        throw "Build failed"
    }

    Write-Host "Build completed successfully!" -ForegroundColor Green

    # Check if executables exist
    $ModularBuildDir = Join-Path $BuildDir "src/modular_pid_system"
    $Executables = @{
        "pid_only"     = "pid_only_test.exe"
        "pid_wind"     = "pid_wind_test.exe" 
        "pid_wind_flc" = "pid_wind_flc_test.exe"
        "zn_tuning"    = "zn_tuning_test.exe"
    }

    Write-Host "`nAvailable tests:" -ForegroundColor Cyan
    foreach ($test in $Executables.Keys) {
        $exePath = Join-Path $ModularBuildDir $Executables[$test]
        if (Test-Path $exePath) {
            Write-Host "  ✓ $test" -ForegroundColor Green
        }
        else {
            Write-Host "  ✗ $test (not built)" -ForegroundColor Red
        }
    }

    # Run tests based on parameter
    if ($TestName -eq "all") {
        Write-Host "`nRunning all available tests..." -ForegroundColor Cyan
        
        foreach ($test in $Executables.Keys) {
            $exePath = Join-Path $ModularBuildDir $Executables[$test]
            if (Test-Path $exePath) {
                Write-Host "`n--- Running $test test ---" -ForegroundColor Yellow
                & $exePath
                if ($LASTEXITCODE -ne 0) {
                    Write-Host "Test $test failed with exit code $LASTEXITCODE" -ForegroundColor Red
                }
                else {
                    Write-Host "Test $test completed successfully" -ForegroundColor Green
                }
            }
        }
    }
    elseif ($Executables.ContainsKey($TestName)) {
        $exePath = Join-Path $ModularBuildDir $Executables[$TestName]
        if (Test-Path $exePath) {
            Write-Host "`n--- Running $TestName test ---" -ForegroundColor Yellow
            & $exePath
            if ($LASTEXITCODE -ne 0) {
                Write-Host "Test $TestName failed with exit code $LASTEXITCODE" -ForegroundColor Red
            }
            else {
                Write-Host "Test $TestName completed successfully" -ForegroundColor Green
            }
        }
        else {
            Write-Host "Test executable for $TestName not found: $exePath" -ForegroundColor Red
        }
    }
    else {
        Write-Host "Unknown test name: $TestName" -ForegroundColor Red
        Write-Host "Available tests: $($Executables.Keys -join ', '), all" -ForegroundColor Yellow
    }

    # Show output directory
    $OutputDir = Join-Path $ProjectRoot "simulation_outputs"
    if (Test-Path $OutputDir) {
        Write-Host "`nOutput files generated in: $OutputDir" -ForegroundColor Cyan
        $OutputFiles = Get-ChildItem $OutputDir -File | Select-Object -First 5
        foreach ($file in $OutputFiles) {
            Write-Host "  - $($file.Name)" -ForegroundColor Gray
        }
        if ((Get-ChildItem $OutputDir -File).Count -gt 5) {
            Write-Host "  ... and more" -ForegroundColor Gray
        }
    }

}
catch {
    Write-Host "Error: $_" -ForegroundColor Red
    exit 1
}
finally {
    # Return to original directory
    Set-Location $ScriptDir
}

Write-Host "`n=== Build and Test Complete ===" -ForegroundColor Green 