# Quick Systematic Testing Script
Write-Host "Starting systematic tests..." -ForegroundColor Green

$ConfigFile = "C:\Users\ataka\OneDrive\Masaüstü\config\simulation_params.yaml"
$Executable = "..\build\Release\multi_drone_pid_tester.exe"

# Check if files exist
if (!(Test-Path $ConfigFile)) {
    Write-Host "Config file not found: $ConfigFile" -ForegroundColor Red
    exit 1
}

if (!(Test-Path $Executable)) {
    Write-Host "Executable not found: $Executable" -ForegroundColor Red
    exit 1
}

Write-Host "Running all 6 systematic tests..." -ForegroundColor Cyan

# Test configurations
$TestConfigs = @(
    @{Name = "01_Baseline"; FLS = "false"; Wind = "false"; FF = "false" },
    @{Name = "02_FLS_Only"; FLS = "true"; Wind = "false"; FF = "false" },
    @{Name = "03_Wind_Only"; FLS = "false"; Wind = "true"; FF = "false" },
    @{Name = "04_FF_Only"; FLS = "false"; Wind = "false"; FF = "true" },
    @{Name = "05_FLS_Wind"; FLS = "true"; Wind = "true"; FF = "false" },
    @{Name = "06_Full_System"; FLS = "true"; Wind = "true"; FF = "true" }
)

# Function to update config
function Update-Config($TestConfig) {
    $content = Get-Content $ConfigFile
    
    # Update FLS
    for ($i = 0; $i -lt $content.Length; $i++) {
        if ($content[$i] -match "fls:" -and $i + 1 -lt $content.Length -and $content[$i + 1] -match "enable:") {
            $content[$i + 1] = "    enable: $($TestConfig.FLS)"
            break
        }
    }
    
    # Update Wind
    for ($i = 0; $i -lt $content.Length; $i++) {
        if ($content[$i] -match "enable_wind:") {
            $content[$i] = "  enable_wind: $($TestConfig.Wind)"
            break
        }
    }
    
    # Update FeedForward
    for ($i = 0; $i -lt $content.Length; $i++) {
        if ($content[$i] -match "enable_feedforward:") {
            $content[$i] = "    enable_feedforward: $($TestConfig.FF)"
            break
        }
    }
    
    $content | Set-Content $ConfigFile
}

# Run all tests
foreach ($config in $TestConfigs) {
    Write-Host "`n🔬 Running $($config.Name)..." -ForegroundColor Yellow
    Write-Host "FLS: $($config.FLS), Wind: $($config.Wind), FF: $($config.FF)" -ForegroundColor Blue
    
    # Update configuration
    Update-Config $config
    Start-Sleep -Seconds 1
    
    # Run simulation
    try {
        & $Executable
        Write-Host "✅ $($config.Name) completed successfully" -ForegroundColor Green
        
        # Rename output files
        if (Test-Path "..\simulation_outputs") {
            $timestamp = Get-Date -Format "HHmmss"
            Get-ChildItem "..\simulation_outputs\*.csv" | Where-Object { $_.LastWriteTime -gt (Get-Date).AddMinutes(-1) } | ForEach-Object {
                $newName = "$($config.Name)_$($timestamp)_$($_.Name)"
                Rename-Item $_.FullName $newName -ErrorAction SilentlyContinue
            }
        }
    }
    catch {
        Write-Host "❌ $($config.Name) failed: $($_.Exception.Message)" -ForegroundColor Red
    }
    
    Start-Sleep -Seconds 2
}

Write-Host "`n🎉 All tests completed!" -ForegroundColor Green
Write-Host "Check ..\simulation_outputs folder for results" -ForegroundColor Yellow
Write-Host "Run 'python ..\analysis\analyze_test_results.py' to analyze results" -ForegroundColor Cyan 