# Multi-Agent Formation Control - Systematic Testing
Write-Host "🎯 Systematic Testing Script" -ForegroundColor Cyan

# Configuration paths
$ConfigFile = "C:\Users\ataka\OneDrive\Masaüstü\config\simulation_params.yaml"
$Executable = ".\build\Release\multi_drone_pid_tester.exe"

Write-Host "Config file: $ConfigFile"
Write-Host "Executable: $Executable"

# Function to update configuration
function Set-ConfigValue {
    param($Setting, $Value)
    
    $content = Get-Content $ConfigFile
    
    if ($Setting -eq "FLS") {
        # Find FLS enable line
        for ($i = 0; $i -lt $content.Length; $i++) {
            if ($content[$i] -match "fls:" -and $content[$i + 1] -match "enable:") {
                $content[$i + 1] = "    enable: $Value"
                break
            }
        }
    }
    elseif ($Setting -eq "Wind") {
        # Find wind enable line  
        for ($i = 0; $i -lt $content.Length; $i++) {
            if ($content[$i] -match "enable_wind:") {
                $content[$i] = "  enable_wind: $Value"
                break
            }
        }
    }
    elseif ($Setting -eq "FeedForward") {
        # Find feedforward enable line
        for ($i = 0; $i -lt $content.Length; $i++) {
            if ($content[$i] -match "enable_feedforward:") {
                $content[$i] = "    enable_feedforward: $Value"  
                break
            }
        }
    }
    
    $content | Set-Content $ConfigFile
}

# Test scenarios
Write-Host ""
Write-Host "Available test scenarios:" -ForegroundColor Yellow
Write-Host "1. Baseline (No FLS, No Wind, No FF)"
Write-Host "2. FLS Only (FLS enabled)" 
Write-Host "3. Wind Only (Wind enabled)"
Write-Host "4. FeedForward Only (FF enabled)"
Write-Host "5. FLS + Wind"
Write-Host "6. Full System (All enabled)"
Write-Host "7. Run all tests automatically"

$choice = Read-Host "`nEnter test number (1-7)"

switch ($choice) {
    "1" {
        Write-Host "Running Baseline Test..." -ForegroundColor Green
        Set-ConfigValue "FLS" "false"
        Set-ConfigValue "Wind" "false" 
        Set-ConfigValue "FeedForward" "false"
    }
    "2" {
        Write-Host "Running FLS Test..." -ForegroundColor Green
        Set-ConfigValue "FLS" "true"
        Set-ConfigValue "Wind" "false"
        Set-ConfigValue "FeedForward" "false"
    }
    "3" {
        Write-Host "Running Wind Test..." -ForegroundColor Green
        Set-ConfigValue "FLS" "false"
        Set-ConfigValue "Wind" "true"
        Set-ConfigValue "FeedForward" "false"
    }
    "4" {
        Write-Host "Running FeedForward Test..." -ForegroundColor Green
        Set-ConfigValue "FLS" "false"
        Set-ConfigValue "Wind" "false"
        Set-ConfigValue "FeedForward" "true"
    }
    "5" {
        Write-Host "Running FLS + Wind Test..." -ForegroundColor Green
        Set-ConfigValue "FLS" "true"
        Set-ConfigValue "Wind" "true"
        Set-ConfigValue "FeedForward" "false"
    }
    "6" {
        Write-Host "Running Full System Test..." -ForegroundColor Green
        Set-ConfigValue "FLS" "true"
        Set-ConfigValue "Wind" "true"
        Set-ConfigValue "FeedForward" "true"
    }
    "7" {
        Write-Host "Running all tests automatically..." -ForegroundColor Green
        
        $tests = @(
            @{Name = "Baseline"; FLS = "false"; Wind = "false"; FF = "false" },
            @{Name = "FLS_Only"; FLS = "true"; Wind = "false"; FF = "false" },
            @{Name = "Wind_Only"; FLS = "false"; Wind = "true"; FF = "false" },
            @{Name = "FF_Only"; FLS = "false"; Wind = "false"; FF = "true" },
            @{Name = "FLS_Wind"; FLS = "true"; Wind = "true"; FF = "false" },
            @{Name = "Full_System"; FLS = "true"; Wind = "true"; FF = "true" }
        )
        
        foreach ($test in $tests) {
            Write-Host "`n🔬 Running $($test.Name) test..." -ForegroundColor Cyan
            Set-ConfigValue "FLS" $test.FLS
            Set-ConfigValue "Wind" $test.Wind
            Set-ConfigValue "FeedForward" $test.FF
            
            Start-Sleep -Seconds 1
            & $Executable
            
            # Rename output files
            if (Test-Path "simulation_outputs") {
                $timestamp = Get-Date -Format "HHmmss"
                Get-ChildItem "simulation_outputs\*.csv" | ForEach-Object {
                    $newName = "$($test.Name)_$($timestamp)_$($_.Name)"
                    Rename-Item $_.FullName $newName
                }
            }
            
            Write-Host "✅ $($test.Name) test completed" -ForegroundColor Green
            Start-Sleep -Seconds 2
        }
        
        Write-Host "`n🎉 All tests completed!" -ForegroundColor Green
        Write-Host "Check simulation_outputs folder for results" -ForegroundColor Yellow
        exit
    }
    default {
        Write-Host "Invalid choice. Exiting." -ForegroundColor Red
        exit
    }
}

# Run single test
Write-Host "Configuration updated. Running test..." -ForegroundColor Blue
Start-Sleep -Seconds 1

& $Executable

Write-Host "`n✅ Test completed!" -ForegroundColor Green 
Write-Host "Check simulation_outputs folder for results" -ForegroundColor Yellow 