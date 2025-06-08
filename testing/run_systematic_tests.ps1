# Multi-Agent Formation Control - Systematic Testing Script
# Automates configuration changes and test execution

param(
    [switch]$RunAll,
    [int]$TestNumber = 0,
    [switch]$Help
)

# Configuration file path (updated for new structure)
$ConfigPath = "C:\Users\ataka\OneDrive\Masaüstü\config\simulation_params.yaml"
$ExePath = "..\build\Release\multi_drone_pid_tester.exe"
$OutputDir = "..\results\simulation_outputs"

# Test configurations
$TestConfigs = @{
    1 = @{
        Name        = "01_Baseline_Clean"
        Description = "Basic PID with improvements, no FLS/Wind/FF"
        FLS         = $false
        Wind        = $false
        FeedForward = $false
    }
    2 = @{
        Name        = "02_FLS_Effect"
        Description = "Baseline + Fuzzy Logic System"
        FLS         = $true
        Wind        = $false
        FeedForward = $false
    }
    3 = @{
        Name        = "03_Wind_Challenge"
        Description = "Baseline + Wind Disturbances"
        FLS         = $false
        Wind        = $true
        FeedForward = $false
    }
    4 = @{
        Name        = "04_FeedForward_Benefit"
        Description = "Baseline + Feed-Forward Control"
        FLS         = $false
        Wind        = $false
        FeedForward = $true
    }
    5 = @{
        Name        = "05_FLS_vs_Wind"
        Description = "FLS handling wind disturbances"
        FLS         = $true
        Wind        = $true
        FeedForward = $false
    }
    6 = @{
        Name        = "06_Full_System_Optimal"
        Description = "All features enabled"
        FLS         = $true
        Wind        = $true
        FeedForward = $true
    }
}

function Show-Help {
    Write-Host "🎯 Multi-Agent Formation Control - Systematic Testing" -ForegroundColor Cyan
    Write-Host "=" * 55
    Write-Host ""
    Write-Host "Usage:" -ForegroundColor Yellow
    Write-Host "  .\run_systematic_tests.ps1 -RunAll          # Run all tests (1-6)"
    Write-Host "  .\run_systematic_tests.ps1 -TestNumber 3    # Run specific test"
    Write-Host "  .\run_systematic_tests.ps1 -Help            # Show this help"
    Write-Host ""
    Write-Host "Available Tests:" -ForegroundColor Yellow
    foreach ($key in $TestConfigs.Keys | Sort-Object) {
        $config = $TestConfigs[$key]
        Write-Host "  $key. $($config.Name) - $($config.Description)"
    }
    Write-Host ""
    Write-Host "Examples:" -ForegroundColor Green
    Write-Host "  .\run_systematic_tests.ps1 -TestNumber 1    # Test baseline performance"
    Write-Host "  .\run_systematic_tests.ps1 -TestNumber 2    # Test FLS effectiveness"
    Write-Host "  .\run_systematic_tests.ps1 -RunAll          # Complete systematic testing"
}

function Update-ConfigParameter {
    param(
        [string]$ConfigPath,
        [string]$Parameter,
        [bool]$Value
    )
    
    $content = Get-Content $ConfigPath
    $valueStr = $Value.ToString().ToLower()
    
    switch ($Parameter) {
        "FLS" {
            # Find FLS enable line (second occurrence of 'enable:')
            for ($i = 0; $i -lt $content.Length; $i++) {
                if ($content[$i] -match "^\s*enable:\s*" -and $content[$i - 1] -match "fls:") {
                    $content[$i] = "    enable: $valueStr"
                    break
                }
            }
        }
        "Wind" {
            # Find wind enable line
            for ($i = 0; $i -lt $content.Length; $i++) {
                if ($content[$i] -match "^\s*enable_wind:\s*") {
                    $content[$i] = "  enable_wind: $valueStr"
                    break
                }
            }
        }
        "FeedForward" {
            # Find feedforward enable line
            for ($i = 0; $i -lt $content.Length; $i++) {
                if ($content[$i] -match "^\s*enable_feedforward:\s*") {
                    $content[$i] = "    enable_feedforward: $valueStr"
                    break
                }
            }
        }
    }
    
    $content | Set-Content $ConfigPath
}

function Run-SingleTest {
    param(
        [int]$TestNum
    )
    
    if (-not $TestConfigs.ContainsKey($TestNum)) {
        Write-Host "❌ Invalid test number: $TestNum" -ForegroundColor Red
        return $false
    }
    
    $config = $TestConfigs[$TestNum]
    
    Write-Host ""
    Write-Host "🔬 Running Test $TestNum`: $($config.Name)" -ForegroundColor Cyan
    Write-Host "=" * 50
    Write-Host "Description: $($config.Description)" -ForegroundColor Yellow
    Write-Host "Configuration: FLS=$($config.FLS), Wind=$($config.Wind), FF=$($config.FeedForward)"
    
    # Apply configuration changes
    try {
        Update-ConfigParameter -ConfigPath $ConfigPath -Parameter "FLS" -Value $config.FLS
        Update-ConfigParameter -ConfigPath $ConfigPath -Parameter "Wind" -Value $config.Wind  
        Update-ConfigParameter -ConfigPath $ConfigPath -Parameter "FeedForward" -Value $config.FeedForward
        
        Write-Host "✅ Configuration updated successfully" -ForegroundColor Green
        
        # Brief pause to ensure file is written
        Start-Sleep -Seconds 1
        
        # Run the simulation
        Write-Host "🚀 Starting simulation..." -ForegroundColor Blue
        $result = & $ExePath
        
        if ($LASTEXITCODE -eq 0) {
            Write-Host "✅ Test completed successfully" -ForegroundColor Green
            
            # Organize results
            $timestamp = Get-Date -Format "yyyyMMdd_HHmmss"
            $testDir = Join-Path $OutputDir "$timestamp`_$($config.Name)"
            
            if (-not (Test-Path $testDir)) {
                New-Item -ItemType Directory -Path $testDir -Force | Out-Null
            }
            
            # Copy recent output files
            $outputPath = "..\simulation_outputs"
            if (Test-Path $outputPath) {
                $cutoffTime = (Get-Date).AddMinutes(-3)
                
                Get-ChildItem "$outputPath\*.csv" | Where-Object { $_.LastWriteTime -gt $cutoffTime } | ForEach-Object {
                    $newName = "$($config.Name)_$($_.Name)"
                    Copy-Item $_.FullName -Destination (Join-Path $testDir $newName)
                }
                
                Get-ChildItem "$outputPath\*.txt" | Where-Object { $_.LastWriteTime -gt $cutoffTime } | ForEach-Object {
                    $newName = "$($config.Name)_$($_.Name)"
                    Copy-Item $_.FullName -Destination (Join-Path $testDir $newName)
                }
            }
            
            Write-Host "📁 Results saved to: $testDir" -ForegroundColor Green
            return $true
        }
        else {
            Write-Host "❌ Test failed with exit code: $LASTEXITCODE" -ForegroundColor Red
            return $false
        }
    }
    catch {
        Write-Host "💥 Test failed with error: $($_.Exception.Message)" -ForegroundColor Red
        return $false
    }
}

function Run-AllTests {
    Write-Host "🚀 Starting Systematic Test Suite (6 tests)" -ForegroundColor Cyan
    Write-Host "📅 Timestamp: $(Get-Date -Format 'yyyy-MM-dd HH:mm:ss')" -ForegroundColor Yellow
    Write-Host "=" * 60
    
    $results = @{}
    $successCount = 0
    
    # Create output directory
    if (-not (Test-Path $OutputDir)) {
        New-Item -ItemType Directory -Path $OutputDir -Force | Out-Null
    }
    
    # Run all tests
    for ($i = 1; $i -le 6; $i++) {
        Write-Host ""
        Write-Host "📊 Progress: $i/6" -ForegroundColor Blue
        
        $success = Run-SingleTest -TestNum $i
        $results[$i] = $success
        
        if ($success) {
            $successCount++
        }
        
        # Brief pause between tests
        Start-Sleep -Seconds 2
    }
    
    # Generate summary
    Write-Host ""
    Write-Host "🎉 Testing Complete!" -ForegroundColor Green
    Write-Host "✅ Successful: $successCount/6" -ForegroundColor Green
    Write-Host "📁 Results saved in: $OutputDir" -ForegroundColor Yellow
    
    # Generate summary report
    $reportPath = Join-Path $OutputDir "test_summary_$(Get-Date -Format 'yyyyMMdd_HHmmss').txt"
    $summary = @"
Multi-Agent Formation Control - Systematic Testing Summary
========================================================
Test Date: $(Get-Date -Format 'yyyy-MM-dd HH:mm:ss')
Total Tests: 6
Successful: $successCount
Failed: $(6 - $successCount)

Detailed Results:
----------------
"@
    
    foreach ($key in $results.Keys | Sort-Object) {
        $config = $TestConfigs[$key]
        $status = if ($results[$key]) { "PASS" } else { "FAIL" }
        $summary += "`n$($config.Name) - $status"
    }
    
    $summary += @"

Analysis Instructions:
=====================
1. Compare baseline vs improved performance metrics
2. Analyze FLS effectiveness under wind disturbances  
3. Evaluate feed-forward impact on tracking accuracy
4. Examine combined system performance
5. Generate comparative plots using existing Python scripts

Next Steps:
----------
- Run: python final_project_analysis.py
- Run: python plot_simulation.py
- Create performance comparison charts
"@
    
    $summary | Out-File -FilePath $reportPath -Encoding UTF8
    Write-Host "📋 Summary report generated: $reportPath" -ForegroundColor Blue
}

# Main execution
if ($Help) {
    Show-Help
    exit 0
}

# Check if executable exists
if (-not (Test-Path $ExePath)) {
    Write-Host "❌ Executable not found: $ExePath" -ForegroundColor Red
    Write-Host "Please build the project first:" -ForegroundColor Yellow
    Write-Host "  cd build" 
    Write-Host "  cmake --build . --config Release"
    exit 1
}

# Check if config file exists
if (-not (Test-Path $ConfigPath)) {
    Write-Host "❌ Configuration file not found: $ConfigPath" -ForegroundColor Red
    exit 1
}

if ($RunAll) {
    Run-AllTests
}
elseif ($TestNumber -gt 0) {
    $success = Run-SingleTest -TestNum $TestNumber
    if ($success) {
        Write-Host ""
        Write-Host "🎉 Test $TestNumber completed successfully!" -ForegroundColor Green
        Write-Host "📊 You can now run analysis scripts on the generated data" -ForegroundColor Blue
    }
}
else {
    Show-Help
} 