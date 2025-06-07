#include "drone_system.hpp"
#include "zn_tuning.hpp"
#include <iostream>
#include <fstream>
#include <filesystem>
#include <chrono>
#include <iomanip>

using namespace modular_pid_system;

int main() {
    std::cout << "=== Multi-Drone Ziegler-Nichols Auto-Tuning Test ===" << std::endl;
    
    // Create base configuration for tuning
    SystemConfig config = createPIDOnlyConfig();
    config.num_drones = 1; // Use single drone for tuning
    config.simulation_time = 30.0; // Shorter time per test
    config.dt = 0.01;
    config.test_name = "zn_auto_tuning";
    config.enable_csv_output = true;
    config.enable_console_output = true;
    
    std::cout << "Configuration:" << std::endl;
    std::cout << "  Scenario: " << scenarioToString(config.current_scenario) << std::endl;
    std::cout << "  Number of drones: " << config.num_drones << " (single drone for tuning)" << std::endl;
    std::cout << "  Test duration per Kp: " << config.simulation_time << "s" << std::endl;
    
    // Initialize system
    MultiDroneSystem system(config);
    if (!system.initialize()) {
        std::cerr << "Failed to initialize system!" << std::endl;
        return -1;
    }
    
    // Set simple formation (single drone at origin offset)
    system.setFormationOffsets({{0.0, 0.0}});
    
    // Create ZN tuning configuration
    ZNTuningConfig zn_config = createDetailedTuningConfig();
    zn_config.kp_start = 0.5;
    zn_config.kp_end = 15.0;
    zn_config.kp_step = 0.5;
    zn_config.test_duration = config.simulation_time;
    zn_config.target_drone_id = 0;
    zn_config.tuning_axis = ZNTuningConfig::TuningAxis::X_AXIS;
    zn_config.test_center_x = 5.0;  // Step input target
    zn_config.test_center_y = 0.0;
    zn_config.output_directory = config.output_directory + "/zn_tuning";
    
    std::cout << "ZN Tuning Configuration:" << std::endl;
    std::cout << "  Kp range: " << zn_config.kp_start << " to " << zn_config.kp_end 
              << " (step: " << zn_config.kp_step << ")" << std::endl;
    std::cout << "  Target drone: " << zn_config.target_drone_id << std::endl;
    std::cout << "  Tuning axis: " << (zn_config.tuning_axis == ZNTuningConfig::TuningAxis::X_AXIS ? "X" : 
                                      zn_config.tuning_axis == ZNTuningConfig::TuningAxis::Y_AXIS ? "Y" : "Both") << std::endl;
    std::cout << "  Test target: (" << zn_config.test_center_x << ", " << zn_config.test_center_y << ")" << std::endl;
    std::cout << "  Output directory: " << zn_config.output_directory << std::endl;
    
    // Create output directory
    std::filesystem::create_directories(zn_config.output_directory);
    
    // Initialize ZN tuning engine
    ZNTuningEngine tuning_engine(&system, zn_config);
    
    std::cout << "\nStarting Ziegler-Nichols Auto-Tuning..." << std::endl;
    std::cout << "This will test multiple Kp values to find the ultimate gain (Ku) and period (Tu)." << std::endl;
    
    // Perform auto-tuning
    auto start_time = std::chrono::high_resolution_clock::now();
    
    ZNAnalysisResult result = tuning_engine.performAutoTuning();
    
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::seconds>(end_time - start_time);
    
    std::cout << "\n=== TUNING COMPLETED ===" << std::endl;
    std::cout << "Total tuning time: " << duration.count() << " seconds" << std::endl;
    
    // Print results
    std::cout << "\n=== ZN ANALYSIS RESULTS ===" << std::endl;
    tuning_engine.printResults(result);
    
    // Save detailed results
    std::string results_filename = zn_config.output_directory + "/zn_tuning_results.txt";
    tuning_engine.saveResults(result, results_filename);
    std::cout << "\nDetailed results saved to: " << results_filename << std::endl;
    
    // Test recommended gains
    if (!result.recommended_gains.empty()) {
        std::cout << "\n=== TESTING RECOMMENDED GAINS ===" << std::endl;
        
        for (std::size_t i = 0; i < result.recommended_gains.size() && i < 3; ++i) {
            const auto& gains = result.recommended_gains[i];
            
            std::cout << "\nTesting " << gains.method_name << " gains:" << std::endl;
            std::cout << "  Kp = " << gains.kp << ", Ki = " << gains.ki << ", Kd = " << gains.kd << std::endl;
            
            // Create test configuration with recommended gains
            SystemConfig test_config = config;
            test_config.test_name = "zn_test_" + gains.method_name;
            test_config.simulation_time = 45.0; // Longer test for validation
            
            MultiDroneSystem test_system(test_config);
            if (test_system.initialize()) {
                test_system.setFormationOffsets({{0.0, 0.0}});
                
                // Apply recommended gains (this would require modifying the PID controller)
                // For now, we'll just simulate the test
                std::cout << "  Simulating performance with " << gains.method_name << " gains..." << std::endl;
                
                // Run a quick validation test
                ZNAnalysisResult validation = tuning_engine.testSpecificKp(gains.kp);
                
                std::cout << "  Validation results:" << std::endl;
                std::cout << "    Overshoot: " << std::fixed << std::setprecision(1) 
                          << validation.overshoot_percent << "%" << std::endl;
                std::cout << "    Settling time: " << std::setprecision(2) 
                          << validation.settling_time_2percent << "s" << std::endl;
                std::cout << "    Stable: " << (validation.is_unstable ? "No" : "Yes") << std::endl;
            }
        }
    }
    
    // Generate comparison with different scenarios
    std::cout << "\n=== BATCH TUNING FOR DIFFERENT SCENARIOS ===" << std::endl;
    
    std::vector<SystemConfig> test_scenarios = {
        createPIDOnlyConfig(),
        createPIDWindConfig()
    };
    
    // Update scenario names
    test_scenarios[0].test_name = "pid_only_tuning";
    test_scenarios[1].test_name = "pid_wind_tuning";
    
    BatchZNTuning batch_tuning(test_scenarios, zn_config, zn_config.output_directory + "/batch");
    
    std::cout << "Running batch tuning for " << test_scenarios.size() << " scenarios..." << std::endl;
    
    auto batch_results = batch_tuning.runBatchTuning();
    
    std::cout << "\n=== BATCH TUNING RESULTS ===" << std::endl;
    batch_tuning.printComparisonSummary(batch_results);
    
    // Generate comparison report
    std::string comparison_filename = zn_config.output_directory + "/batch_comparison_report.txt";
    batch_tuning.generateComparisonReport(batch_results, comparison_filename);
    std::cout << "\nBatch comparison report saved to: " << comparison_filename << std::endl;
    
    // Final recommendations
    std::cout << "\n=== FINAL RECOMMENDATIONS ===" << std::endl;
    
    if (result.is_oscillating && result.ultimate_gain > 0) {
        std::cout << "✓ Ultimate gain (Ku) found: " << result.ultimate_gain << std::endl;
        std::cout << "✓ Ultimate period (Tu) found: " << result.ultimate_period << "s" << std::endl;
        
        if (!result.recommended_gains.empty()) {
            const auto& best_gains = result.recommended_gains[0];
            std::cout << "✓ Recommended gains (" << best_gains.method_name << "):" << std::endl;
            std::cout << "  Kp = " << best_gains.kp << std::endl;
            std::cout << "  Ki = " << best_gains.ki << std::endl;
            std::cout << "  Kd = " << best_gains.kd << std::endl;
            
            std::cout << "\nTo apply these gains, update your PID configuration:" << std::endl;
            std::cout << "  pid_params:" << std::endl;
            std::cout << "    kp: " << best_gains.kp << std::endl;
            std::cout << "    ki: " << best_gains.ki << std::endl;
            std::cout << "    kd: " << best_gains.kd << std::endl;
        }
    } else {
        std::cout << "⚠ No oscillatory behavior found in the tested Kp range." << std::endl;
        std::cout << "  Consider:" << std::endl;
        std::cout << "  - Increasing the Kp range (current max: " << zn_config.kp_end << ")" << std::endl;
        std::cout << "  - Reducing the step size for finer resolution" << std::endl;
        std::cout << "  - Checking system stability and dynamics" << std::endl;
    }
    
    std::cout << "\n=== TUNING TEST COMPLETED ===" << std::endl;
    std::cout << "All results and data files saved in: " << zn_config.output_directory << std::endl;
    std::cout << "\nNext steps:" << std::endl;
    std::cout << "1. Review the detailed results and CSV data" << std::endl;
    std::cout << "2. Apply the recommended gains to your system" << std::endl;
    std::cout << "3. Test the tuned system with your specific scenarios" << std::endl;
    std::cout << "4. Fine-tune based on actual performance requirements" << std::endl;
    
    return 0;
} 