#include "drone_system.hpp"
#include <iostream>
#include <fstream>
#include <filesystem>
#include <chrono>
#include <iomanip>

using namespace modular_pid_system;

int main() {
    std::cout << "=== Multi-Drone PID-Only Test ===" << std::endl;
    
    // Create PID-only configuration
    SystemConfig config = createPIDOnlyConfig();
    config.num_drones = 3;
    config.simulation_time = 60.0;
    config.dt = 0.01;
    config.test_name = "pid_only_formation_test";
    
    std::cout << "Configuration:" << std::endl;
    std::cout << "  Scenario: " << scenarioToString(config.current_scenario) << std::endl;
    std::cout << "  Number of drones: " << config.num_drones << std::endl;
    std::cout << "  Simulation time: " << config.simulation_time << "s" << std::endl;
    std::cout << "  PID enabled: " << (config.enable_pid ? "Yes" : "No") << std::endl;
    std::cout << "  Wind enabled: " << (config.enable_wind ? "Yes" : "No") << std::endl;
    std::cout << "  Fuzzy Logic enabled: " << (config.enable_fuzzy_logic ? "Yes" : "No") << std::endl;
    
    // Initialize system
    MultiDroneSystem system(config);
    if (!system.initialize()) {
        std::cerr << "Failed to initialize system!" << std::endl;
        return -1;
    }
    
    // Set triangle formation
    auto formation = createTriangleFormation(2.0);
    system.setFormationOffsets(formation);
    
    // Create output directory
    std::filesystem::create_directories(config.output_directory);
    
    // Setup CSV output
    std::ofstream csv_file;
    std::string csv_filename = config.output_directory + "/" + config.test_name + "_data.csv";
    
    if (config.enable_csv_output) {
        csv_file.open(csv_filename);
        if (csv_file.is_open()) {
            // Write CSV header
            csv_file << "time";
            for (int i = 0; i < config.num_drones; ++i) {
                csv_file << ",drone" << i << "_target_x,drone" << i << "_pos_x,drone" << i << "_error_x"
                         << ",drone" << i << "_target_y,drone" << i << "_pos_y,drone" << i << "_error_y"
                         << ",drone" << i << "_cmd_x,drone" << i << "_cmd_y";
            }
            csv_file << "\n";
        }
    }
    
    // Define test phases
    struct TestPhase {
        double start_time;
        double duration;
        double center_x;
        double center_y;
        std::string description;
    };
    
    std::vector<TestPhase> phases = {
        {0.0, 15.0, 0.0, 0.0, "Initial formation at origin"},
        {15.0, 15.0, 5.0, 5.0, "Move to (5,5)"},
        {30.0, 15.0, -3.0, 7.0, "Move to (-3,7)"},
        {45.0, 15.0, 2.0, -4.0, "Move to (2,-4)"}
    };
    
    // Simulation variables
    double time_now = 0.0;
    std::size_t current_phase = 0;
    double last_console_output = 0.0;
    const double console_interval = 1.0; // Print every 1 second
    
    std::cout << "\nStarting simulation..." << std::endl;
    
    // Main simulation loop
    while (time_now < config.simulation_time && current_phase < phases.size()) {
        // Check for phase transitions
        if (current_phase < phases.size() && time_now >= phases[current_phase].start_time) {
            std::cout << "\nPhase " << current_phase + 1 << " (" << time_now << "s): " 
                      << phases[current_phase].description << std::endl;
            
            // Set new formation center and start metrics
            system.setFormationCenter(phases[current_phase].center_x, phases[current_phase].center_y);
            system.startMetricsPhase(static_cast<int>(current_phase), phases[current_phase].center_x, phases[current_phase].center_y);
            
            current_phase++;
        }
        
        // Update system
        system.update(time_now, config.dt);
        
        // Get current drone states
        const auto& drones = system.getDrones();
        
        // Write to CSV
        if (config.enable_csv_output && csv_file.is_open()) {
            csv_file << std::fixed << std::setprecision(3) << time_now;
            
            for (std::size_t i = 0; i < static_cast<std::size_t>(config.num_drones); ++i) {
                // For this simple test, we'll calculate targets based on current phase
                double target_x = 0.0, target_y = 0.0;
                if (current_phase > 0 && current_phase <= phases.size()) {
                    target_x = phases[current_phase-1].center_x + formation[i].first;
                    target_y = phases[current_phase-1].center_y + formation[i].second;
                }
                
                double error_x = target_x - drones[i].position_x;
                double error_y = target_y - drones[i].position_y;
                
                csv_file << "," << target_x << "," << drones[i].position_x << "," << error_x
                         << "," << target_y << "," << drones[i].position_y << "," << error_y
                         << ",0.0,0.0"; // Command values would need to be stored separately
            }
            csv_file << "\n";
        }
        
        // Console output
        if (config.enable_console_output && (time_now - last_console_output) >= console_interval) {
            std::cout << "T=" << std::fixed << std::setprecision(1) << time_now 
                      << "s, Phase=" << current_phase;
            if (!drones.empty()) {
                std::cout << ", Drone0: (" << std::setprecision(2) 
                          << drones[0].position_x << "," << drones[0].position_y << ")";
            }
            std::cout << std::endl;
            last_console_output = time_now;
        }
        
        time_now += config.dt;
    }
    
    // Close CSV file
    if (csv_file.is_open()) {
        csv_file.close();
        std::cout << "\nCSV data saved to: " << csv_filename << std::endl;
    }
    
    // Finalize and print metrics for all phases
    std::cout << "\n=== PERFORMANCE METRICS ===" << std::endl;
    for (std::size_t phase_idx = 0; phase_idx < current_phase - 1 && phase_idx < phases.size(); ++phase_idx) {
        system.finalizeMetrics(static_cast<int>(phase_idx), phases[phase_idx].start_time);
        system.printMetrics(static_cast<int>(phase_idx));
        
        // Save metrics to file
        if (config.enable_metrics_analysis) {
            std::string metrics_filename = config.output_directory + "/" + 
                                         config.test_name + "_metrics.txt";
            system.saveMetricsToFile(metrics_filename, static_cast<int>(phase_idx));
        }
    }
    
    std::cout << "\n=== TEST COMPLETED ===" << std::endl;
    std::cout << "Test scenario: " << scenarioToString(config.current_scenario) << std::endl;
    std::cout << "Total simulation time: " << time_now << "s" << std::endl;
    std::cout << "Output directory: " << config.output_directory << std::endl;
    
    return 0;
} 