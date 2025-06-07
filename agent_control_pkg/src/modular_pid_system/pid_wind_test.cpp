#include "drone_system.hpp"
#include <iostream>
#include <fstream>
#include <filesystem>
#include <chrono>
#include <iomanip>

using namespace modular_pid_system;

int main() {
    std::cout << "=== Multi-Drone PID + Wind Disturbance Test ===" << std::endl;
    
    // Create PID + Wind configuration
    SystemConfig config = createPIDWindConfig();
    config.num_drones = 3;
    config.simulation_time = 60.0;
    config.dt = 0.01;
    config.test_name = "pid_wind_formation_test";
    
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
    
    // Configure wind model
    auto& wind_model = system.getWindModel();
    wind_model.is_sine_wave = true;
    wind_model.sine_frequency = 0.5; // 0.5 rad/s
    wind_model.amplitude_x = 2.0;    // 2 N/kg wind force in X
    wind_model.amplitude_y = 1.5;    // 1.5 N/kg wind force in Y
    
    std::cout << "Wind Configuration:" << std::endl;
    std::cout << "  Type: " << (wind_model.is_sine_wave ? "Sinusoidal" : "Constant") << std::endl;
    std::cout << "  X Amplitude: " << wind_model.amplitude_x << " N/kg" << std::endl;
    std::cout << "  Y Amplitude: " << wind_model.amplitude_y << " N/kg" << std::endl;
    std::cout << "  Frequency: " << wind_model.sine_frequency << " rad/s" << std::endl;
    
    // Create output directory
    std::filesystem::create_directories(config.output_directory);
    
    // Setup CSV output
    std::ofstream csv_file;
    std::string csv_filename = config.output_directory + "/" + config.test_name + "_data.csv";
    
    if (config.enable_csv_output) {
        csv_file.open(csv_filename);
        if (csv_file.is_open()) {
            // Write CSV header
            csv_file << "time,wind_x,wind_y";
            for (int i = 0; i < config.num_drones; ++i) {
                csv_file << ",drone" << i << "_target_x,drone" << i << "_pos_x,drone" << i << "_error_x"
                         << ",drone" << i << "_target_y,drone" << i << "_pos_y,drone" << i << "_error_y"
                         << ",drone" << i << "_cmd_x,drone" << i << "_cmd_y";
            }
            csv_file << "\n";
        }
    }
    
    // Define test phases with different wind conditions
    struct TestPhase {
        double start_time;
        double duration;
        double center_x;
        double center_y;
        std::string description;
        bool enable_wind_for_phase;
        double wind_amplitude_multiplier;
    };
    
    std::vector<TestPhase> phases = {
        {0.0, 15.0, 0.0, 0.0, "Initial formation (no wind)", false, 0.0},
        {15.0, 15.0, 5.0, 5.0, "Move to (5,5) with light wind", true, 0.5},
        {30.0, 15.0, -3.0, 7.0, "Move to (-3,7) with moderate wind", true, 1.0},
        {45.0, 15.0, 2.0, -4.0, "Move to (2,-4) with strong wind", true, 1.5}
    };
    
    // Simulation variables
    double time_now = 0.0;
    std::size_t current_phase = 0;
    double last_console_output = 0.0;
    const double console_interval = 1.0; // Print every 1 second
    
    std::cout << "\nStarting simulation with wind disturbances..." << std::endl;
    
    // Main simulation loop
    while (time_now < config.simulation_time && current_phase < phases.size()) {
        // Check for phase transitions
        if (current_phase < phases.size() && time_now >= phases[current_phase].start_time) {
            std::cout << "\nPhase " << current_phase + 1 << " (" << time_now << "s): " 
                      << phases[current_phase].description << std::endl;
            
            // Update wind model for phase
            auto& wind_model = system.getWindModel();
            switch (current_phase) {
                case 0: // No wind
                    wind_model.wind_x = 0.0;
                    wind_model.wind_y = 0.0;
                    wind_model.is_sine_wave = false;
                    break;
                case 1: // Constant wind
                    wind_model.wind_x = 1.5;
                    wind_model.wind_y = 0.8;
                    wind_model.is_sine_wave = false;
                    break;
                case 2: // Variable wind
                    wind_model.amplitude_x = 2.0;
                    wind_model.amplitude_y = 1.5;
                    wind_model.sine_frequency = 0.5; // 0.5 Hz
                    wind_model.is_sine_wave = true;
                    break;
                case 3: // Strong constant wind
                    wind_model.wind_x = -2.0;
                    wind_model.wind_y = 2.5;
                    wind_model.is_sine_wave = false;
                    break;
                default:
                    break;
            }
            
            // Set new formation center and start metrics
            system.setFormationCenter(phases[current_phase].center_x, phases[current_phase].center_y);
            system.startMetricsPhase(static_cast<int>(current_phase), phases[current_phase].center_x, phases[current_phase].center_y);
            
            current_phase++;
        }
        
        // Update system
        system.update(time_now, config.dt);
        
        // Get current drone states and wind
        const auto& drones = system.getDrones();
        const auto& wind_model = system.getWindModel();
        
        // Write to CSV
        if (config.enable_csv_output && csv_file.is_open()) {
            csv_file << std::fixed << std::setprecision(3) << time_now;
            
            for (std::size_t i = 0; i < static_cast<std::size_t>(config.num_drones); ++i) {
                // Calculate targets based on current phase
                double target_x = 0.0, target_y = 0.0;
                if (current_phase > 0 && current_phase <= phases.size()) {
                    target_x = phases[current_phase-1].center_x + formation[i].first;
                    target_y = phases[current_phase-1].center_y + formation[i].second;
                }
                
                double error_x = target_x - drones[i].position_x;
                double error_y = target_y - drones[i].position_y;
                
                csv_file << "," << target_x << "," << drones[i].position_x << "," << error_x
                         << "," << target_y << "," << drones[i].position_y << "," << error_y
                         << "," << wind_model.wind_x << "," << wind_model.wind_y;
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
            std::cout << ", Wind: (" << std::setprecision(1) 
                      << wind_model.wind_x << "," << wind_model.wind_y << ")" << std::endl;
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
    std::cout << "\nNote: This test demonstrates PID controller performance under varying wind disturbances." << std::endl;
    std::cout << "Compare the metrics across phases to see the impact of wind on system performance." << std::endl;
    
    return 0;
} 