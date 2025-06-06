#ifndef AGENT_CONTROL_PKG__CONFIG_READER_HPP_
#define AGENT_CONTROL_PKG__CONFIG_READER_HPP_

#include <string>
#include <vector>
#include <utility> // For std::pair
#include <stdexcept> // For std::runtime_error
#include <map>    // For FuzzyParams
#include <array>  // For FuzzyParams
#include "yaml-cpp/yaml.h" // YAML library

namespace agent_control_pkg {

// Structure for PID parameters
struct SimPIDParams {
    double kp{3.1}; 
    double ki{0.4};
    double kd{2.2};
    double output_min{-10.0};
    double output_max{10.0};
};

// Structure for individual phase configuration
struct PhaseConfig { // Renamed from SimPhase for consistency
    std::vector<double> center; // {x, y}
    double start_time{0.0};
};

// Structure for wind time windows
struct TimeWindow { // Renamed from WindTimeWindow
    double start_time{0.0}; // Relative to the start of its parent simulation phase
    double end_time{0.0};   // Relative to the start of its parent simulation phase
    std::vector<double> force; // {wind_x_magnitude, wind_y_magnitude}
    bool is_sine_wave{false};
    double sine_frequency_rad_s{1.0}; // Default frequency if not specified for sine wave
};

// Structure for wind configuration per phase
struct WindPhaseConfig { // Renamed from WindPhase
    int phase_number{0}; // 1-indexed, refers to the Nth phase in SimulationConfig::phases
    std::vector<TimeWindow> time_windows;
};

// Structure for Ziegler-Nichols tuning parameters
struct ZNTuningParams {
    bool enable{false};
    double kp_test_value{1.0};
    double simulation_time{30.0};
};

// Main simulation configuration structure
struct SimulationConfig {
    // Simulation Settings
    double dt{0.05};
    double total_time{120.0};
    int num_drones{3};
    ZNTuningParams zn_tuning_params; // Z-N tuning parameters

    // Controller Settings
    SimPIDParams pid_params;
    bool enable_fls{false};
    std::string fuzzy_params_file{"fuzzy_params.yaml"}; 

    // Scenario Settings
    bool wind_enabled{false}; // Master switch for wind
    double formation_side_length{4.0};
    std::vector<std::pair<double, double>> initial_positions; 
    std::vector<PhaseConfig> phases;
    std::vector<WindPhaseConfig> wind_phases; // Wind details per phase

    // Output Settings
    std::string output_directory{"simulation_outputs"};
    bool csv_enabled{true}; 
    std::string csv_prefix{"multi_agent_sim"}; 
    bool metrics_enabled{true};
    std::string metrics_prefix{"metrics_sim"};
    bool console_output_enabled{true};
    double console_update_interval{5.0};
};

// Class responsible for loading configurations
class ConfigReader {
public:
    static SimulationConfig loadConfig(const std::string& config_filepath);

private:
    // Helper methods for loading different sections of the config
    static void loadSimulationSettings(const YAML::Node& node, SimulationConfig& config);
    static void loadControllerSettings(const YAML::Node& node, SimulationConfig& config);
    static void loadScenarioSettings(const YAML::Node& node, SimulationConfig& config);
    // Note: formation, phases, and wind are now typically loaded *within* loadScenarioSettings
    static void loadOutputSettings(const YAML::Node& node, SimulationConfig& config);
};

// For Fuzzy Params loading (defined here for completeness, matching your multi_drone_pid_test_main.cpp)
struct FuzzySetFOU { double l1, l2, l3, u1, u2, u3; };
struct FuzzyParams {
    std::map<std::string, std::map<std::string, FuzzySetFOU>> sets;
    std::vector<std::array<std::string,4>> rules;
};

// Utility function to find config file path
std::string findConfigFilePath(const std::string& filename);

// Declaration for loading fuzzy parameters (implementation will be in config_reader.cpp or fuzzy_loader.cpp)
bool loadFuzzyParamsYAML(const std::string& file_path, FuzzyParams& fp);

} // namespace agent_control_pkg
#endif // AGENT_CONTROL_PKG__CONFIG_READER_HPP_