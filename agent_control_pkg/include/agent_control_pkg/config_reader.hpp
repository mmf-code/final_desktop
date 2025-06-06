#ifndef CONFIG_READER_HPP
#define CONFIG_READER_HPP

#include <string>
#include <vector>
#include <map>    // For FuzzyParams
#include <array>  // For FuzzyParams
#include "yaml-cpp/yaml.h" // YAML library

namespace agent_control_pkg {

// Structure for PID parameters, nested within SimulationConfig
struct SimPIDParams {
    double kp{3.1}; // Default values
    double ki{0.4};
    double kd{2.2};
    double output_min{-10.0};
    double output_max{10.0};
};

// Structure for individual phase configuration
struct SimPhase { 
    std::vector<double> center; // {x, y}
    double start_time;
};

// Structure for wind time windows
struct WindTimeWindow {
    double start_time;
    double end_time;
    std::vector<double> force; // {force_x, force_y_or_offset_for_sine}
    bool is_sine_wave;
};

// Structure for wind configuration per phase
struct WindPhase {
    int phase_number; // 1-based phase number this wind config applies to
    std::vector<WindTimeWindow> time_windows;
};

// Main simulation configuration structure
struct SimulationConfig {
    // Simulation Settings
    double dt{0.05};
    double total_time{120.0};
    int num_drones{3};

    // Controller Settings
    SimPIDParams pid_params; // PID parameters are now a member struct
    bool enable_fls{false};
    std::string fuzzy_params_file{"fuzzy_params.yaml"}; 

    // Formation Settings
    double formation_side_length{4.0};
    std::vector<std::pair<double, double>> initial_positions; 

    // Phase Settings
    std::vector<SimPhase> phases;

    // Wind Settings
    bool wind_enabled{false};
    std::vector<WindPhase> wind_phases;

    // Output Settings
    bool csv_enabled{true}; 
    std::string csv_prefix{"multi_agent_sim"}; 
    bool console_output_enabled{true};
    double console_update_interval{5.0};
    std::string output_directory{"outputs"};
};

// Class responsible for loading configurations
class ConfigReader {
public:
    static SimulationConfig loadConfig(const std::string& primary_sim_config_path);

private:
    // Helper methods for loading different sections of the config
    static void loadSimulationSettings(SimulationConfig& config, const YAML::Node& node);
    static void loadControllerSettings(SimulationConfig& config, const YAML::Node& node);
    static void loadScenarioSettings(SimulationConfig& config, const YAML::Node& node);
    static void loadFormationSettings(SimulationConfig& config, const YAML::Node& node); // Specifically for formation.initial_positions and side_length
    static void loadPhases(SimulationConfig& config, const YAML::Node& node);
    static void loadWindConfig(SimulationConfig& config, const YAML::Node& wind_phases_node);
    static void loadOutputSettings(SimulationConfig& config, const YAML::Node& node);
};

// For Fuzzy Params loading (kept separate as per your existing structure, can be integrated into ConfigReader later if desired)
struct FuzzySetFOU { double l1, l2, l3, u1, u2, u3; }; // Already in your main
struct FuzzyParams { // Already in your main
    std::map<std::string, std::map<std::string, FuzzySetFOU>> sets;
    std::vector<std::array<std::string,4>> rules;
};
std::string findConfigFilePath(const std::string& filename); // Declaration
bool loadFuzzyParamsYAML(const std::string& file, FuzzyParams& fp); // Declaration, matches cpp

} // namespace agent_control_pkg
#endif // CONFIG_READER_HPP