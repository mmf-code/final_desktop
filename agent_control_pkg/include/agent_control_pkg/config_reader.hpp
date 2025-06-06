#ifndef CONFIG_READER_HPP
#define CONFIG_READER_HPP

#include <string>
#include <vector>
<<<<<<< HEAD
#include <map>    // For FuzzyParams
#include <array>  // For FuzzyParams
#include <yaml-cpp/yaml.h> // For YAML parsing

namespace agent_control_pkg {

// Structure for PID parameters, nested within SimulationConfig
struct SimPIDParams {
    double kp{3.1}; // Default values
    double ki{0.4};
    double kd{2.2};
    double output_min{-10.0};
    double output_max{10.0};
};

// Structure for auto Kp tuning parameters
struct AutoKpTuningParams {
    bool enable{false};
    double kp_start{0.5};
    double kp_step{0.5};
    double kp_max{10.0};
    double simulation_time{30.0};
};

// Structure for individual phase configuration
<<<<<<< HEAD
struct PhaseConfig {
    std::vector<double> center;
    double start_time{0.0};
};

// Structure for wind time windows
struct TimeWindow {
    double start_time{0.0};
    double end_time{0.0};
    std::vector<double> force;
    bool is_sine_wave{false};
    double sine_frequency_rad_s{1.0};
};

// Structure for wind configuration per phase
struct WindPhaseConfig {
    int phase_number{0};
    std::vector<TimeWindow> time_windows;
};

// Structure for Ziegler-Nichols tuning parameters
struct ZNTuningParams {
    bool enable{false};
    double kp_test_value{1.0};
    double simulation_time{30.0};
    bool enable_auto_search{false};
    double auto_search_kp_start{0.5};
    double auto_search_kp_step{0.5};
    double auto_search_kp_max{10.0};
=======
struct SimPhase { 
    std::vector<double> center; // {x, y}
    double start_time;
};

// Structure for wind time windows
=======
#include <yaml-cpp/yaml.h>

namespace agent_control_pkg {

>>>>>>> parent of b1aaa38 (itworks)
struct WindTimeWindow {
    double start_time;
    double end_time;
    std::vector<double> force;
    bool is_sine_wave;
};

struct WindPhase {
    int phase_number;
    std::vector<WindTimeWindow> time_windows;
>>>>>>> parent of 2a992f0 (zn_again)
};

<<<<<<< HEAD
// Main simulation configuration structure
struct SimulationConfig {
    // Simulation Settings
    double dt{0.05};
    double total_time{120.0};
    int num_drones{3};
<<<<<<< HEAD
    ZNTuningParams zn_tuning_params;
    AutoKpTuningParams auto_kp_tuning_params;
=======
>>>>>>> parent of 2a992f0 (zn_again)

    // Controller Settings
    SimPIDParams pid_params; // PID parameters are now a member struct
    bool enable_fls{false};
    std::string fuzzy_params_file{"fuzzy_params.yaml"};
=======
struct PhaseConfig {
    std::vector<double> center;
    double start_time;
};

struct SimulationConfig {
    // PID Parameters
    double kp;
    double ki;
    double kd;
    double output_min;
    double output_max;

    // Simulation Settings
    double dt;
    double total_time;
    int num_drones;
>>>>>>> parent of b1aaa38 (itworks)

<<<<<<< HEAD
    // Scenario Settings
    bool wind_enabled{false};
    double formation_side_length{4.0};
    std::vector<std::pair<double, double>> initial_positions;
    std::vector<PhaseConfig> phases;
    std::vector<WindPhaseConfig> wind_phases;

    // Output Settings
    std::string output_directory{"simulation_outputs"};
    bool csv_enabled{true};
    std::string csv_prefix{"multi_agent_sim"};
    bool metrics_enabled{true};
    std::string metrics_prefix{"metrics_sim"};
=======
    // Formation Settings
    double formation_side_length;
    std::vector<std::pair<double, double>> initial_positions;

    // Phase Settings
    std::vector<PhaseConfig> phases;

    // Wind Settings
    bool wind_enabled;
    std::vector<WindPhase> wind_phases;
  
// Inside struct SimulationConfig:
// ...
// Controller Settings
SimPIDParams pid_params;
bool enable_fls{false}; // THIS IS THE ONE TO KEEP
// std::string fuzzy_params_filename{"fuzzy_params.yaml"}; // Add this if you want to configure the FLS param file name
// ...

    // Output Settings
<<<<<<< HEAD
    bool csv_enabled{true}; 
    std::string csv_prefix{"multi_agent_sim"}; 
>>>>>>> parent of 2a992f0 (zn_again)
    bool console_output_enabled{true};
    double console_update_interval{5.0};
    std::string output_directory{"outputs"};
=======
    bool csv_enabled;
    std::string csv_prefix;
    bool console_output_enabled;
    double console_update_interval;
>>>>>>> parent of b1aaa38 (itworks)
};

class ConfigReader {
public:
    static SimulationConfig loadConfig(const std::string& pid_config_path, 
                                     const std::string& sim_config_path);

private:
<<<<<<< HEAD
<<<<<<< HEAD
    static void loadSimulationSettings(const YAML::Node& node, SimulationConfig& config);
    static void loadControllerSettings(const YAML::Node& node, SimulationConfig& config);
    static void loadScenarioSettings(const YAML::Node& node, SimulationConfig& config);
    static void loadOutputSettings(const YAML::Node& node, SimulationConfig& config);
};

// For Fuzzy Params loading
struct FuzzySetFOU { double l1, l2, l3, u1, u2, u3; };
struct FuzzyParams {
    std::map<std::string, std::map<std::string, FuzzySetFOU>> sets;
    std::vector<std::array<std::string,4>> rules;
};

// Utility function to find config file path
std::string findConfigFilePath(const std::string& filename);

// Declaration for loading fuzzy parameters
bool loadFuzzyParamsYAML(const std::string& file_path, FuzzyParams& fp);
=======
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
>>>>>>> parent of 2a992f0 (zn_again)

=======
    static void loadPIDParams(SimulationConfig& config, const YAML::Node& pid_yaml);
    static void loadSimulationParams(SimulationConfig& config, const YAML::Node& sim_yaml);
    static void loadWindConfig(SimulationConfig& config, const YAML::Node& wind_node);
};

>>>>>>> parent of b1aaa38 (itworks)
} // namespace agent_control_pkg

#endif // CONFIG_READER_HPP 