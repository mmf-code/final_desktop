#ifndef CONFIG_READER_HPP
#define CONFIG_READER_HPP

#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>

namespace agent_control_pkg {

struct WindTimeWindow {
    double start_time;
    double end_time;
    std::vector<double> force;
    bool is_sine_wave;
};

struct WindPhase {
    int phase_number;
    std::vector<WindTimeWindow> time_windows;
};

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

    // Formation Settings
    double formation_side_length;
    std::vector<std::pair<double, double>> initial_positions;

    // Phase Settings
    std::vector<PhaseConfig> phases;

    // Wind Settings
    bool wind_enabled;
    std::vector<WindPhase> wind_phases;

    // Controller Settings
    bool fls_enabled;

    // Output Settings
    bool csv_enabled;
    std::string csv_prefix;
    bool console_output_enabled;
    double console_update_interval;

    // Fuzzy Logic Settings
    bool fls_enabled{false};
    std::string fuzzy_params_file{"fuzzy_params.yaml"};
};

class ConfigReader {
public:
    static SimulationConfig loadConfig(const std::string& pid_config_path, 
                                     const std::string& sim_config_path);

private:
    static void loadPIDParams(SimulationConfig& config, const YAML::Node& pid_yaml);
    static void loadSimulationParams(SimulationConfig& config, const YAML::Node& sim_yaml);
    static void loadWindConfig(SimulationConfig& config, const YAML::Node& wind_node);
};

} // namespace agent_control_pkg

#endif // CONFIG_READER_HPP 