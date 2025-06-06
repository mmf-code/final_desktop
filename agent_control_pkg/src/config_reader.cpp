#include "../include/agent_control_pkg/config_reader.hpp"
#include <stdexcept>
#include <cmath>

namespace agent_control_pkg {

SimulationConfig ConfigReader::loadConfig(const std::string& pid_config_path, 
                                        const std::string& sim_config_path) {
    SimulationConfig config;
    
    try {
        YAML::Node pid_yaml = YAML::LoadFile(pid_config_path);
        YAML::Node sim_yaml = YAML::LoadFile(sim_config_path);
        
        loadPIDParams(config, pid_yaml);
        loadSimulationParams(config, sim_yaml);
        
    } catch (const YAML::Exception& e) {
        throw std::runtime_error("Error loading configuration: " + std::string(e.what()));
    }
    
    return config;
}

void ConfigReader::loadPIDParams(SimulationConfig& config, const YAML::Node& pid_yaml) {
    config.kp = pid_yaml["kp"].as<double>();
    config.ki = pid_yaml["ki"].as<double>();
    config.kd = pid_yaml["kd"].as<double>();
    config.output_min = pid_yaml["output_limits"]["min"].as<double>();
    config.output_max = pid_yaml["output_limits"]["max"].as<double>();
}

void ConfigReader::loadSimulationParams(SimulationConfig& config, const YAML::Node& sim_yaml) {
    // Load simulation settings
    config.dt = sim_yaml["simulation"]["dt"].as<double>();
    config.total_time = sim_yaml["simulation"]["total_time"].as<double>();
    config.num_drones = sim_yaml["simulation"]["num_drones"].as<int>();
    if (sim_yaml["fls_enabled"]) {
        config.fls_enabled = sim_yaml["fls_enabled"].as<bool>();
    } else {
        config.fls_enabled = false;
    }

    // Load formation settings
    config.formation_side_length = sim_yaml["formation"]["side_length"].as<double>();
    
    // Load initial positions
    config.initial_positions.clear();
    auto initial_pos = sim_yaml["formation"]["initial_positions"];
    for (int i = 0; i < config.num_drones; ++i) {
        std::string drone_key = "drone_" + std::to_string(i);
        auto pos = initial_pos[drone_key].as<std::vector<double>>();
        config.initial_positions.push_back({pos[0], pos[1]});
    }

    // Load phases
    config.phases.clear();
    for (const auto& phase : sim_yaml["phases"]) {
        PhaseConfig phase_config;
        phase_config.center = phase["center"].as<std::vector<double>>();
        phase_config.start_time = phase["start_time"].as<double>();
        config.phases.push_back(phase_config);
    }

    // Load wind and controller settings
    config.wind_enabled = sim_yaml["wind"]["enabled"].as<bool>();
    if (sim_yaml["controller"] && sim_yaml["controller"].IsMap()) {
        config.fls_enabled = sim_yaml["controller"]["fls_enabled"].as<bool>();
    } else {
        config.fls_enabled = false;
    }

    // Load detailed wind configuration
    loadWindConfig(config, sim_yaml["wind"]);

    // Load output settings
    config.csv_enabled = sim_yaml["output"]["csv_enabled"].as<bool>();
    config.csv_prefix = sim_yaml["output"]["csv_prefix"].as<std::string>();
    config.console_output_enabled = sim_yaml["output"]["console_output"]["enabled"].as<bool>();
    config.console_update_interval = sim_yaml["output"]["console_output"]["update_interval"].as<double>();

    // Load fuzzy logic settings with defaults
    config.fls_enabled = false;
    config.fuzzy_params_file = "fuzzy_params.yaml";
    if (sim_yaml["fuzzy_logic"]) {
        if (sim_yaml["fuzzy_logic"]["enabled"]) {
            config.fls_enabled = sim_yaml["fuzzy_logic"]["enabled"].as<bool>();
        }
        if (sim_yaml["fuzzy_logic"]["params_file"]) {
            config.fuzzy_params_file = sim_yaml["fuzzy_logic"]["params_file"].as<std::string>();
        }
    }
}

void ConfigReader::loadWindConfig(SimulationConfig& config, const YAML::Node& wind_node) {
    config.wind_phases.clear();
    
    if (!config.wind_enabled) return;

    for (const auto& phase : wind_node["phases"]) {
        WindPhase wind_phase;
        wind_phase.phase_number = phase["phase"].as<int>();
        
        for (const auto& window : phase["time_windows"]) {
            WindTimeWindow time_window;
            time_window.start_time = window["start"].as<double>();
            time_window.end_time = window["end"].as<double>();
            
            YAML::Node force_node = window["force"];
            time_window.is_sine_wave = false;

            if (force_node.IsSequence() && force_node.size() == 2 &&
                force_node[0].IsScalar() &&
                force_node[0].as<std::string>() == "sin") {
                time_window.is_sine_wave = true;
                double offset = 0.0;
                try {
                    offset = force_node[1].as<double>();
                } catch (const YAML::Exception&) {
                    offset = 0.0;
                }
                time_window.force = {1.0, offset}; // Amplitude and offset for sine wave
            } else {
                std::vector<double> force;
                for (const auto& val : force_node) {
                    force.push_back(val.as<double>());
                }
                time_window.force = force;
            }
            
            wind_phase.time_windows.push_back(time_window);
        }
        
        config.wind_phases.push_back(wind_phase);
    }
}

} // namespace agent_control_pkg 