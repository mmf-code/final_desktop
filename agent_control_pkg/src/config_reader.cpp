#include "../include/agent_control_pkg/config_reader.hpp"
#include <iostream>
#include <fstream>   // For std::ifstream in loadFuzzyParamsYAML
#include <sstream>   // For std::stringstream in loadFuzzyParamsYAML
#include <filesystem> // For robust path finding

namespace agent_control_pkg {

// --- Utility functions for fuzzy_params.yaml parsing ---
static std::string trim_fuzzy_cfg(const std::string& s) {
    size_t start = s.find_first_not_of(" \t\r\n");
    size_t end = s.find_last_not_of(" \t\r\n");
    if(start==std::string::npos) return "";
    return s.substr(start, end-start+1);
}

static std::vector<double> parseNumberList_fuzzy_cfg(const std::string& in) {
    std::vector<double> values;
    std::stringstream ss(in);
    std::string tok;
    while(std::getline(ss, tok, ',')) {
        tok = trim_fuzzy_cfg(tok);
        if(!tok.empty()) {
            try {
                values.push_back(std::stod(tok));
            } catch (const std::invalid_argument& ia) {
                std::cerr << "Warning: Invalid number '" << tok << "' in list. Skipping." << std::endl;
            } catch (const std::out_of_range& oor) {
                std::cerr << "Warning: Number '" << tok << "' out of range. Skipping." << std::endl;
            }
        }
    }
    return values;
}

static std::vector<std::string> parseStringList_fuzzy_cfg(const std::string& in) {
    std::vector<std::string> values;
    std::stringstream ss(in);
    std::string tok;
    while(std::getline(ss, tok, ',')) {
        values.push_back(trim_fuzzy_cfg(tok));
    }
    return values;
}
// --- End Utility functions for fuzzy_params.yaml ---


std::string findConfigFilePath(const std::string& filename) {
    // This list of candidate directories is searched in order.
    std::vector<std::filesystem::path> candidate_dirs = {
        ".", // Current working directory (e.g., build/Debug)
        "config", // A 'config' folder inside the CWD
        "../config", // For builds in a subdir like 'build', this finds a 'config' folder next to it
        
        // --- THIS IS THE KEY PATH TO ADD ---
        // For running from a deep build folder like 'build/Debug', 
        // this finds 'agent_control_pkg/config'
        "../../../agent_control_pkg/config", 

        // Original paths for completeness
        "../../config", 
        "../../agent_control_pkg/config" 
    };

    // Also check if filename is an absolute path
    std::filesystem::path direct_path(filename);
    if (direct_path.is_absolute() && std::filesystem::exists(direct_path) && std::filesystem::is_regular_file(direct_path)) {
        return direct_path.string();
    }

    for (const auto& dir : candidate_dirs) {
        std::filesystem::path full_path = dir / filename;
        if (std::filesystem::exists(full_path) && std::filesystem::is_regular_file(full_path)) {
            // Normalize the path for consistent output and to resolve ".."
            return std::filesystem::weakly_canonical(full_path).string(); 
        }
    }
    
    // If we get here, the file was not found in any common location.
    std::cerr << "WARNING: Config file '" << filename << "' not found in common candidate paths. Trying original path as a last resort." << std::endl;
    return filename; // Fallback to the original filename
}


SimulationConfig ConfigReader::loadConfig(const std::string& config_filepath) {
    SimulationConfig config; // Initialize with defaults
    std::string actual_filepath = findConfigFilePath(config_filepath);

    YAML::Node root_node;
    try {
        root_node = YAML::LoadFile(actual_filepath);
        std::cout << "Successfully loaded configuration from: " << actual_filepath << std::endl;
    } catch (const YAML::Exception& e) {
        std::cerr << "FATAL: Error loading YAML file '" << actual_filepath << "': " << e.what() << std::endl;
        std::cerr << "Using default configuration values for everything." << std::endl;
        return config; // Return default config
    }

    // Load each section, providing the relevant sub-node
    if (root_node["simulation_settings"]) {
        loadSimulationSettings(root_node["simulation_settings"], config);
    } else {
        std::cerr << "Warning: 'simulation_settings' not found in " << actual_filepath << ". Using defaults for this section." << std::endl;
    }

    if (root_node["controller_settings"]) {
        loadControllerSettings(root_node["controller_settings"], config);
    } else {
        std::cerr << "Warning: 'controller_settings' not found in " << actual_filepath << ". Using defaults for this section." << std::endl;
    }

    if (root_node["scenario_settings"]) {
        loadScenarioSettings(root_node["scenario_settings"], config);
    } else {
        std::cerr << "Warning: 'scenario_settings' not found in " << actual_filepath << ". Using defaults for this section." << std::endl;
    }
    
    if (root_node["output_settings"]) {
        loadOutputSettings(root_node["output_settings"], config);
    } else {
        std::cerr << "Warning: 'output_settings' not found in " << actual_filepath << ". Using defaults for this section." << std::endl;
    }
    
    return config;
}

void ConfigReader::loadSimulationSettings(const YAML::Node& node, SimulationConfig& config) {
    if (!node.IsMap()) {
        std::cerr << "Warning: 'simulation_settings' node is not a map. Skipping." << std::endl;
        return;
    }
    if (node["dt"]) config.dt = node["dt"].as<double>(config.dt);
    if (node["total_time"]) config.total_time = node["total_time"].as<double>(config.total_time);
    if (node["num_drones"]) config.num_drones = node["num_drones"].as<int>(config.num_drones);

    if (node["ziegler_nichols_tuning"]) {
        YAML::Node zn_node = node["ziegler_nichols_tuning"];
        if (zn_node["enable"]) config.zn_tuning_params.enable = zn_node["enable"].as<bool>(config.zn_tuning_params.enable);
        if (zn_node["kp_test_value"]) config.zn_tuning_params.kp_test_value = zn_node["kp_test_value"].as<double>(config.zn_tuning_params.kp_test_value);
        if (zn_node["simulation_time"]) config.zn_tuning_params.simulation_time = zn_node["simulation_time"].as<double>(config.zn_tuning_params.simulation_time);
    }
}

void ConfigReader::loadControllerSettings(const YAML::Node& node, SimulationConfig& config) {
     if (!node.IsMap()) {
        std::cerr << "Warning: 'controller_settings' node is not a map. Skipping." << std::endl;
        return;
    }
    if (node["pid"]) {
        YAML::Node pid_node = node["pid"];
        if (pid_node["kp"]) config.pid_params.kp = pid_node["kp"].as<double>(config.pid_params.kp);
        if (pid_node["ki"]) config.pid_params.ki = pid_node["ki"].as<double>(config.pid_params.ki);
        if (pid_node["kd"]) config.pid_params.kd = pid_node["kd"].as<double>(config.pid_params.kd);
        if (pid_node["output_min"]) config.pid_params.output_min = pid_node["output_min"].as<double>(config.pid_params.output_min);
        if (pid_node["output_max"]) config.pid_params.output_max = pid_node["output_max"].as<double>(config.pid_params.output_max);
    }
    if (node["fls"]) {
        YAML::Node fls_node = node["fls"];
        if (fls_node["enable"]) config.enable_fls = fls_node["enable"].as<bool>(config.enable_fls);
        if (fls_node["params_file"]) config.fuzzy_params_file = fls_node["params_file"].as<std::string>(config.fuzzy_params_file);
    }
}

void ConfigReader::loadScenarioSettings(const YAML::Node& node, SimulationConfig& config) {
    if (!node.IsMap()) {
        std::cerr << "Warning: 'scenario_settings' node is not a map. Skipping." << std::endl;
        return;
    }
    if (node["enable_wind"]) config.wind_enabled = node["enable_wind"].as<bool>(config.wind_enabled);
    
    if (node["formation_side_length"]) config.formation_side_length = node["formation_side_length"].as<double>(config.formation_side_length);

    if (node["formation"] && node["formation"]["initial_positions"]) {
        YAML::Node ip_node = node["formation"]["initial_positions"];
        if (ip_node.IsMap()) {
            config.initial_positions.clear(); 
            for (int i = 0; i < config.num_drones; ++i) {
                std::string drone_key = "drone_" + std::to_string(i);
                if (ip_node[drone_key] && ip_node[drone_key].IsSequence() && ip_node[drone_key].size() == 2) {
                    config.initial_positions.push_back({
                        ip_node[drone_key][0].as<double>(),
                        ip_node[drone_key][1].as<double>()
                    });
                } else {
                    std::cerr << "Warning: Initial position for " << drone_key << " missing/malformed. Using default (0,0) for it." << std::endl;
                    config.initial_positions.push_back({0.0, 0.0}); // Add a default if specific one fails
                }
            }
            // Ensure correct number of positions, even if some failed to load above
            while(config.initial_positions.size() < (size_t)config.num_drones) {
                 config.initial_positions.push_back({0.0,0.0});
            }
        } else {
            std::cerr << "Warning: 'formation.initial_positions' is not a map. Using default positions." << std::endl;
        }
    } else {
         std::cerr << "Warning: 'formation.initial_positions' not found. Using default positions." << std::endl;
    }
    
    // Fallback if loading failed entirely or not enough positions for num_drones
    if (config.initial_positions.size() != (size_t)config.num_drones && config.num_drones > 0) {
        std::cout << "INFO: Initial positions count mismatch or load error. Generating default positions." << std::endl;
        config.initial_positions.clear();
        for(int i = 0; i < config.num_drones; ++i) {
            // A simple default: spread them out on X axis
            config.initial_positions.push_back({static_cast<double>(i) * 2.0 - (static_cast<double>(config.num_drones - 1) * 1.0), 0.0});
        }
    }


    if (node["phases"] && node["phases"].IsSequence()) {
        config.phases.clear();
        for (const auto& phase_node : node["phases"]) {
            PhaseConfig pc;
            if (phase_node["center"] && phase_node["center"].IsSequence() && phase_node["center"].size() == 2) {
                pc.center = phase_node["center"].as<std::vector<double>>();
            } else {
                 std::cerr << "Warning: Phase 'center' malformed. Using [0,0]." << std::endl;
                 pc.center = {0.0, 0.0};
            }
            if (phase_node["start_time"]) {
                pc.start_time = phase_node["start_time"].as<double>();
            } else {
                pc.start_time = 0.0; // Default if not specified
            }
            config.phases.push_back(pc);
        }
    }

    if (config.wind_enabled && node["wind"] && node["wind"]["phases"] && node["wind"]["phases"].IsSequence()) {
        config.wind_phases.clear();
        for (const auto& wp_node : node["wind"]["phases"]) {
            WindPhaseConfig wpc;
            if (wp_node["phase"]) wpc.phase_number = wp_node["phase"].as<int>();
            
            if (wp_node["time_windows"] && wp_node["time_windows"].IsSequence()) {
                for (const auto& tw_node : wp_node["time_windows"]) {
                    TimeWindow tw;
                    if (tw_node["start_time"]) tw.start_time = tw_node["start_time"].as<double>(); else tw.start_time = 0.0;
                    if (tw_node["end_time"]) tw.end_time = tw_node["end_time"].as<double>(); else tw.end_time = 0.0;
                    
                    if (tw_node["force"] && tw_node["force"].IsSequence() && tw_node["force"].size() == 2) {
                        tw.force = tw_node["force"].as<std::vector<double>>();
                    } else {
                         std::cerr << "Warning: Wind time_window 'force' malformed or missing. Using [0,0]." << std::endl;
                        tw.force = {0.0, 0.0};
                    }
                    if (tw_node["is_sine_wave"]) tw.is_sine_wave = tw_node["is_sine_wave"].as<bool>(false);
                    if (tw_node["sine_frequency_rad_s"]) tw.sine_frequency_rad_s = tw_node["sine_frequency_rad_s"].as<double>(1.0);
                    wpc.time_windows.push_back(tw);
                }
            }
            config.wind_phases.push_back(wpc);
        }
    }
}


void ConfigReader::loadOutputSettings(const YAML::Node& node, SimulationConfig& config) {
    if (!node.IsMap()) {
        std::cerr << "Warning: 'output_settings' node is not a map. Skipping." << std::endl;
        return;
    }
    if (node["output_directory"]) config.output_directory = node["output_directory"].as<std::string>(config.output_directory);
    if (node["csv_enabled"]) config.csv_enabled = node["csv_enabled"].as<bool>(config.csv_enabled);
    if (node["csv_prefix"]) config.csv_prefix = node["csv_prefix"].as<std::string>(config.csv_prefix);
    if (node["metrics_enabled"]) config.metrics_enabled = node["metrics_enabled"].as<bool>(config.metrics_enabled);
    if (node["metrics_prefix"]) config.metrics_prefix = node["metrics_prefix"].as<std::string>(config.metrics_prefix);

    if (node["console_output"]) {
        YAML::Node co_node = node["console_output"];
        if (co_node["enabled"]) config.console_output_enabled = co_node["enabled"].as<bool>(config.console_output_enabled);
        if (co_node["update_interval"]) config.console_update_interval = co_node["update_interval"].as<double>(config.console_update_interval);
    }
}

// Implementation for loading fuzzy parameters (matches your multi_drone_pid_test_main.cpp style)
bool loadFuzzyParamsYAML(const std::string& file_path, FuzzyParams& fp) {
    std::string actual_filepath = findConfigFilePath(file_path); // Use the same path finding logic
    std::ifstream in(actual_filepath);
    if (!in.is_open()) {
        std::cerr << "Error: Could not open fuzzy params file: " << actual_filepath << std::endl;
        return false;
    }
    std::string line;
    std::string section;
    std::string current_var;
    fp.sets.clear(); // Clear previous data
    fp.rules.clear();

    while (std::getline(in, line)) {
        line = trim_fuzzy_cfg(line);
        if (line.empty() || line[0] == '#') continue;
        if (line == "membership_functions:") { section = "mf"; current_var = ""; continue; }
        if (line == "rules:") { section = "rules"; current_var = ""; continue; }

        if (section == "mf") {
            if (line.length() > 1 && line.back() == ':' && line.find_first_of(" \t") == std::string::npos) {
                current_var = trim_fuzzy_cfg(line.substr(0, line.size() - 1));
                fp.sets[current_var]; // Ensure the map entry for current_var exists
                continue;
            }
            if (current_var.empty()) {
                 // std::cerr << "Warning: Fuzzy MF line encountered outside a variable block: " << line << std::endl;
                continue;
            }

            auto pos = line.find(':');
            if (pos == std::string::npos) continue;
            std::string setname = trim_fuzzy_cfg(line.substr(0, pos));
            std::string rest = line.substr(pos + 1);
            auto lb = rest.find('[');
            auto rb = rest.find(']');
            if (lb == std::string::npos || rb == std::string::npos) continue;
            std::string nums_str = rest.substr(lb + 1, rb - lb - 1);
            auto values = parseNumberList_fuzzy_cfg(nums_str);
            if (values.size() == 6) {
                fp.sets[current_var][setname] = {values[0], values[1], values[2], values[3], values[4], values[5]};
            } else {
                std::cerr << "Warning: Fuzzy set '" << setname << "' for var '" << current_var << "' has incorrect number of params. Expected 6, got " << values.size() << std::endl;
            }
        } else if (section == "rules") {
            if (line.rfind("- [", 0) == 0 && line.back() == ']') { // More robust rule parsing
                auto tokens = parseStringList_fuzzy_cfg(line.substr(3, line.size() - 4)); // Extract content between "- [" and "]"
                if (tokens.size() == 4) {
                    fp.rules.push_back({tokens[0], tokens[1], tokens[2], tokens[3]});
                } else {
                     std::cerr << "Warning: Fuzzy rule has incorrect number of tokens. Expected 4, got " << tokens.size() << " in: " << line << std::endl;
                }
            }
        }
    }
    return true;
}

} // namespace agent_control_pkg