#include "../include/agent_control_pkg/config_reader.hpp"
#include <iostream>
#include <fstream>
#include <sstream>
#include <filesystem>

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
        "../../../agent_control_pkg/config", // For running from 'build/Debug', finds 'agent_control_pkg/config'
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
            return std::filesystem::weakly_canonical(full_path).string(); 
        }
    }
    
    std::cerr << "WARNING: Config file '" << filename << "' not found in common candidate paths. Trying original path as a last resort." << std::endl;
    return filename; // Fallback
}

SimulationConfig ConfigReader::loadConfig(const std::string& config_filepath) {
    SimulationConfig config; // Initialize with defaults from the header
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

    // Load each section, calling the respective private helper method
    if (root_node["simulation_settings"]) { loadSimulationSettings(root_node["simulation_settings"], config); }
    if (root_node["controller_settings"]) { loadControllerSettings(root_node["controller_settings"], config); }
    if (root_node["scenario_settings"]) { loadScenarioSettings(root_node["scenario_settings"], config); }
    if (root_node["output_settings"]) { loadOutputSettings(root_node["output_settings"], config); }
    
    return config;
}

void ConfigReader::loadSimulationSettings(const YAML::Node& node, SimulationConfig& config) {
    if (!node.IsMap()) {
        std::cerr << "Warning: 'simulation_settings' node is not a map. Skipping." << std::endl;
        return;
    }
    config.dt = node["dt"].as<double>(config.dt);
    config.total_time = node["total_time"].as<double>(config.total_time);
    config.num_drones = node["num_drones"].as<int>(config.num_drones);

    if (node["ziegler_nichols_tuning"]) {
        const YAML::Node& zn_node = node["ziegler_nichols_tuning"];
        config.zn_tuning_params.enable = zn_node["enable"].as<bool>(config.zn_tuning_params.enable);
        config.zn_tuning_params.kp_test_value = zn_node["kp_test_value"].as<double>(config.zn_tuning_params.kp_test_value);
        config.zn_tuning_params.simulation_time = zn_node["simulation_time"].as<double>(config.zn_tuning_params.simulation_time);
        
        // Load auto-search parameters
        config.zn_tuning_params.enable_auto_search = zn_node["enable_auto_search"].as<bool>(config.zn_tuning_params.enable_auto_search);
        config.zn_tuning_params.auto_search_kp_start = zn_node["auto_search_kp_start"].as<double>(config.zn_tuning_params.auto_search_kp_start);
        config.zn_tuning_params.auto_search_kp_step = zn_node["auto_search_kp_step"].as<double>(config.zn_tuning_params.auto_search_kp_step);
        config.zn_tuning_params.auto_search_kp_max = zn_node["auto_search_kp_max"].as<double>(config.zn_tuning_params.auto_search_kp_max);
        
        // Load quick method testing parameters
        config.zn_tuning_params.quick_method_test = zn_node["quick_method_test"].as<std::string>(config.zn_tuning_params.quick_method_test);
        config.zn_tuning_params.manual_ku = zn_node["manual_ku"].as<double>(config.zn_tuning_params.manual_ku);
        config.zn_tuning_params.manual_pu = zn_node["manual_pu"].as<double>(config.zn_tuning_params.manual_pu);
    }
}

void ConfigReader::loadControllerSettings(const YAML::Node& node, SimulationConfig& config) {
     if (!node.IsMap()) {
        std::cerr << "Warning: 'controller_settings' node is not a map. Skipping." << std::endl;
        return;
    }
    if (node["pid"]) {
        const YAML::Node& pid_node = node["pid"];
        config.pid_params.kp = pid_node["kp"].as<double>(config.pid_params.kp);
        config.pid_params.ki = pid_node["ki"].as<double>(config.pid_params.ki);
        config.pid_params.kd = pid_node["kd"].as<double>(config.pid_params.kd);
        config.pid_params.output_min = pid_node["output_min"].as<double>(config.pid_params.output_min);
        config.pid_params.output_max = pid_node["output_max"].as<double>(config.pid_params.output_max);
    }
    if (node["fls"]) {
        const YAML::Node& fls_node = node["fls"];
        config.enable_fls = fls_node["enable"].as<bool>(config.enable_fls);
        config.fuzzy_params_file = fls_node["params_file"].as<std::string>(config.fuzzy_params_file);
    }
}

void ConfigReader::loadScenarioSettings(const YAML::Node& node, SimulationConfig& config) {
    if (!node.IsMap()) {
        std::cerr << "Warning: 'scenario_settings' node is not a map. Skipping." << std::endl;
        return;
    }
    config.wind_enabled = node["enable_wind"].as<bool>(config.wind_enabled);
    config.formation_side_length = node["formation_side_length"].as<double>(config.formation_side_length);

    if (node["formation"] && node["formation"]["initial_positions"]) {
        const YAML::Node& ip_node = node["formation"]["initial_positions"];
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
                    std::cerr << "Warning: Initial position for " << drone_key << " missing/malformed. Adding default (0,0)." << std::endl;
                    config.initial_positions.push_back({0.0, 0.0});
                }
            }
            while(config.initial_positions.size() < (size_t)config.num_drones) {
                 config.initial_positions.push_back({0.0,0.0});
            }
        }
    }
    
    if (config.initial_positions.size() != (size_t)config.num_drones) {
        std::cout << "INFO: Initial positions count mismatch or load error. Generating default staggered positions." << std::endl;
        config.initial_positions.clear();
        for(int i = 0; i < config.num_drones; ++i) {
            double x_pos = (static_cast<double>(i) * 2.0) - (static_cast<double>(config.num_drones - 1) * 1.0);
            config.initial_positions.push_back({x_pos, 0.0});
        }
    }

    if (node["phases"] && node["phases"].IsSequence()) {
        config.phases.clear();
        for (const auto& phase_node : node["phases"]) {
            PhaseConfig pc;
            pc.center = phase_node["center"].as<std::vector<double>>(std::vector<double>{0.0, 0.0});
            pc.start_time = phase_node["start_time"].as<double>(0.0);
            config.phases.push_back(pc);
        }
    }

    if (config.wind_enabled && node["wind"] && node["wind"]["phases"] && node["wind"]["phases"].IsSequence()) {
        config.wind_phases.clear();
        for (const auto& wp_node : node["wind"]["phases"]) {
            WindPhaseConfig wpc;
            wpc.phase_number = wp_node["phase"].as<int>(0);
            
            if (wp_node["time_windows"] && wp_node["time_windows"].IsSequence()) {
                for (const auto& tw_node : wp_node["time_windows"]) {
                    TimeWindow tw;
                    tw.start_time = tw_node["start_time"].as<double>(0.0);
                    tw.end_time = tw_node["end_time"].as<double>(0.0);
                    tw.force = tw_node["force"].as<std::vector<double>>(std::vector<double>{0.0, 0.0});
                    tw.is_sine_wave = tw_node["is_sine_wave"].as<bool>(false);
                    tw.sine_frequency_rad_s = tw_node["sine_frequency_rad_s"].as<double>(1.0);
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
    config.output_directory = node["output_directory"].as<std::string>(config.output_directory);
    config.csv_enabled = node["csv_enabled"].as<bool>(config.csv_enabled);
    config.csv_prefix = node["csv_prefix"].as<std::string>(config.csv_prefix);
    config.metrics_enabled = node["metrics_enabled"].as<bool>(config.metrics_enabled);
    config.metrics_prefix = node["metrics_prefix"].as<std::string>(config.metrics_prefix);

    if (node["console_output"]) {
        const YAML::Node& co_node = node["console_output"];
        config.console_output_enabled = co_node["enabled"].as<bool>(config.console_output_enabled);
        config.console_update_interval = co_node["update_interval"].as<double>(config.console_update_interval);
    }
}

bool loadFuzzyParamsYAML(const std::string& file_path, FuzzyParams& fp) {
    std::string actual_filepath = findConfigFilePath(file_path);
    std::ifstream in(actual_filepath);
    if (!in.is_open()) {
        std::cerr << "Error: Could not open fuzzy params file: " << actual_filepath << std::endl;
        return false;
    }
    std::string line;
    std::string section;
    std::string current_var;
    fp.sets.clear();
    fp.rules.clear();

    while (std::getline(in, line)) {
        line = trim_fuzzy_cfg(line);
        if (line.empty() || line[0] == '#') continue;
        if (line == "membership_functions:") { section = "mf"; current_var = ""; continue; }
        if (line == "rules:") { section = "rules"; current_var = ""; continue; }

        if (section == "mf") {
            if (line.length() > 1 && line.back() == ':' && line.find_first_of(" \t") == std::string::npos) {
                current_var = trim_fuzzy_cfg(line.substr(0, line.size() - 1));
                fp.sets[current_var];
                continue;
            }
            if (current_var.empty()) {
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
            if (line.rfind("- [", 0) == 0 && line.back() == ']') {
                auto tokens = parseStringList_fuzzy_cfg(line.substr(3, line.size() - 4));
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