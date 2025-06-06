#include "../include/agent_control_pkg/config_reader.hpp"
#include <stdexcept> // For std::runtime_error
#include <iostream>  // For std::cerr, std::cout
#include <fstream>   // For std::ifstream
#include <sstream>   // For std::stringstream

namespace agent_control_pkg {

// --- Utility functions (copied from your agent_control_main.cpp for now) ---
static std::string trim_cfg(const std::string& s) {
    size_t start = s.find_first_not_of(" \t\r\n");
    size_t end = s.find_last_not_of(" \t\r\n");
    if(start==std::string::npos) return "";
    return s.substr(start, end-start+1);
}

static std::vector<double> parseNumberList_cfg(const std::string& in) {
    std::vector<double> values;
    std::stringstream ss(in);
    std::string tok;
    while(std::getline(ss, tok, ',')) {
        tok = trim_cfg(tok);
        if(!tok.empty()) values.push_back(std::stod(tok));
    }
    return values;
}

static std::vector<std::string> parseStringList_cfg(const std::string& in) {
    std::vector<std::string> values;
    std::stringstream ss(in);
    std::string tok;
    while(std::getline(ss, tok, ',')) {
        values.push_back(trim_cfg(tok));
    }
    return values;
}
// --- End Utility functions ---

std::string findConfigFilePath(const std::string& filename) {
    std::vector<std::string> candidates = {
        filename,                                     // 1. Check CWD (e.g., build/Debug/simulation_params.yaml)
        "config/" + filename,                         // 2. Check CWD/config/ (e.g., build/Debug/config/simulation_params.yaml)
        "../config/" + filename,                      // 3. Check CWD/../config/ (e.g., build/config/simulation_params.yaml)
        "../../config/" + filename,                   // 4. Check CWD/../../config/ (e.g., <project_root>/config/simulation_params.yaml)
        "../../agent_control_pkg/config/" + filename  // 5. Check CWD/../../agent_control_pkg/config/
    };

    // std::cout << "Searching for config file: " << filename << std::endl; // For debugging
    for(const auto& p : candidates) {
        // std::cout << "  Checking relative path: " << p << std::endl; // For debugging
        std::ifstream f(p);
        if(f.good()) {
            // std::cout << "  SUCCESS: Found config file at: " << p << std::endl; // For debugging
            return p;
        }
    }
    std::cerr << "WARNING: Config file '" << filename << "' not found in candidate paths." << std::endl;
    return filename; // Fallback
}


SimulationConfig ConfigReader::loadConfig(const std::string& primary_sim_config_path) {
    SimulationConfig config; // Uses default values from struct definition
    std::string actual_filepath = findConfigFilePath(primary_sim_config_path);

    try {
        YAML::Node root_node = YAML::LoadFile(actual_filepath);
        
        // It's better to pass the relevant sub-node to each load function
        if (root_node["simulation_settings"]) loadSimulationSettings(config, root_node["simulation_settings"]);
        else if (root_node["simulation"]) loadSimulationSettings(config, root_node["simulation"]); // Old key from your YAML

        if (root_node["controller_settings"]) loadControllerSettings(config, root_node["controller_settings"]);
        else if (root_node["controller"]) loadControllerSettings(config, root_node["controller"]); // Old key

        if (root_node["scenario_settings"]) loadScenarioSettings(config, root_node["scenario_settings"]);
        else { // Try loading formation, phases, wind individually if scenario_settings is not top-level
            if (root_node["formation"]) loadFormationSettings(config, root_node["formation"]);
            if (root_node["phases"]) loadPhases(config, root_node["phases"]);
            if (root_node["wind"]) loadWindConfig(config, root_node["wind"]); // Load from top-level "wind" node
        }
        
        if (root_node["output_settings"]) loadOutputSettings(config, root_node["output_settings"]);
        else if (root_node["output"]) loadOutputSettings(config, root_node["output"]); // Old key
        
    } catch (const YAML::Exception& e) {
        throw std::runtime_error("Error loading configuration file '" + actual_filepath + "': " + std::string(e.what()));
    }
    return config;
}

void ConfigReader::loadSimulationSettings(SimulationConfig& config, const YAML::Node& node) {
    if (!node || !node.IsMap()) {
        std::cerr << "Warning: 'simulation_settings' or 'simulation' node not found or not a map in YAML." << std::endl;
        return;
    }
    config.dt = node["dt"].as<double>(config.dt);
    config.total_time = node["total_time"].as<double>(config.total_time);
    config.num_drones = node["num_drones"].as<int>(config.num_drones);
}

void ConfigReader::loadControllerSettings(SimulationConfig& config, const YAML::Node& node) {
    if (!node || !node.IsMap()) {
        std::cerr << "Warning: 'controller_settings' or 'controller' node not found or not a map in YAML." << std::endl;
        return;
    }
    if (node["pid"]) {
        config.pid_params.kp = node["pid"]["kp"].as<double>(config.pid_params.kp);
        config.pid_params.ki = node["pid"]["ki"].as<double>(config.pid_params.ki);
        config.pid_params.kd = node["pid"]["kd"].as<double>(config.pid_params.kd);
        config.pid_params.output_min = node["pid"]["output_min"].as<double>(config.pid_params.output_min);
        config.pid_params.output_max = node["pid"]["output_max"].as<double>(config.pid_params.output_max);
    } else { // Try loading PID from top level of controller node (compatibility)
        config.pid_params.kp = node["kp"].as<double>(config.pid_params.kp);
        config.pid_params.ki = node["ki"].as<double>(config.pid_params.ki);
        config.pid_params.kd = node["kd"].as<double>(config.pid_params.kd);
        if (node["output_limits"]){
            config.pid_params.output_min = node["output_limits"]["min"].as<double>(config.pid_params.output_min);
            config.pid_params.output_max = node["output_limits"]["max"].as<double>(config.pid_params.output_max);
        }
    }

    // FLS settings can be under controller_settings.fls or a top-level fuzzy_logic
    if (node["fls"]) {
        config.enable_fls = node["fls"]["enable"].as<bool>(config.enable_fls);
        config.fuzzy_params_file = node["fls"]["params_file"].as<std::string>(config.fuzzy_params_file);
    } else if (node["fuzzy_logic"]) { // Check alternative structure from your YAML
         config.enable_fls = node["fuzzy_logic"]["enabled"].as<bool>(config.enable_fls);
         config.fuzzy_params_file = node["fuzzy_logic"]["params_file"].as<std::string>(config.fuzzy_params_file);
    } else if (node["enable_fls"]){ // Check for direct enable_fls under controller
         config.enable_fls = node["enable_fls"].as<bool>(config.enable_fls);
    }
}

// Combined scenario settings loader
void ConfigReader::loadScenarioSettings(SimulationConfig& config, const YAML::Node& node) {
    if (!node || !node.IsMap()) {
        std::cerr << "Warning: 'scenario_settings' node not found or not a map in YAML." << std::endl;
        return;
    }
    config.wind_enabled = node["enable_wind"].as<bool>(config.wind_enabled); // Assuming enable_wind is directly under scenario_settings
    
    if (node["formation"]) loadFormationSettings(config, node["formation"]);
    if (node["phases"]) loadPhases(config, node["phases"]);
    if (config.wind_enabled && node["wind"]) loadWindConfig(config, node["wind"]["phases"]); // Assuming wind config is under wind.phases
}


void ConfigReader::loadFormationSettings(SimulationConfig& config, const YAML::Node& node) {
    if (!node || !node.IsMap()) {
        std::cerr << "Warning: 'formation' node not found or not a map in YAML." << std::endl;
        return;
    }
    config.formation_side_length = node["side_length"].as<double>(config.formation_side_length);

    if (node["initial_positions"] && node["initial_positions"].IsMap()) {
        YAML::Node initial_pos_node = node["initial_positions"];
        config.initial_positions.clear();
        bool all_loaded_successfully = true;
        for (int i = 0; i < config.num_drones; ++i) {
            std::string drone_key = "drone_" + std::to_string(i);
            if (initial_pos_node[drone_key]) {
                try {
                    auto pos_vec = initial_pos_node[drone_key].as<std::vector<double>>();
                    if (pos_vec.size() == 2) {
                        config.initial_positions.push_back({pos_vec[0], pos_vec[1]});
                    } else {
                        std::cerr << "Warning: Initial position for " << drone_key << " must have 2 elements (x, y)." << std::endl;
                        all_loaded_successfully = false; break;
                    }
                } catch (const YAML::Exception& e) {
                    std::cerr << "Warning: Error parsing initial position for " << drone_key << ": " << e.what() << std::endl;
                    all_loaded_successfully = false; break;
                }
            } else {
                std::cerr << "Warning: Initial position for " << drone_key << " not found in YAML." << std::endl;
                all_loaded_successfully = false; break;
            }
        }
        if (!all_loaded_successfully || config.initial_positions.size() != config.num_drones) {
            std::cerr << "Warning: Could not load all initial positions correctly or count mismatch. Using default positions." << std::endl;
            config.initial_positions.clear(); // Clear any partial load
        }
    } else {
        std::cerr << "Warning: 'formation.initial_positions' is not a map or not found. Using default positions." << std::endl;
    }

    // Fallback if loading failed or not enough positions
    if (config.initial_positions.size() != config.num_drones) {
        std::cout << "INFO: Using default calculated initial drone positions." << std::endl;
        config.initial_positions.clear();
        for(int i = 0; i < config.num_drones; ++i) {
            config.initial_positions.push_back({static_cast<double>(i) * 2.0 - (static_cast<double>(config.num_drones - 1) * 1.0), 0.0});
        }
    }
}

void ConfigReader::loadPhases(SimulationConfig& config, const YAML::Node& node) {
    if (!node || !node.IsSequence()) {
         std::cerr << "Warning: 'phases' node not found or not a sequence in YAML." << std::endl;
        return;
    }
    config.phases.clear();
    for (const auto& phase_node : node) {
        SimPhase p;
        p.center = phase_node["center"].as<std::vector<double>>();
        p.start_time = phase_node["start_time"].as<double>();
        config.phases.push_back(p);
    }
}

void ConfigReader::loadWindConfig(SimulationConfig& config, const YAML::Node& wind_phases_node) {
    config.wind_phases.clear();
    if (!config.wind_enabled || !wind_phases_node || !wind_phases_node.IsSequence()) {
        if (config.wind_enabled) std::cerr << "Warning: Wind enabled but 'wind.phases' node not found or not a sequence." << std::endl;
        return;
    }

    for (const auto& phase_node : wind_phases_node) {
        WindPhase wind_phase_cfg; // Use the struct from hpp
        wind_phase_cfg.phase_number = phase_node["phase"].as<int>();
        
        if (phase_node["time_windows"] && phase_node["time_windows"].IsSequence()) {
            for (const auto& window_node : phase_node["time_windows"]) {
                WindTimeWindow time_window_cfg; // Use the struct from hpp
                time_window_cfg.start_time = window_node["start"].as<double>();
                time_window_cfg.end_time = window_node["end"].as<double>();
                
                YAML::Node force_node = window_node["force"];
                time_window_cfg.is_sine_wave = false; 

                if (force_node.IsSequence() && force_node.size() >= 1) {
                    if (force_node[0].IsScalar() && force_node[0].as<std::string>() == "sin") {
                        time_window_cfg.is_sine_wave = true;
                        time_window_cfg.force.push_back(1.0); // Default amplitude for X
                        if (force_node.size() > 1) time_window_cfg.force.push_back(force_node[1].as<double>(0.0));
                        else time_window_cfg.force.push_back(0.0); 
                    } else {
                        for(const auto& val : force_node) {
                            time_window_cfg.force.push_back(val.as<double>());
                        }
                        while(time_window_cfg.force.size() < 2) time_window_cfg.force.push_back(0.0); // Pad to 2 elements
                    }
                } else { 
                    time_window_cfg.force.assign({0.0, 0.0}); // Default to [0,0] if not a sequence
                }
                wind_phase_cfg.time_windows.push_back(time_window_cfg);
            }
        }
        config.wind_phases.push_back(wind_phase_cfg);
    }
}

void ConfigReader::loadOutputSettings(SimulationConfig& config, const YAML::Node& node) {
    if (!node || !node.IsMap()) {
        std::cerr << "Warning: 'output_settings' or 'output' node not found or not a map in YAML." << std::endl;
        return;
    }
    config.csv_enabled = node["csv_enabled"].as<bool>(config.csv_enabled);
    config.csv_prefix = node["csv_prefix"].as<std::string>(config.csv_prefix);
    if (node["console_output"]) {
        config.console_output_enabled = node["console_output"]["enabled"].as<bool>(config.console_output_enabled);
        config.console_update_interval = node["console_output"]["update_interval"].as<double>(config.console_update_interval);
    }
}

bool loadFuzzyParamsYAML(const std::string& file, FuzzyParams& fp) {
    std::string actual_filepath = findConfigFilePath(file);
    std::ifstream in(actual_filepath);
    if (!in.is_open()) {
        std::cerr << "Could not open fuzzy params file: " << actual_filepath << std::endl;
        return false;
    }
    std::string line;
    std::string section;
    std::string current_var;
    while (std::getline(in, line)) {
        line = trim_cfg(line);
        if (line.empty() || line[0] == '#') continue;
        if (line == "membership_functions:") { section = "mf"; current_var = ""; continue; }
        if (line == "rules:") { section = "rules"; current_var = ""; continue; }

        if (section == "mf") {
            if (line.length() > 1 && line.back() == ':' && line.find_first_of(" \t") == std::string::npos) {
                current_var = trim_cfg(line.substr(0, line.size() - 1));
                fp.sets[current_var] = {}; 
                continue;
            }
            if (current_var.empty()) continue;

            auto pos = line.find(':');
            if (pos == std::string::npos) continue;
            std::string setname = trim_cfg(line.substr(0, pos));
            std::string rest = line.substr(pos + 1);
            auto lb = rest.find('[');
            auto rb = rest.find(']');
            if (lb == std::string::npos || rb == std::string::npos) continue;
            std::string nums_str = rest.substr(lb + 1, rb - lb - 1);
            auto values = parseNumberList_cfg(nums_str);
            if (values.size() == 6) {
                fp.sets[current_var][setname] = {values[0], values[1], values[2], values[3], values[4], values[5]};
            }
        } else if (section == "rules") {
            if (line[0] == '-') {
                auto lb = line.find('[');
                auto rb = line.find(']');
                if (lb == std::string::npos || rb == std::string::npos) continue;
                auto tokens = parseStringList_cfg(line.substr(lb + 1, rb - lb - 1));
                if (tokens.size() == 4) {
                    fp.rules.push_back({tokens[0], tokens[1], tokens[2], tokens[3]});
                }
            }
        }
    }
    return true;
}

} // namespace agent_control_pkg