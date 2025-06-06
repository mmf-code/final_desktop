// agent_control_pkg/src/multi_drone_pid_test_main.cpp
#include "../include/agent_control_pkg/pid_controller.hpp"
#include "../include/agent_control_pkg/gt2_fuzzy_logic_system.hpp"
#include "../include/agent_control_pkg/config_reader.hpp" // Uses the new ConfigReader
#include <iostream>
#include <vector>
#include <iomanip>
#include <cmath>
#include <fstream>
#include <string>
#include <algorithm> // For std::clamp
#include <sstream>   // For std::ostringstream
#include <map>       // For FuzzyParams
#include <array>     // For FuzzyParams
#include <chrono>    // For timestamp generation
#include <filesystem>// For directory creation

// For filesystem operations
namespace fs = std::filesystem;

// --- Oscillation Analysis for Z-N Auto-tuning ---
struct OscillationAnalysisResult {
    enum Status { UNDEFINED, DAMPED, SUSTAINED, GROWING, NO_OSCILLATION, INSUFFICIENT_DATA, CSV_ERROR };
    Status status = UNDEFINED;
    double period_pu = 0.0;
    double last_kp_tested = 0.0; // Kp that led to this result
};

// Function to parse CSV and analyze for oscillations
OscillationAnalysisResult analyze_zn_csv_for_oscillations(
    const std::string& csv_filepath,
    double target_x_for_phase1,
    double phase1_start_time,
    double phase1_end_time_for_analysis
);

// Function to run a single Z-N simulation instance (P-only)
// Returns the path to the CSV generated
std::string run_single_zn_instance(
    double current_kp_for_zn_test,
    agent_control_pkg::SimulationConfig base_config // Pass by value
);

// --- Start: Local FLS helper structs and functions (specific to this main file) ---
// These are kept local as per your original structure for this file.
// If they were identical to what agent_control_main.cpp uses, they could be moved
// to a common helper or into config_reader.cpp if loadFuzzyParamsYAML was part of ConfigReader class.
struct FuzzySetFOU_Local {
    double l1, l2, l3, u1, u2, u3;
};

struct FuzzyParams_Local {
    std::map<std::string, std::map<std::string, FuzzySetFOU_Local>> sets;
    std::vector<std::array<std::string, 4>> rules;
};

static std::string trim_local(const std::string& s) {
    size_t start = s.find_first_not_of(" \t\r\n");
    size_t end = s.find_last_not_of(" \t\r\n");
    if (start == std::string::npos) return "";
    return s.substr(start, end - start + 1);
}

static std::vector<double> parseNumberList_local(const std::string& in) {
    std::vector<double> values;
    std::stringstream ss(in);
    std::string tok;
    while (std::getline(ss, tok, ',')) {
        tok = trim_local(tok);
        if (!tok.empty()) {
            try {
                values.push_back(std::stod(tok));
            } catch (const std::exception& e) {
                std::cerr << "Warning: Could not parse number '" << tok << "' in list. Error: " << e.what() << std::endl;
            }
        }
    }
    return values;
}

static std::vector<std::string> parseStringList_local(const std::string& in) {
    std::vector<std::string> values;
    std::stringstream ss(in);
    std::string tok;
    while (std::getline(ss, tok, ',')) {
        values.push_back(trim_local(tok));
    }
    return values;
}

// Uses the global findConfigFilePath from config_reader.hpp/cpp
static bool loadFuzzyParamsLocal(const std::string& file, FuzzyParams_Local& fp) {
    std::string actual_path = agent_control_pkg::findConfigFilePath(file);
    std::ifstream in(actual_path);
    if (!in.is_open()) {
        std::cerr << "Could not open fuzzy params file: " << actual_path << std::endl;
        return false;
    }
    std::string line;
    std::string section;
    std::string current_var;
    fp.sets.clear();
    fp.rules.clear();

    while (std::getline(in, line)) {
        line = trim_local(line);
        if (line.empty() || line[0] == '#') continue;
        if (line == "membership_functions:") { section = "mf"; current_var = ""; continue; }
        if (line == "rules:") { section = "rules"; current_var = ""; continue; }
        if (section == "mf") {
            if (line.length() > 1 && line.back() == ':' && line.find_first_of(" \t") == std::string::npos) { // Check for valid var name
                current_var = trim_local(line.substr(0, line.size() - 1));
                fp.sets[current_var]; // Ensure the map entry for current_var exists
                continue;
            }
            if (current_var.empty()) {
                // std::cerr << "Warning: Fuzzy MF line encountered outside a variable block: " << line << std::endl;
                continue;
            }
            auto pos = line.find(':');
            if (pos == std::string::npos) continue;
            std::string setname = trim_local(line.substr(0, pos));
            std::string rest = line.substr(pos + 1);
            auto lb = rest.find('[');
            auto rb = rest.find(']');
            if (lb == std::string::npos || rb == std::string::npos) continue;
            std::string nums = rest.substr(lb + 1, rb - lb - 1);
            auto values = parseNumberList_local(nums);
            if (values.size() == 6) {
                fp.sets[current_var][setname] = {values[0], values[1], values[2], values[3], values[4], values[5]};
            } else {
                 std::cerr << "Warning: Fuzzy set '" << setname << "' for var '" << current_var << "' has incorrect number of params. Expected 6, got " << values.size() << std::endl;
            }
        } else if (section == "rules") {
             if (line.rfind("- [", 0) == 0 && line.back() == ']') {
                auto tokens = parseStringList_local(line.substr(3, line.size() - 4));
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

static void applyFuzzyParamsLocal(agent_control_pkg::GT2FuzzyLogicSystem& fls, const FuzzyParams_Local& fp) {
    using FOU_Ctrl = agent_control_pkg::GT2FuzzyLogicSystem::IT2TriangularFS_FOU;
    for (const auto& varPair : fp.sets) {
        const std::string& var = varPair.first;
        if (var == "correction") fls.addOutputVariable(var);
        else fls.addInputVariable(var);
        for (const auto& setPair : varPair.second) {
            const auto& fou_data = setPair.second;
            fls.addFuzzySetToVariable(var, setPair.first, FOU_Ctrl{fou_data.l1, fou_data.l2, fou_data.l3, fou_data.u1, fou_data.u2, fou_data.u3});
        }
    }
    for (const auto& r : fp.rules) {
        fls.addRule({{{"error", r[0]}, {"dError", r[1]}, {"wind", r[2]}}, {"correction", r[3]}});
    }
}
// --- End: Local FLS helper structs and functions ---

// --- FakeDrone Struct ---
struct FakeDrone {
    double position_x = 0.0;
    double position_y = 0.0;
    double velocity_x = 0.0;
    double velocity_y = 0.0;
    double prev_error_x_fls = 0.0; // For FLS dError calculation
    double prev_error_y_fls = 0.0; // For FLS dError calculation

    void update(double accel_cmd_x, double accel_cmd_y, double dt,
                double external_force_x = 0.0, double external_force_y = 0.0) {
        velocity_x += (accel_cmd_x + external_force_x) * dt;
        velocity_y += (accel_cmd_y + external_force_y) * dt;
        velocity_x *= 0.98; // Simple drag
        velocity_y *= 0.98; // Simple drag
        position_x += velocity_x * dt;
        position_y += velocity_y * dt;
    }
};

// --- PerformanceMetrics Struct ---
struct PerformanceMetrics {
    double peak_value = 0.0;
    double peak_time = 0.0;
    double overshoot_percent = 0.0;
    double settling_time_2percent = -1.0;
    double initial_value_for_metrics = 0.0;
    double target_value_for_metrics = 0.0;
    bool in_settling_band = false;
    double time_entered_settling_band = -1.0;
    bool phase_active_for_metrics = false;

    void reset(double initial_val, double target_val) {
        peak_value = initial_val; 
        peak_time = 0.0;
        overshoot_percent = 0.0;
        settling_time_2percent = -1.0;
        initial_value_for_metrics = initial_val;
        target_value_for_metrics = target_val;
        in_settling_band = false;
        time_entered_settling_band = -1.0;
        phase_active_for_metrics = true;
    }

    void update_metrics(double current_value, double time_now) {
        if (!phase_active_for_metrics) return;

        if (target_value_for_metrics > initial_value_for_metrics) { 
            if (current_value > peak_value) { peak_value = current_value; peak_time = time_now; }
        } else if (target_value_for_metrics < initial_value_for_metrics) { 
             if (current_value < peak_value) { peak_value = current_value; peak_time = time_now; }
        } else { 
            peak_value = initial_value_for_metrics; 
        }

        const double SETTLING_PERCENTAGE = 0.02;
        double settling_range_abs = std::abs(target_value_for_metrics - initial_value_for_metrics);
        double settling_tolerance;

        if (settling_range_abs < 1e-3 && std::abs(target_value_for_metrics) > 1e-9) { 
             settling_tolerance = std::abs(target_value_for_metrics * 0.10); 
        } else if (settling_range_abs < 1e-3) { 
             settling_tolerance = 0.05; 
        } else { 
             settling_tolerance = settling_range_abs * SETTLING_PERCENTAGE;
        }
        if (settling_tolerance < 1e-4) settling_tolerance = 1e-4; 

        if (std::abs(current_value - target_value_for_metrics) <= settling_tolerance) {
            if (!in_settling_band) {
                time_entered_settling_band = time_now;
                in_settling_band = true;
            }
            if (settling_time_2percent < 0.0) { 
                settling_time_2percent = time_entered_settling_band;
            }
        } else {
            if (in_settling_band) { 
                settling_time_2percent = -1.0; 
            }
            in_settling_band = false;
            time_entered_settling_band = -1.0; 
        }
    }

    void finalize_metrics_calculation(double phase_start_time_for_relative_metrics) {
        if (!phase_active_for_metrics) return;

        if (std::abs(target_value_for_metrics - initial_value_for_metrics) > 1e-6) { 
            if (target_value_for_metrics > initial_value_for_metrics) { 
                overshoot_percent = ((peak_value - target_value_for_metrics) / (target_value_for_metrics - initial_value_for_metrics)) * 100.0;
            } else { 
                overshoot_percent = ((target_value_for_metrics - peak_value) / (initial_value_for_metrics - target_value_for_metrics)) * 100.0;
            }
            
            if ( (target_value_for_metrics > initial_value_for_metrics && peak_value <= target_value_for_metrics) ||
                 (target_value_for_metrics < initial_value_for_metrics && peak_value >= target_value_for_metrics) ||
                 (std::abs(peak_value - initial_value_for_metrics) < 1e-6 && std::abs(target_value_for_metrics - initial_value_for_metrics) > 1e-6 ) ) {
                overshoot_percent = 0.0;
            }
            if (overshoot_percent < 0) overshoot_percent = 0.0; 
        } else {
            overshoot_percent = 0.0; 
        }

        if (settling_time_2percent < 0.0 && in_settling_band) {
            settling_time_2percent = time_entered_settling_band;
        }
        // Make times relative to phase start
        if (peak_time >= phase_start_time_for_relative_metrics) peak_time -= phase_start_time_for_relative_metrics; else peak_time = 0;
        if (settling_time_2percent >= phase_start_time_for_relative_metrics) settling_time_2percent -= phase_start_time_for_relative_metrics;
        else if (settling_time_2percent >=0.0) settling_time_2percent = 0.0; // Settled at or before phase start
    }
};

// --- Custom Clamp Function ---
template<typename T>
T clamp(T value, T min_val, T max_val) {
    if (value < min_val) return min_val;
    if (value > max_val) return max_val;
    return value;
}

// --- Timestamp for filenames ---
static std::string getCurrentTimestamp() {
    auto now = std::chrono::system_clock::now();
    std::time_t tt = std::chrono::system_clock::to_time_t(now);
    std::tm tm_struct;
    #ifdef _WIN32
        localtime_s(&tm_struct, &tt);
    #else
        localtime_r(&tt, &tm_struct);
    #endif
    std::ostringstream oss;
    oss << std::put_time(&tm_struct, "%Y%m%d_%H%M%S");
    return oss.str();
}

// Function to setup default configuration if YAML fails
agent_control_pkg::SimulationConfig get_default_simulation_config() {
    std::cout << "WARNING: Using internal default simulation parameters for multi_drone_pid_test_main." << std::endl;
    agent_control_pkg::SimulationConfig defaultConfig;
    // PID defaults are already set in SimPIDParams struct definition in config_reader.hpp

    // Simulation settings
    defaultConfig.dt = 0.05;
    defaultConfig.total_time = 65.0; 
    defaultConfig.num_drones = 1; // Default to 1 drone for simpler fallback

    // ZN Tuning defaults (off by default)
    defaultConfig.zn_tuning_params.enable = false;
    defaultConfig.zn_tuning_params.kp_test_value = 1.0;
    defaultConfig.zn_tuning_params.simulation_time = 30.0;

    // Controller settings
    defaultConfig.enable_fls = false;
    defaultConfig.fuzzy_params_file = "fuzzy_params.yaml";

    // Scenario settings
    defaultConfig.wind_enabled = false;
    defaultConfig.formation_side_length = 4.0; // Less relevant for 1 drone default

    // Default initial positions for 1 drone
    defaultConfig.initial_positions.push_back({0.0,0.0});

    // Default phases for 1 drone
    defaultConfig.phases.push_back({{5.0,0.0}, 0.0}); // Target (5,0), starts at t=0
    defaultConfig.total_time = 15.0; // Shorter for single drone default

    // Default Wind (empty, as wind_enabled is false)
    // defaultConfig.wind_phases already default empty

    // Output settings
    defaultConfig.output_directory = "simulation_outputs_default";
    defaultConfig.csv_enabled = true;
    defaultConfig.csv_prefix = "multi_drone_test_DEFAULT";
    defaultConfig.metrics_enabled = true;
    defaultConfig.metrics_prefix = "metrics_DEFAULT";
    defaultConfig.console_output_enabled = true;
    defaultConfig.console_update_interval = 1.0;

    return defaultConfig;
}

// =====================================================================================
// Function to run a single Z-N simulation instance (P-only)
// =====================================================================================
std::string run_single_zn_instance(
    double current_kp_for_zn_test,
    agent_control_pkg::SimulationConfig base_config // Pass by value to modify locally
) {
    agent_control_pkg::SimulationConfig config = base_config; // Work on a copy

    // Ensure ZN specific settings for this instance run
    config.zn_tuning_params.enable = true; // This flag controls PID gains (Kp=test, Ki=0, Kd=0) in C++
    config.zn_tuning_params.kp_test_value = current_kp_for_zn_test;
    config.enable_fls = false;   // FLS must be off for ZN
    config.wind_enabled = false; // Wind must be off for ZN
    // Simulation time for ZN is already set in config.zn_tuning_params.simulation_time by caller

    double kp_actual = config.zn_tuning_params.kp_test_value;
    double ki_actual = 0.0;
    double kd_actual = 0.0;
    double simulation_time_actual = config.zn_tuning_params.simulation_time;

    std::cout << "  Running ZN Instance: Kp_test = " << std::fixed << std::setprecision(3) << kp_actual 
              << ", SimTime = " << simulation_time_actual << "s" << std::endl;

    // --- File Naming & Output ---
    std::string timestamp = getCurrentTimestamp();
    std::string base_filename_prefix_val = "zn_auto_test"; // Specific prefix
    std::string directory_path_str = config.output_directory + "/zn_tuning_auto_search"; // Specific subdir
    
    fs::path output_dir_path(directory_path_str);
    try {
        if (!fs::exists(output_dir_path)) {
            fs::create_directories(output_dir_path);
        }
    } catch (const fs::filesystem_error& fs_err) {
        std::cerr << "  Filesystem error creating directory " << output_dir_path.string() << ": " << fs_err.what() << std::endl;
        return ""; // Indicate failure
    }

    std::ostringstream oss_filename_suffix;
    oss_filename_suffix << "_Kp" << std::fixed << std::setprecision(3) << kp_actual;
    // No FLS/Wind for ZN, so _FLS_OFF_WIND_OFF is implicit
    oss_filename_suffix << "_" << timestamp;
    
    fs::path csv_filepath = output_dir_path / (base_filename_prefix_val + oss_filename_suffix.str() + ".csv");
    // No separate metrics/info text file needed for each auto ZN step, CSV is key

    // --- Core Simulation Logic (simplified from your previous main loop) ---
    const int NUM_DRONES = config.num_drones > 0 ? config.num_drones : 1;
    double dt = config.dt;
    double output_min = config.pid_params.output_min;
    double output_max = config.pid_params.output_max;
    std::vector<FakeDrone> drones(NUM_DRONES);
    // Initialize drone positions
    if (config.initial_positions.size() >= NUM_DRONES) {
        for (int i = 0; i < NUM_DRONES; ++i) {
            drones[i].position_x = config.initial_positions[i].first;
            drones[i].position_y = config.initial_positions[i].second;
        }
    } else {
        for(int i=0; i < NUM_DRONES; ++i) {
            drones[i].position_x = static_cast<double>(i) * 1.0; 
            drones[i].position_y = 0.0;
        }
    }

    std::vector<agent_control_pkg::PIDController> pid_x_controllers;
    std::vector<agent_control_pkg::PIDController> pid_y_controllers;
    for (int i = 0; i < NUM_DRONES; ++i) {
        pid_x_controllers.emplace_back(kp_actual, ki_actual, kd_actual, output_min, output_max, drones[i].position_x);
        pid_y_controllers.emplace_back(kp_actual, ki_actual, kd_actual, output_min, output_max, drones[i].position_y);
    }
    
    // Formation offsets
    std::vector<std::pair<double, double>> formation_offsets(NUM_DRONES);
    if (NUM_DRONES > 0) formation_offsets[0] = {0.0, (sqrt(3.0) / 3.0) * config.formation_side_length};
    if (NUM_DRONES > 1) formation_offsets[1] = {-config.formation_side_length / 2.0, -(sqrt(3.0) / 6.0) * config.formation_side_length};
    if (NUM_DRONES > 2) formation_offsets[2] = {config.formation_side_length / 2.0, -(sqrt(3.0) / 6.0) * config.formation_side_length};
    for(int i=3; i < NUM_DRONES; ++i) formation_offsets[i] = {0.0, 0.0};

    // Phase Management (simplified for ZN - primarily first phase step)
    const int MAX_PHASES_FROM_CONFIG = static_cast<int>(config.phases.size());
    int current_phase_idx = -1; 

    std::ofstream csv_file;
    if (config.csv_enabled) {
        csv_file.open(csv_filepath);
        if (!csv_file.is_open()) {
            std::cerr << "  Error opening ZN CSV file: " << csv_filepath.string() << std::endl;
            return ""; // Indicate failure
        }
        // Write CSV header
        csv_file << "Time";
        for (int i = 0; i < NUM_DRONES; ++i) {
             csv_file << ",TargetX" << i << ",CurrentX" << i << ",ErrorX" << i << ",PIDOutX" << i << ",FinalCmdX" << i;
             // For P-only, I and D terms are zero, FLS is off.
             csv_file << ",TargetY" << i << ",CurrentY" << i << ",ErrorY" << i << ",PIDOutY" << i << ",FinalCmdY" << i;
        }
        csv_file << "\n";
    }

    double last_console_print_time_zn = -config.console_update_interval; 

    for (double time_now = 0.0; time_now <= simulation_time_actual + dt/2.0; time_now += dt) {
        int new_phase_idx_candidate = current_phase_idx;
        // Simplified phase logic for ZN: focus on first phase, or ensure one default phase exists
        if (config.phases.empty()) { // Should have been handled by caller, but defensive
            if (current_phase_idx == -1) new_phase_idx_candidate = 0;
        } else {
            for (int p_cfg_idx = MAX_PHASES_FROM_CONFIG - 1; p_cfg_idx >= 0; --p_cfg_idx) {
                if (time_now >= config.phases[p_cfg_idx].start_time - dt/2.0) { 
                    new_phase_idx_candidate = p_cfg_idx;
                    break;
                }
            }
             if (current_phase_idx == -1 && time_now >= config.phases[0].start_time - dt/2.0) new_phase_idx_candidate = 0;
        }
        
        if (new_phase_idx_candidate != current_phase_idx ) { // Phase change or initial phase
            current_phase_idx = new_phase_idx_candidate;
            // Set setpoints for PIDs based on config.phases[current_phase_idx]
            double center_x = 5.0, center_y = 0.0; // Default ZN target
            if (current_phase_idx >= 0 && current_phase_idx < MAX_PHASES_FROM_CONFIG && !config.phases.empty()) {
                center_x = config.phases[current_phase_idx].center[0];
                center_y = config.phases[current_phase_idx].center[1];
            }
            for (int i = 0; i < NUM_DRONES; ++i) {
                pid_x_controllers[i].setSetpoint(center_x + formation_offsets[i].first);
                pid_y_controllers[i].setSetpoint(center_y + formation_offsets[i].second);
                pid_x_controllers[i].reset(); pid_y_controllers[i].reset();
            }
        }

        if(config.csv_enabled && csv_file.is_open()) csv_file << time_now;
        for (int i = 0; i < NUM_DRONES; ++i) {
            // P-only control
            double error_x = pid_x_controllers[i].getSetpoint() - drones[i].position_x;
            double error_y = pid_y_controllers[i].getSetpoint() - drones[i].position_y;
            auto terms_x = pid_x_controllers[i].calculate_with_terms(drones[i].position_x, dt);
            auto terms_y = pid_y_controllers[i].calculate_with_terms(drones[i].position_y, dt);
            double cmd_x = clamp(terms_x.total_output, output_min, output_max);
            double cmd_y = clamp(terms_y.total_output, output_min, output_max);
            drones[i].update(cmd_x, cmd_y, dt, 0,0); // No wind, no FLS

            if(config.csv_enabled && csv_file.is_open()){
                 csv_file << "," << pid_x_controllers[i].getSetpoint() << "," << drones[i].position_x << "," << error_x 
                          << "," << terms_x.total_output << "," << cmd_x;
                 csv_file << "," << pid_y_controllers[i].getSetpoint() << "," << drones[i].position_y << "," << error_y 
                          << "," << terms_y.total_output << "," << cmd_y;
            }
        }
        if (config.csv_enabled && csv_file.is_open()) csv_file << "\n";
        
        if (config.console_output_enabled && (time_now - last_console_print_time_zn >= config.console_update_interval - dt/2.0 ) ) {
             if (NUM_DRONES > 0) { 
                std::cout << "  ZN Kp=" << kp_actual << " T=" << std::fixed << std::setprecision(1) << time_now
                          << " D0_X:" << std::fixed << std::setprecision(2) << drones[0].position_x 
                          << " D0_ErrX: " << std::fixed << std::setprecision(3) << (pid_x_controllers[0].getSetpoint() - drones[0].position_x)
                          << std::endl;
             }
             last_console_print_time_zn = time_now;
        }
    }
    if(config.csv_enabled && csv_file.is_open()) csv_file.close();
    
    std::cout << "  ZN Instance for Kp=" << kp_actual << " complete. CSV: " << csv_filepath.string() << std::endl;
    return csv_filepath.string();
}

// =====================================================================================
// Function to parse CSV and analyze for oscillations
// =====================================================================================
OscillationAnalysisResult analyze_zn_csv_for_oscillations(
    const std::string& csv_filepath,
    double target_x_for_phase1,
    double phase1_start_time,
    double phase1_end_time_for_analysis) 
{
    OscillationAnalysisResult result;
    result.status = OscillationAnalysisResult::UNDEFINED; // Default

    std::ifstream file(csv_filepath);
    if (!file.is_open()) {
        std::cerr << "Error: Could not open CSV file for analysis: " << csv_filepath << std::endl;
        result.status = OscillationAnalysisResult::CSV_ERROR;
        return result;
    }

    std::string line;
    std::vector<double> time_data;
    std::vector<double> drone0_x_pos_data;
    std::vector<double> drone0_x_error_data;

    // Skip header
    if (!std::getline(file, line)) {
        result.status = OscillationAnalysisResult::CSV_ERROR;
        return result;
    }

    int time_col = 0;    // Assuming Time is column 0
    int current_x0_col = 2; // Assuming CurrentX0 is column 2 (Time, TargetX0, CurrentX0, ...)
    int error_x0_col = 3;   // Assuming ErrorX0 is column 3
    
    // Rudimentary CSV parsing
    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string field;
        std::vector<std::string> fields;
        while(std::getline(ss, field, ',')) {
            fields.push_back(field);
        }
        if (fields.size() > std::max(current_x0_col, error_x0_col)) {
            try {
                double t = std::stod(fields[time_col]);
                if (t >= phase1_start_time && t < phase1_end_time_for_analysis) {
                    time_data.push_back(t);
                    drone0_x_error_data.push_back(std::stod(fields[error_x0_col]));
                }
            } catch (const std::exception& e) {
                // Skip parsing errors
            }
        }
    }
    file.close();

    if (drone0_x_error_data.size() < 20) {
        result.status = OscillationAnalysisResult::INSUFFICIENT_DATA;
        std::cout << "  Analysis: Insufficient data points (" << drone0_x_error_data.size() << ") in the phase window for robust analysis." << std::endl;
        return result;
    }

    // Find peaks and troughs
    std::vector<int> peak_indices, trough_indices;
    for (size_t i = 1; i < drone0_x_error_data.size() - 1; ++i) {
        if (drone0_x_error_data[i] > drone0_x_error_data[i-1] && drone0_x_error_data[i] > drone0_x_error_data[i+1]) {
            peak_indices.push_back(i);
        }
        if (drone0_x_error_data[i] < drone0_x_error_data[i-1] && drone0_x_error_data[i] < drone0_x_error_data[i+1]) {
            trough_indices.push_back(i);
        }
    }
    
    std::cout << "  Analysis: Found " << peak_indices.size() << " peaks and " << trough_indices.size() << " troughs." << std::endl;

    if (peak_indices.size() < 3 || trough_indices.size() < 3) {
        result.status = OscillationAnalysisResult::DAMPED;
        if (peak_indices.empty() && trough_indices.empty() && drone0_x_error_data.size() > 0) {
             bool settled = true;
             for(size_t i = drone0_x_error_data.size()/2; i < drone0_x_error_data.size(); ++i) {
                 if (std::abs(drone0_x_error_data[i]) > 0.1 * std::abs(target_x_for_phase1)) {
                     settled = false; break;
                 }
             }
             if (settled && std::abs(drone0_x_error_data.back()) < 0.05 * std::abs(target_x_for_phase1)) 
                 result.status = OscillationAnalysisResult::NO_OSCILLATION;
        }
        return result;
    }

    // Combine and sort all extrema
    std::vector<int> all_extrema_indices = peak_indices;
    all_extrema_indices.insert(all_extrema_indices.end(), trough_indices.begin(), trough_indices.end());
    std::sort(all_extrema_indices.begin(), all_extrema_indices.end());
    all_extrema_indices.erase(std::unique(all_extrema_indices.begin(), all_extrema_indices.end()), all_extrema_indices.end());

    if (all_extrema_indices.size() < 5) {
        result.status = OscillationAnalysisResult::DAMPED;
        return result;
    }

    // Calculate amplitudes
    std::vector<double> amplitudes;
    for (size_t i = 0; i < all_extrema_indices.size(); ++i) {
         amplitudes.push_back(std::abs(drone0_x_error_data[all_extrema_indices[i]]));
    }

    // Calculate periods
    std::vector<double> periods;
    for (size_t i = 0; i < peak_indices.size() - 1; ++i) {
        periods.push_back(time_data[peak_indices[i+1]] - time_data[peak_indices[i]]);
    }
    for (size_t i = 0; i < trough_indices.size() - 1; ++i) {
        periods.push_back(time_data[trough_indices[i+1]] - time_data[trough_indices[i]]);
    }
    std::sort(periods.begin(), periods.end());
    
    if (periods.empty()) {
        result.status = OscillationAnalysisResult::DAMPED;
        return result;
    }
    result.period_pu = periods.size() > 0 ? periods[periods.size()/2] : 0.0;

    // Analyze amplitude trend
    size_t half_size = amplitudes.size() / 2;
    if (half_size < 2) {
        result.status = OscillationAnalysisResult::DAMPED;
        return result;
    }

    double first_half_avg_amp = 0.0;
    for (size_t i = 0; i < half_size; ++i) first_half_avg_amp += amplitudes[i];
    first_half_avg_amp /= half_size;

    double second_half_avg_amp = 0.0;
    for (size_t i = amplitudes.size() - half_size; i < amplitudes.size(); ++i) second_half_avg_amp += amplitudes[i];
    second_half_avg_amp /= half_size;

    std::cout << "  Analysis: AvgAmp1=" << first_half_avg_amp << ", AvgAmp2=" << second_half_avg_amp 
              << ", EstPu=" << result.period_pu << std::endl;

    if (result.period_pu < 0.1) {
        result.status = OscillationAnalysisResult::NO_OSCILLATION;
        result.period_pu = 0.0;
        return result;
    }

    double amp_ratio = (first_half_avg_amp > 1e-3) ? (second_half_avg_amp / first_half_avg_amp) : 1.0;
    double stability_threshold = 0.20; // +/- 20% for sustained

    if (amp_ratio > (1.0 + stability_threshold) && second_half_avg_amp > 0.05) {
        result.status = OscillationAnalysisResult::GROWING;
    } else if (amp_ratio < (1.0 - stability_threshold) || second_half_avg_amp < 0.02) {
        result.status = OscillationAnalysisResult::DAMPED;
    } else {
        result.status = OscillationAnalysisResult::SUSTAINED;
    }

    return result;
}

int main() {
    // --- Load Configuration ---
    agent_control_pkg::SimulationConfig config;
    bool config_loaded_successfully = false;
    try {
        config = agent_control_pkg::ConfigReader::loadConfig("simulation_params.yaml");
        config_loaded_successfully = true; 
        if (config.phases.empty() && config.num_drones > 0 && !config.zn_tuning_params.enable) {
            std::cerr << "Warning: Main config loaded but phases are empty for a normal run. Consider defaults or check YAML." << std::endl;
        }
    } catch (const YAML::Exception& e) {
        std::cerr << "YAML parsing error loading main configuration: " << e.what() << std::endl;
    } catch (const std::runtime_error& e) {
        std::cerr << "Runtime error loading main configuration: " << e.what() << std::endl;
    }

    if (!config_loaded_successfully) {
        std::cout << "Configuration loading failed or was incomplete. Using internal defaults." << std::endl;
        config = get_default_simulation_config();
    }

    if (config.zn_tuning_params.enable_auto_search) { // New YAML flag for auto Ku/Pu
        std::cout << "!!!!!!!! AUTONOMOUS ZIEGLER-NICHOLS Ku/Pu SEARCH ACTIVE !!!!!!!!" << std::endl;
        
        double kp_current = config.zn_tuning_params.auto_search_kp_start;
        double kp_step    = config.zn_tuning_params.auto_search_kp_step;
        double kp_max     = config.zn_tuning_params.auto_search_kp_max;

        double found_ku = -1.0;
        double found_pu = -1.0;
        double prev_kp_damped = -1.0; // Kp from last damped response

        OscillationAnalysisResult analysis_result;

        for (; kp_current <= kp_max; kp_current += kp_step) {
            std::cout << "\n--- Auto Z-N: Testing Kp = " << kp_current << " ---" << std::endl;
            
            // Prepare config for this specific ZN run
            agent_control_pkg::SimulationConfig run_config = config;
            run_config.zn_tuning_params.enable = true; // Ensure ZN mode is on for the instance
            run_config.zn_tuning_params.kp_test_value = kp_current;
            // run_config.total_time will be overridden by zn_tuning_params.simulation_time inside run_single_zn_instance

            std::string generated_csv_path = run_single_zn_instance(kp_current, run_config);

            if (generated_csv_path.empty()) {
                std::cerr << "Error running simulation for Kp = " << kp_current << ". Aborting auto Z-N." << std::endl;
                break;
            }
            
            // Determine target X and analysis window for Phase 1 from config
            double target_x_p1 = 5.0; // Default
            double start_time_p1 = 0.0;
            double end_time_analysis = run_config.zn_tuning_params.simulation_time;

            if (!run_config.phases.empty()) {
                target_x_p1 = run_config.phases[0].center[0]; // Assuming X is first
                // start_time_p1 is effectively 0 due to ZN logic in run_single_zn_instance
                if (run_config.phases.size() > 1) {
                    end_time_analysis = std::min(run_config.zn_tuning_params.simulation_time, 
                                                 run_config.phases[1].start_time);
                }
            }

            analysis_result = analyze_zn_csv_for_oscillations(generated_csv_path, target_x_p1, start_time_p1, end_time_analysis);
            analysis_result.last_kp_tested = kp_current;

            std::cout << "Kp = " << kp_current << ": Status = ";
            switch(analysis_result.status) {
                case OscillationAnalysisResult::DAMPED: std::cout << "DAMPED"; prev_kp_damped = kp_current; break;
                case OscillationAnalysisResult::SUSTAINED: std::cout << "SUSTAINED"; break;
                case OscillationAnalysisResult::GROWING: std::cout << "GROWING"; break;
                case OscillationAnalysisResult::NO_OSCILLATION: std::cout << "NO_OSCILLATION"; prev_kp_damped = kp_current; break;
                default: std::cout << "ANALYSIS_ERROR_OR_INSUFFICIENT_DATA"; break;
            }
            if (analysis_result.period_pu > 0) {
                std::cout << ", Estimated Pu = " << analysis_result.period_pu;
            }
            std::cout << std::endl;

            if (analysis_result.status == OscillationAnalysisResult::SUSTAINED) {
                found_ku = kp_current;
                found_pu = analysis_result.period_pu;
                std::cout << "*** Found Ku = " << found_ku << ", Pu = " << found_pu << " ***" << std::endl;
                break; 
            } else if (analysis_result.status == OscillationAnalysisResult::GROWING) {
                std::cout << "Oscillations growing. Ku is likely less than " << kp_current << "." << std::endl;
                if (prev_kp_damped > 0) {
                    found_ku = (prev_kp_damped + kp_current) / 2.0; // Midpoint or just prev_kp_damped
                     std::cout << "Estimating Ku based on transition from damped/no_osc to growing: " << found_ku << std::endl;
                    // Could try to re-run with this estimated Ku or re-analyze previous CSV for Pu.
                    // For simplicity, if prev_kp_damped had a Pu estimate, use that, or Pu from current growing if stable enough.
                    // This part needs more sophisticated logic for robust Pu if Ku is interpolated.
                    // Let's assume if it's growing, the period from *this* run is still a decent estimate for Pu near Ku.
                    if (analysis_result.period_pu > 0) {
                        found_pu = analysis_result.period_pu;
                         std::cout << "Using Pu = " << found_pu << " from current growing oscillation for this Ku estimate." << std::endl;
                    } else {
                        std::cout << "Could not estimate Pu for the growing oscillation. Manual check needed." << std::endl;
                    }

                } else {
                    std::cout << "No prior damped response recorded to refine Ku. Manual check needed." << std::endl;
                }
                break;
            }
        }

        if (found_ku > 0 && found_pu > 0) {
            std::cout << "\n--- AUTONOMOUS Z-N RESULT ---" << std::endl;
            std::cout << "Final Ku = " << found_ku << std::endl;
            std::cout << "Final Pu = " << found_pu << std::endl;
            // Calculate and print Z-N PID gains
            double Kp_zn_pid = 0.6 * found_ku;
            double Ti_zn = found_pu / 2.0;
            double Kd_zn_pid_val = Kp_zn_pid * (found_pu / 8.0);
            double Ki_zn_pid = (Ti_zn > 1e-6) ? Kp_zn_pid / Ti_zn : 0.0;
            std::cout << "Calculated Z-N PID: Kp=" << Kp_zn_pid << ", Ki=" << Ki_zn_pid << ", Kd=" << Kd_zn_pid_val << std::endl;
            // TODO: Optionally write these to a result file or update simulation_params.yaml
        } else {
            std::cout << "\n--- AUTONOMOUS Z-N: Could not automatically determine Ku and Pu. ---" << std::endl;
            std::cout << "Please review generated CSV files in " 
                      << (config.output_directory + "/zn_tuning_auto_search/") << std::endl;
        }

    } else if (config.auto_kp_tuning_params.enable) { // Your existing auto Kp sweep
        // ... your existing auto_kp_tuning logic using run_single_simulation_instance ...
        // ... (this part can remain as it was for general Kp sweeps) ...
    }

    // >>> ZIEGLER-NICHOLS TUNING PARAMETERS (from config) <<<
    const bool ZN_TUNING_ACTIVE = config.zn_tuning_params.enable;
    const double ZN_KP_TEST_VALUE = config.zn_tuning_params.kp_test_value;
    const double ZN_SIMULATION_TIME = config.zn_tuning_params.simulation_time;
    // >>> END ZIEGLER-NICHOLS TUNING SECTION <<<

    FuzzyParams_Local fls_yaml_params_local; // Use the local struct for this file
    bool fls_should_be_enabled = config.enable_fls && !ZN_TUNING_ACTIVE; 
    if (fls_should_be_enabled) {
        if (!loadFuzzyParamsLocal(config.fuzzy_params_file, fls_yaml_params_local)) { // Use the local loader
            std::cerr << "Failed to load FLS params from: " << config.fuzzy_params_file
                      << ". Disabling FLS for this run." << std::endl;
            fls_should_be_enabled = false;
        }
    }

    // --- Use loaded/default config values ---
    double kp_config = config.pid_params.kp;
    double ki_config = config.pid_params.ki;
    double kd_config = config.pid_params.kd;
    double output_min = config.pid_params.output_min;
    double output_max = config.pid_params.output_max;

    double kp_actual, ki_actual, kd_actual; 

    if (ZN_TUNING_ACTIVE) {
        std::cout << "!!!!!!!! ZIEGLER-NICHOLS Ku/Pu FINDING MODE ACTIVE (from config) !!!!!!!!" << std::endl;
        std::cout << "Testing with Kp = " << ZN_KP_TEST_VALUE << ", Ki = 0, Kd = 0" << std::endl;
        std::cout << "Observe Drone 0 X-axis for sustained oscillations on the first setpoint change." << std::endl;
        kp_actual = ZN_KP_TEST_VALUE;
        ki_actual = 0.0;
        kd_actual = 0.0;
        fls_should_be_enabled = false; 
        config.wind_enabled = false;   // Effective wind status for ZN run
        
        if (config.phases.empty()) {
            std::cout << "Z-N Tuning: No phases in config, adding a default step phase: Target (5,0) at t=0." << std::endl;
            config.phases.push_back({{5.0, 0.0}, 0.0}); 
        } else if (config.phases[0].start_time != 0.0) {
            std::cout << "Z-N Tuning: Forcing first phase to start at t=0 for immediate step response." << std::endl;
            config.phases[0].start_time = 0.0;
        }
        // For ZN, we might only care about the first phase for ZN_SIMULATION_TIME
        // If ZN_SIMULATION_TIME is shorter than the first phase's implicit duration, that's fine.
        // If ZN_SIMULATION_TIME is longer, it might go into subsequent configured phases, which is usually not intended for simple Ku/Pu finding.
        // Consider making it so ZN mode only uses the first configured phase.
        // For now, it will run for ZN_SIMULATION_TIME and process phases that fall within that time.

    } else {
        kp_actual = kp_config;
        ki_actual = ki_config;
        kd_actual = kd_config;
        std::cout << "Running with configured PID gains: Kp=" << kp_actual << ", Ki=" << ki_actual << ", Kd=" << kd_actual << std::endl;
    }

    const int NUM_DRONES = config.num_drones > 0 ? config.num_drones : 1;
    double dt = config.dt;
    double simulation_time_actual = ZN_TUNING_ACTIVE ? ZN_SIMULATION_TIME : config.total_time; 
    bool ENABLE_WIND_ACTUAL = config.wind_enabled && !ZN_TUNING_ACTIVE; 
    bool USE_FLS_ACTUAL = fls_should_be_enabled;

    // --- Initialize Drones ---
    std::vector<FakeDrone> drones(NUM_DRONES);
    if (config.initial_positions.size() >= NUM_DRONES) {
        for (int i = 0; i < NUM_DRONES; ++i) {
            drones[i].position_x = config.initial_positions[i].first;
            drones[i].position_y = config.initial_positions[i].second;
        }
    } else {
        std::cerr << "Warning: Insufficient initial_positions in config for " << NUM_DRONES << " drones. Using default staggered positions." << std::endl;
        for(int i=0; i < NUM_DRONES; ++i) {
            drones[i].position_x = static_cast<double>(i) * 1.0; drones[i].position_y = 0.0;
        }
    }

    // --- PID & FLS Controllers ---
    std::vector<agent_control_pkg::PIDController> pid_x_controllers;
    std::vector<agent_control_pkg::PIDController> pid_y_controllers;
    std::vector<agent_control_pkg::GT2FuzzyLogicSystem> fls_x_controllers_vec(NUM_DRONES);
    std::vector<agent_control_pkg::GT2FuzzyLogicSystem> fls_y_controllers_vec(NUM_DRONES);

    for (int i = 0; i < NUM_DRONES; ++i) {
        pid_x_controllers.emplace_back(kp_actual, ki_actual, kd_actual, output_min, output_max, drones[i].position_x);
        pid_y_controllers.emplace_back(kp_actual, ki_actual, kd_actual, output_min, output_max, drones[i].position_y);
        if (USE_FLS_ACTUAL) {
            applyFuzzyParamsLocal(fls_x_controllers_vec[i], fls_yaml_params_local); // Use local FLS params
            applyFuzzyParamsLocal(fls_y_controllers_vec[i], fls_yaml_params_local);
        }
    }

    // --- Formation Definition ---
    double formation_side_length = config.formation_side_length;
    std::vector<std::pair<double, double>> formation_offsets(NUM_DRONES);
    if (NUM_DRONES > 0) formation_offsets[0] = {0.0, (sqrt(3.0) / 3.0) * formation_side_length};
    if (NUM_DRONES > 1) formation_offsets[1] = {-formation_side_length / 2.0, -(sqrt(3.0) / 6.0) * formation_side_length};
    if (NUM_DRONES > 2) formation_offsets[2] = {formation_side_length / 2.0, -(sqrt(3.0) / 6.0) * formation_side_length};
    for(int i=3; i < NUM_DRONES; ++i) formation_offsets[i] = {0.0, 0.0}; // Default for >3 drones

    // --- Phase Management & Targets ---
    const int MAX_PHASES_FROM_CONFIG = static_cast<int>(config.phases.size());
    int current_phase_idx = -1; 

    // --- Performance Metrics Storage ---
    int num_metric_phases = std::max(1, MAX_PHASES_FROM_CONFIG); // Ensure at least one phase for metrics
    std::vector<std::vector<PerformanceMetrics>> drone_metrics_x(NUM_DRONES, std::vector<PerformanceMetrics>(num_metric_phases));
    std::vector<std::vector<PerformanceMetrics>> drone_metrics_y(NUM_DRONES, std::vector<PerformanceMetrics>(num_metric_phases));
    std::vector<bool> phase_has_been_active(num_metric_phases, false);
    std::vector<double> phase_actual_start_times(num_metric_phases, 0.0);


    // --- File Naming & Output ---
    std::string timestamp = getCurrentTimestamp();
    std::string base_filename_prefix_val = ZN_TUNING_ACTIVE ? "zn_test" : config.csv_prefix;
    std::string directory_path_str = config.output_directory; // Base output directory
    if (ZN_TUNING_ACTIVE) { 
        directory_path_str += "/zn_tuning"; // Optional: Subfolder for ZN results
    }

    fs::path output_dir_path(directory_path_str);
     try {
        if (!fs::exists(output_dir_path)) {
            fs::create_directories(output_dir_path);
        }
    } catch (const fs::filesystem_error& fs_err) {
        std::cerr << "Filesystem error creating directory " << output_dir_path.string() << ": " << fs_err.what() << std::endl;
        output_dir_path = "."; // Fallback to current directory
    }

    std::ostringstream oss_filename_suffix;
    if (ZN_TUNING_ACTIVE) {
        oss_filename_suffix << "_Kp" << std::fixed << std::setprecision(3) << ZN_KP_TEST_VALUE;
    } else {
        oss_filename_suffix << "_Kp" << std::fixed << std::setprecision(3) << kp_actual
                            << "_Ki" << std::fixed << std::setprecision(3) << ki_actual
                            << "_Kd" << std::fixed << std::setprecision(3) << kd_actual;
    }
    oss_filename_suffix << (USE_FLS_ACTUAL ? "_FLS_ON" : "_FLS_OFF")
                        << (ENABLE_WIND_ACTUAL ? "_WIND_ON" : "_WIND_OFF")
                        << "_" << timestamp;

    fs::path csv_filepath = output_dir_path / (base_filename_prefix_val + oss_filename_suffix.str() + ".csv");
    fs::path metrics_filepath = output_dir_path / ( (ZN_TUNING_ACTIVE ? "zn_metrics_info" : config.metrics_prefix) + oss_filename_suffix.str() + ".txt");
    
    std::ofstream csv_file;
    if (config.csv_enabled) {
        csv_file.open(csv_filepath);
        if (!csv_file.is_open()) {
            std::cerr << "Error opening CSV file: " << csv_filepath.string() << std::endl;
            // config.csv_enabled = false; // Or handle error differently
        } else {
            csv_file << "Time";
            for (int i = 0; i < NUM_DRONES; ++i) {
                csv_file << ",TargetX" << i << ",CurrentX" << i << ",ErrorX" << i << ",PIDOutX" << i
                         << ",FLSCorrX" << i << ",FinalCmdX" << i
                         << ",PTermX" << i << ",ITermX" << i << ",DTermX" << i
                         << ",TargetY" << i << ",CurrentY" << i << ",ErrorY" << i << ",PIDOutY" << i
                         << ",FLSCorrY" << i << ",FinalCmdY" << i
                         << ",PTermY" << i << ",ITermY" << i << ",DTermY" << i;
            }
            if (ENABLE_WIND_ACTUAL) csv_file << ",SimWindX,SimWindY";
            csv_file << "\n";
        }
    }

    std::ofstream metrics_file_stream;
    if (config.metrics_enabled && !ZN_TUNING_ACTIVE) { 
        metrics_file_stream.open(metrics_filepath);
        if(!metrics_file_stream.is_open()){
            std::cerr << "Error opening metrics file: " << metrics_filepath.string() << std::endl;
            // config.metrics_enabled = false; // Or handle error
        } else {
            metrics_file_stream << "Simulation Metrics for: " << base_filename_prefix_val + oss_filename_suffix.str() << "\n";
            metrics_file_stream << "PID Gains: Kp=" << kp_actual << ", Ki=" << ki_actual << ", Kd=" << kd_actual << "\n";
            metrics_file_stream << "FLS: " << (USE_FLS_ACTUAL ? "ON" : "OFF") << ", Wind: " << (ENABLE_WIND_ACTUAL ? "ON" : "OFF") << "\n";
            metrics_file_stream << "=======================================================================\n";
        }
    } else if (ZN_TUNING_ACTIVE) { // Create a simple info file for ZN runs
         metrics_file_stream.open(metrics_filepath);
         if(metrics_file_stream.is_open()){
            metrics_file_stream << "Ziegler-Nichols Tuning Run Information\n";
            metrics_file_stream << "Timestamp: " << timestamp << "\n";
            metrics_file_stream << "Kp_test_value: " << ZN_KP_TEST_VALUE << "\n";
            metrics_file_stream << "Simulation Duration: " << ZN_SIMULATION_TIME << "s\n";
            metrics_file_stream << "CSV Data File: " << csv_filepath.filename().string() << "\n";
            metrics_file_stream << "Objective: Observe Drone 0 X-axis for sustained oscillations after first setpoint change.\n";
         }
    }


    std::cout << "Starting Multi-Drone Test..." << std::endl;
    std::cout << " Config File: " << (config_loaded_successfully ? "simulation_params.yaml (Loaded)" : "Internal Defaults Used") << std::endl;
    if (ZN_TUNING_ACTIVE) {
         std::cout << " Z-N Kp_test = " << ZN_KP_TEST_VALUE << std::endl;
    } else {
        std::cout << " PID Gains: Kp=" << kp_actual << ", Ki=" << ki_actual << ", Kd=" << kd_actual << std::endl;
        std::cout << " FLS: " << (USE_FLS_ACTUAL ? "ON" : "OFF") << std::endl;
        std::cout << " Wind: " << (ENABLE_WIND_ACTUAL ? "ON" : "OFF") << std::endl;
    }
    if(config.csv_enabled && csv_file.is_open()) std::cout << " CSV Output: " << csv_filepath.string() << std::endl;
    if(metrics_file_stream.is_open()) std::cout << " Metrics/Info Output: " << metrics_filepath.string() << std::endl;

    double simulated_wind_x = 0.0;
    double simulated_wind_y = 0.0;
    double last_console_print_time = -config.console_update_interval; 

    for (double time_now = 0.0; time_now <= simulation_time_actual + dt/2.0; time_now += dt) { // Loop slightly past to include endpoint
        // --- Phase Transition Logic ---
        int new_phase_idx_candidate = current_phase_idx;
        if (!config.phases.empty()) { // Only if phases are defined
            for (int p_cfg_idx = MAX_PHASES_FROM_CONFIG - 1; p_cfg_idx >= 0; --p_cfg_idx) {
                if (time_now >= config.phases[p_cfg_idx].start_time - dt/2.0) { 
                    new_phase_idx_candidate = p_cfg_idx;
                    break;
                }
            }
        } else if (current_phase_idx == -1 && ZN_TUNING_ACTIVE) { // Default phase for ZN if config.phases was empty
            new_phase_idx_candidate = 0; 
        } else if (current_phase_idx == -1 && !config.phases.empty()) { // No phases active yet, but phases are configured
             if (time_now >= config.phases[0].start_time - dt/2.0) new_phase_idx_candidate = 0;
        }


        if (new_phase_idx_candidate != current_phase_idx && new_phase_idx_candidate < num_metric_phases ) {
            current_phase_idx = new_phase_idx_candidate;
            phase_has_been_active[current_phase_idx] = true;
            phase_actual_start_times[current_phase_idx] = time_now;

            std::cout << "Time: " << std::fixed << std::setprecision(1) << time_now
                      << "s - Activating PHASE " << current_phase_idx + 1;
            
            double formation_center_x = 0.0, formation_center_y = 0.0; 
            // Get target from config if phase index is valid for config.phases
            if (current_phase_idx < MAX_PHASES_FROM_CONFIG && !config.phases.empty()) { 
                formation_center_x = config.phases[current_phase_idx].center[0];
                formation_center_y = config.phases[current_phase_idx].center[1];
                std::cout << " (Target Center: " << formation_center_x << ", " << formation_center_y << ")";
            } else if (ZN_TUNING_ACTIVE && MAX_PHASES_FROM_CONFIG == 0) { // ZN tuning and no phases were in config initially
                formation_center_x = 5.0; formation_center_y = 0.0; // Default step for ZN
                std::cout << " (ZN Default Target Center: " << formation_center_x << ", " << formation_center_y << ")";
            }
             std::cout << std::endl;

            for (int i = 0; i < NUM_DRONES; ++i) {
                double target_x = formation_center_x + formation_offsets[i].first;
                double target_y = formation_center_y + formation_offsets[i].second;
                
                pid_x_controllers[i].setSetpoint(target_x);
                pid_y_controllers[i].setSetpoint(target_y);
                pid_x_controllers[i].reset(); 
                pid_y_controllers[i].reset();
                
                drones[i].prev_error_x_fls = 0.0; 
                drones[i].prev_error_y_fls = 0.0;

                drone_metrics_x[i][current_phase_idx].reset(drones[i].position_x, target_x);
                drone_metrics_y[i][current_phase_idx].reset(drones[i].position_y, target_y);
            }
        }

        // --- Wind Simulation ---
        simulated_wind_x = 0.0;
        simulated_wind_y = 0.0;
        if (ENABLE_WIND_ACTUAL && current_phase_idx >=0 ) { // current_phase_idx is 0-indexed
            for (const auto& wp_cfg : config.wind_phases) { // wp_cfg.phase_number is 1-indexed
                if (wp_cfg.phase_number == (current_phase_idx + 1) ) { 
                    for (const auto& tw : wp_cfg.time_windows) {
                        // Time window start/end are relative to the start of *this* simulation phase
                        double time_in_current_phase = time_now - phase_actual_start_times[current_phase_idx];
                        if (time_in_current_phase >= tw.start_time && time_in_current_phase < tw.end_time) {
                            if (tw.is_sine_wave) {
                                if(!tw.force.empty()) simulated_wind_x += tw.force[0] * sin(time_now * tw.sine_frequency_rad_s); 
                                if(tw.force.size() > 1) simulated_wind_y += tw.force[1] * sin(time_now * tw.sine_frequency_rad_s); // Optional: Y can also be sine
                            } else {
                                if(!tw.force.empty()) simulated_wind_x += tw.force[0];
                                if(tw.force.size() > 1) simulated_wind_y += tw.force[1];
                            }
                        }
                    }
                }
            }
        }

        // --- Control and Update Loop ---
        if(config.csv_enabled && csv_file.is_open()) csv_file << time_now;

        for (int i = 0; i < NUM_DRONES; ++i) {
            double error_x = pid_x_controllers[i].getSetpoint() - drones[i].position_x;
            double error_y = pid_y_controllers[i].getSetpoint() - drones[i].position_y;

            double d_error_x_fls = (dt > 1e-9) ? (error_x - drones[i].prev_error_x_fls) / dt : 0.0;
            double d_error_y_fls = (dt > 1e-9) ? (error_y - drones[i].prev_error_y_fls) / dt : 0.0;

            agent_control_pkg::PIDController::PIDTerms terms_x = pid_x_controllers[i].calculate_with_terms(drones[i].position_x, dt);
            agent_control_pkg::PIDController::PIDTerms terms_y = pid_y_controllers[i].calculate_with_terms(drones[i].position_y, dt);

            double fls_correction_x = 0.0;
            double fls_correction_y = 0.0;
            if (USE_FLS_ACTUAL) {
                fls_correction_x = fls_x_controllers_vec[i].calculateOutput(error_x, d_error_x_fls, simulated_wind_x);
                fls_correction_y = fls_y_controllers_vec[i].calculateOutput(error_y, d_error_y_fls, simulated_wind_y);
            }

            double final_cmd_x = clamp(terms_x.total_output + fls_correction_x, output_min, output_max);
            double final_cmd_y = clamp(terms_y.total_output + fls_correction_y, output_min, output_max);

            drones[i].update(final_cmd_x, final_cmd_y, dt, simulated_wind_x, simulated_wind_y);

            drones[i].prev_error_x_fls = error_x;
            drones[i].prev_error_y_fls = error_y;

            if(current_phase_idx >= 0 && current_phase_idx < num_metric_phases && drone_metrics_x[i][current_phase_idx].phase_active_for_metrics) {
                drone_metrics_x[i][current_phase_idx].update_metrics(drones[i].position_x, time_now);
                drone_metrics_y[i][current_phase_idx].update_metrics(drones[i].position_y, time_now);
            }

             if(config.csv_enabled && csv_file.is_open()){
                csv_file << "," << pid_x_controllers[i].getSetpoint() << "," << drones[i].position_x << "," << error_x << "," << terms_x.total_output
                         << "," << fls_correction_x << "," << final_cmd_x
                         << "," << terms_x.p << "," << terms_x.i << "," << terms_x.d
                         << "," << pid_y_controllers[i].getSetpoint() << "," << drones[i].position_y << "," << error_y << "," << terms_y.total_output
                         << "," << fls_correction_y << "," << final_cmd_y
                         << "," << terms_y.p << "," << terms_y.i << "," << terms_y.d;
            }
        }
        if (config.csv_enabled && csv_file.is_open() && ENABLE_WIND_ACTUAL) csv_file << "," << simulated_wind_x << "," << simulated_wind_y;
        if (config.csv_enabled && csv_file.is_open()) csv_file << "\n";
        
        if (config.console_output_enabled && (time_now - last_console_print_time >= config.console_update_interval - dt/2.0 ) ) {
             if (NUM_DRONES > 0) { 
                std::cout << "T=" << std::fixed << std::setprecision(1) << time_now
                          << " Ph:" << (current_phase_idx >=0 ? current_phase_idx + 1 : 0) // Show 0 if no phase active
                          << " D0_Pos:(" << std::fixed << std::setprecision(2) << drones[0].position_x << "," << drones[0].position_y << ")";
                if (ENABLE_WIND_ACTUAL) {
                     std::cout << " Wind:(" << std::fixed << std::setprecision(2) << simulated_wind_x << "," << simulated_wind_y << ")";
                }
                if (ZN_TUNING_ACTIVE) { 
                    double zn_error_x = pid_x_controllers[0].getSetpoint() - drones[0].position_x;
                    std::cout << " D0_ErrX: " << std::fixed << std::setprecision(3) << zn_error_x;
                }
                std::cout << std::endl;
             }
             last_console_print_time = time_now;
        }
    }
    if(config.csv_enabled && csv_file.is_open()) csv_file.close();

    // --- Finalize and Print/Save Metrics ---
    std::cout << "\n--- FINAL PERFORMANCE METRICS ---" << std::endl;
    if (ZN_TUNING_ACTIVE) {
        std::cout << "Z-N Tuning Run (Kp_test = " << ZN_KP_TEST_VALUE << ") complete." << std::endl;
        std::cout << "Analyze CSV (" << csv_filepath.string() << ") for Drone 0 X-axis oscillations." << std::endl;
    } else if (num_metric_phases > 0) { 
        for (int p_idx = 0; p_idx < num_metric_phases; ++p_idx) {
            if (!phase_has_been_active[p_idx] && !(p_idx == 0 && MAX_PHASES_FROM_CONFIG == 0 && ZN_TUNING_ACTIVE) ) {
                // Skip phases that never became active, unless it's the default phase 0 for a ZN run with no phases in config
                 if (p_idx == 0 && MAX_PHASES_FROM_CONFIG == 0 && ZN_TUNING_ACTIVE && !phase_has_been_active[0]) {
                    // This case is for ZN when config.phases was empty, phase_has_been_active[0] might still be false
                    // but we want to finalize metrics for this implicit phase 0.
                } else if (p_idx > 0) { // Always process phase 0 if it was active, otherwise skip if not active
                    continue;
                } else if (p_idx == 0 && !phase_has_been_active[0]) { // Phase 0 was never active
                    continue;
                }
            }


            if(config.metrics_enabled && metrics_file_stream.is_open()) metrics_file_stream << "\n-- METRICS FOR PHASE " << p_idx + 1 << " --\n";
            std::cout << "\n-- METRICS FOR PHASE " << p_idx + 1 << " --" << std::endl;

            if (p_idx < MAX_PHASES_FROM_CONFIG && !config.phases.empty()) { 
                 std::string phase_info = " Target Center: (" + std::to_string(config.phases[p_idx].center[0]) + ", " + std::to_string(config.phases[p_idx].center[1]) + ")\n";
                 if(config.metrics_enabled && metrics_file_stream.is_open()) metrics_file_stream << phase_info;
                 std::cout << phase_info;
            } else if (ZN_TUNING_ACTIVE && p_idx == 0 && MAX_PHASES_FROM_CONFIG == 0){
                std::string phase_info = " ZN Default Target Center: (5.0, 0.0)\n"; // Matches the default ZN phase target
                 if(config.metrics_enabled && metrics_file_stream.is_open()) metrics_file_stream << phase_info;
                 std::cout << phase_info;
            }


            for (int i = 0; i < NUM_DRONES; ++i) {
                // Ensure metrics are finalized only if the phase was active for this drone's metrics
                if (drone_metrics_x[i][p_idx].phase_active_for_metrics) {
                    drone_metrics_x[i][p_idx].finalize_metrics_calculation(phase_actual_start_times[p_idx]);
                    drone_metrics_y[i][p_idx].finalize_metrics_calculation(phase_actual_start_times[p_idx]);

                    std::ostringstream drone_metric_oss;
                    drone_metric_oss << " Drone " << i << ":\n";
                    drone_metric_oss << "  X-axis: OS=" << std::fixed << std::setprecision(1) << drone_metrics_x[i][p_idx].overshoot_percent << "%"
                              << ", ST(2%)=" << (drone_metrics_x[i][p_idx].settling_time_2percent >=0 ? std::to_string(drone_metrics_x[i][p_idx].settling_time_2percent) : "N/A") << "s"
                              << ", Peak=" << std::fixed << std::setprecision(2) << drone_metrics_x[i][p_idx].peak_value << " @ " << drone_metrics_x[i][p_idx].peak_time << "s"
                              << " (Tgt:" << drone_metrics_x[i][p_idx].target_value_for_metrics << " Init:" << drone_metrics_x[i][p_idx].initial_value_for_metrics << ")\n";
                    drone_metric_oss << "  Y-axis: OS=" << std::fixed << std::setprecision(1) << drone_metrics_y[i][p_idx].overshoot_percent << "%"
                              << ", ST(2%)=" << (drone_metrics_y[i][p_idx].settling_time_2percent >=0 ? std::to_string(drone_metrics_y[i][p_idx].settling_time_2percent) : "N/A") << "s"
                              << ", Peak=" << std::fixed << std::setprecision(2) << drone_metrics_y[i][p_idx].peak_value << " @ " << drone_metrics_y[i][p_idx].peak_time << "s"
                              << " (Tgt:" << drone_metrics_y[i][p_idx].target_value_for_metrics << " Init:" << drone_metrics_y[i][p_idx].initial_value_for_metrics << ")\n";
                    
                    std::cout << drone_metric_oss.str();
                    if(config.metrics_enabled && metrics_file_stream.is_open()) metrics_file_stream << drone_metric_oss.str();
                }
            }
        }
    } else {
        std::cout << "No phases were defined or run, or ZN mode without detailed metrics. No phase-specific metrics to display." << std::endl;
    }

    if(metrics_file_stream.is_open()) metrics_file_stream.close();

    std::cout << "\nMulti-Drone Test complete." << std::endl;
    if(config.csv_enabled && csv_file.is_open()) {} // File already closed or error handled
    else if (config.csv_enabled) {std::cout << "CSV Data was intended for: " << csv_filepath.string() << " (but may have failed to open)" << std::endl;}
    
    if(metrics_file_stream.is_open()) {} // Already closed
    else if ( (config.metrics_enabled && !ZN_TUNING_ACTIVE) || ZN_TUNING_ACTIVE) { // Check if metrics/info file was intended
        std::cout << "Metrics/Info was intended for: " << metrics_filepath.string() << " (but may have failed to open)" << std::endl;
    }

    return 0;
}