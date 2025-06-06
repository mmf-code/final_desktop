// agent_control_pkg/src/multi_drone_pid_test_main.cpp
#include "../include/agent_control_pkg/pid_controller.hpp"
#include "../include/agent_control_pkg/gt2_fuzzy_logic_system.hpp"
#include "../include/agent_control_pkg/config_reader.hpp"
#include <iostream>
#include <vector>
#include <iomanip>
#include <cmath>
#include <fstream>
#include <string>
#include <algorithm> // For std::clamp
#include <sstream>   // For std::ostringstream for filename
#include <map>
#include <array>

<<<<<<< HEAD
<<<<<<< HEAD
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
=======
// --- Start: Local FLS helper structs and functions ---
>>>>>>> parent of 2a992f0 (zn_again)
struct FuzzySetFOU_Local {
=======
// ------------------------------------------------------------
// Simple helpers for loading FLS configuration
// ------------------------------------------------------------

struct FuzzySetFOU {
>>>>>>> parent of b1aaa38 (itworks)
    double l1, l2, l3, u1, u2, u3;
};

struct FuzzyParams {
    std::map<std::string, std::map<std::string, FuzzySetFOU>> sets;
    std::vector<std::array<std::string, 4>> rules;
};

static std::string trim(const std::string& s) {
    size_t start = s.find_first_not_of(" \t\r\n");
    size_t end = s.find_last_not_of(" \t\r\n");
    if (start == std::string::npos) return "";
    return s.substr(start, end - start + 1);
}

static std::vector<double> parseNumberList(const std::string& in) {
    std::vector<double> values;
    std::stringstream ss(in);
    std::string tok;
    while (std::getline(ss, tok, ',')) {
        tok = trim(tok);
        if (!tok.empty()) values.push_back(std::stod(tok));
    }
    return values;
}

static std::vector<std::string> parseStringList(const std::string& in) {
    std::vector<std::string> values;
    std::stringstream ss(in);
    std::string tok;
    while (std::getline(ss, tok, ',')) {
        values.push_back(trim(tok));
    }
    return values;
}

static std::string findConfigFile(const std::string& filename) {
    std::vector<std::string> candidates = {
        "../config/" + filename,
        "../../config/" + filename,
        "config/" + filename,
    };
    for (const auto& p : candidates) {
        std::ifstream f(p);
        if (f.good()) return p;
    }
    return filename;
}

static bool loadFuzzyParams(const std::string& file, FuzzyParams& fp) {
    std::ifstream in(findConfigFile(file));
    if (!in.is_open()) {
        std::cerr << "Could not open fuzzy params file: " << file << std::endl;
        return false;
    }
    std::string line;
    std::string section;
    std::string current_var;
    while (std::getline(in, line)) {
        line = trim(line);
        if (line.empty() || line[0] == '#') continue;
        if (line == "membership_functions:") { section = "mf"; continue; }
        if (line == "rules:") { section = "rules"; continue; }
        if (section == "mf") {
            if (line.back() == ':') {
                current_var = trim(line.substr(0, line.size() - 1));
                continue;
            }
            auto pos = line.find(':');
            if (pos == std::string::npos) continue;
            std::string setname = trim(line.substr(0, pos));
            std::string rest = line.substr(pos + 1);
            auto lb = rest.find('[');
            auto rb = rest.find(']');
            if (lb == std::string::npos || rb == std::string::npos) continue;
            std::string nums = rest.substr(lb + 1, rb - lb - 1);
            auto values = parseNumberList(nums);
            if (values.size() == 6) {
                fp.sets[current_var][setname] = {values[0], values[1], values[2], values[3], values[4], values[5]};
            }
        } else if (section == "rules") {
            if (line[0] == '-') {
                auto lb = line.find('[');
                auto rb = line.find(']');
                if (lb == std::string::npos || rb == std::string::npos) continue;
                auto tokens = parseStringList(line.substr(lb + 1, rb - lb - 1));
                if (tokens.size() == 4) {
                    fp.rules.push_back({tokens[0], tokens[1], tokens[2], tokens[3]});
                }
            }
        }
    }
    return true;
}

static void applyFuzzyParams(agent_control_pkg::GT2FuzzyLogicSystem& fls, const FuzzyParams& fp) {
    using FOU = agent_control_pkg::GT2FuzzyLogicSystem::IT2TriangularFS_FOU;
    for (const auto& varPair : fp.sets) {
        const std::string& var = varPair.first;
        if (var == "correction")
            fls.addOutputVariable(var);
        else
            fls.addInputVariable(var);
        for (const auto& setPair : varPair.second) {
            const auto& fou = setPair.second;
            fls.addFuzzySetToVariable(var, setPair.first, FOU{fou.l1, fou.l2, fou.l3, fou.u1, fou.u2, fou.u3});
        }
    }
    for (const auto& r : fp.rules) {
        fls.addRule({{{"error", r[0]}, {"dError", r[1]}, {"wind", r[2]}}, {"correction", r[3]}});
    }
}

// --- FakeDrone Struct ---
struct FakeDrone {
    double position_x = 0.0;
    double position_y = 0.0;
    double velocity_x = 0.0;
    double velocity_y = 0.0;
    double prev_error_x = 0.0;
    double prev_error_y = 0.0;

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
    bool phase_active_for_metrics = false; // To know if metrics should be updated for this phase

    void reset(double initial_val, double target_val) {
        peak_value = initial_val;
        peak_time = 0.0;
        overshoot_percent = 0.0;
        settling_time_2percent = -1.0;
        initial_value_for_metrics = initial_val;
        target_value_for_metrics = target_val;
        in_settling_band = false;
        time_entered_settling_band = -1.0;
        phase_active_for_metrics = true; // Mark as active when reset
    }

    void update_metrics(double current_value, double time_now) {
        if (!phase_active_for_metrics) return;

        if (target_value_for_metrics > initial_value_for_metrics) {
            if (current_value > peak_value) {
                peak_value = current_value;
                peak_time = time_now;
            }
        } else if (target_value_for_metrics < initial_value_for_metrics) {
             if (current_value < peak_value) {
                peak_value = current_value;
                peak_time = time_now;
            }
        } else {
             peak_value = initial_value_for_metrics;
        }

        const double SETTLING_PERCENTAGE = 0.02;
        double settling_range_abs = std::abs(target_value_for_metrics - initial_value_for_metrics);
        double settling_tolerance;
        if (settling_range_abs < 1e-3 && std::abs(target_value_for_metrics) > 1e-9) { // Target not zero, but close to initial
             settling_tolerance = std::abs(target_value_for_metrics * 0.10); // 10% of target
        } else if (settling_range_abs < 1e-3) { // Target is zero or very close to initial and zero
             settling_tolerance = 0.05; // Absolute tolerance like 0.05 units
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

    void finalize_metrics_calculation() {
        if (!phase_active_for_metrics) return;

        if (std::abs(target_value_for_metrics - initial_value_for_metrics) > 1e-6) {
            if (target_value_for_metrics > initial_value_for_metrics) {
                overshoot_percent = ((peak_value - target_value_for_metrics) / (target_value_for_metrics - initial_value_for_metrics)) * 100.0;
            } else {
                overshoot_percent = ((target_value_for_metrics - peak_value) / (initial_value_for_metrics - target_value_for_metrics)) * 100.0;
            }
            if (peak_value == initial_value_for_metrics && target_value_for_metrics != initial_value_for_metrics) overshoot_percent = 0.0;
            if (overshoot_percent < 0) overshoot_percent = 0.0;
        } else {
            overshoot_percent = 0.0;
        }

        if (settling_time_2percent < 0.0 && in_settling_band) {
            settling_time_2percent = time_entered_settling_band;
        }
    }
};

template<typename T>
T clamp(T value, T min_val, T max_val) {
    if (value < min_val) return min_val;
    if (value > max_val) return max_val;
    return value;
}

<<<<<<< HEAD
// Function to setup default configuration if YAML fails
agent_control_pkg::SimulationConfig get_default_simulation_config() {
    std::cout << "WARNING: Using internal default simulation parameters." << std::endl;
    agent_control_pkg::SimulationConfig defaultConfig;
    // PID defaults are already set in SimPIDParams struct definition
    // defaultConfig.pid_params.kp = 3.1; ... (already done by struct)

    // Simulation settings
    defaultConfig.dt = 0.05;
    defaultConfig.total_time = 65.0; // Shorter time for default
    defaultConfig.num_drones = 3;

    // Controller settings
    defaultConfig.enable_fls = false;
    defaultConfig.fuzzy_params_file = "fuzzy_params.yaml"; // Still needs to try and load this if FLS is ever enabled

    // Scenario settings
    defaultConfig.wind_enabled = false;
    defaultConfig.formation_side_length = 4.0;

    // Default initial positions
    defaultConfig.initial_positions.push_back({0.0, (sqrt(3.0)/3.0) * 4.0 - 2.0});
    defaultConfig.initial_positions.push_back({-2.0, -(sqrt(3.0)/6.0) * 4.0 - 2.0});
    defaultConfig.initial_positions.push_back({2.0, -(sqrt(3.0)/6.0) * 4.0 - 2.0});
    if (defaultConfig.num_drones == 1) { // Simpler default for 1 drone
        defaultConfig.initial_positions.clear();
        defaultConfig.initial_positions.push_back({0.0,0.0});
    }


    // Default phases
    defaultConfig.phases.push_back({{5.0, 5.0}, 0.0});
    defaultConfig.phases.push_back({{-5.0, 0.0}, 20.0});
    defaultConfig.phases.push_back({{0.0, -5.0}, 45.0});
    if (defaultConfig.num_drones == 1) { // Simpler default for 1 drone
        defaultConfig.phases.clear();
        defaultConfig.phases.push_back({{5.0,0.0},0.0});
        defaultConfig.total_time = 15.0; // Shorter for single drone default
    }


    // Default Wind (empty, as wind_enabled is false)
    // defaultConfig.wind_phases = ...;

    // Output settings
    defaultConfig.csv_enabled = true;
    defaultConfig.csv_prefix = "multi_drone_test_DEFAULT";
    defaultConfig.console_output_enabled = true;
    defaultConfig.console_update_interval = 5.0;

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

=======
>>>>>>> parent of b1aaa38 (itworks)
int main() {
    agent_control_pkg::SimulationConfig config =
        agent_control_pkg::ConfigReader::loadConfig("pid_params.yaml",
                                                   "simulation_params.yaml");

    const bool USE_FLS = config.fls_enabled;

    FuzzyParams fls_cfg;
    if (USE_FLS) {
        loadFuzzyParams("fuzzy_params.yaml", fls_cfg);
    }

<<<<<<< HEAD
    if (!config_loaded_successfully || config.phases.empty() && config.num_drones > 0) { // Added check for empty phases
        config = get_default_simulation_config();
    }

<<<<<<< HEAD
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
=======
>>>>>>> parent of 2a992f0 (zn_again)

    FuzzyParams_Local fls_yaml_params;
    if (config.enable_fls) {
        if (!loadFuzzyParamsLocal(config.fuzzy_params_file, fls_yaml_params)) {
            std::cerr << "Failed to load FLS params from: " << config.fuzzy_params_file
                      << ". Disabling FLS for this run." << std::endl;
            config.enable_fls = false;
        }
    }

    // --- Use loaded config values ---
    double kp = config.pid_params.kp;
    double ki = config.pid_params.ki;
    double kd = config.pid_params.kd;
    double output_min = config.pid_params.output_min;
    double output_max = config.pid_params.output_max;

=======
>>>>>>> parent of b1aaa38 (itworks)
    const int NUM_DRONES = config.num_drones;
    double kp = config.kp;
    double ki = config.ki;
    double kd = config.kd;
    double output_min = config.output_min;
    double output_max = config.output_max;
    double dt = config.dt;
    double simulation_time = config.total_time;

    std::vector<FakeDrone> drones(NUM_DRONES);
    for (int i = 0; i < NUM_DRONES; ++i) {
        drones[i].position_x = config.initial_positions[i].first;
        drones[i].position_y = config.initial_positions[i].second;
    }


    // --- PID and optional FLS Controllers ---
    std::vector<agent_control_pkg::PIDController> pid_x_controllers;
    std::vector<agent_control_pkg::PIDController> pid_y_controllers;
    std::vector<agent_control_pkg::GT2FuzzyLogicSystem> fls_x_controllers(NUM_DRONES);
    std::vector<agent_control_pkg::GT2FuzzyLogicSystem> fls_y_controllers(NUM_DRONES);

    for (int i = 0; i < NUM_DRONES; ++i) {
        pid_x_controllers.emplace_back(kp, ki, kd, output_min, output_max, drones[i].position_x);
        pid_y_controllers.emplace_back(kp, ki, kd, output_min, output_max, drones[i].position_y);
        if (USE_FLS) {
            applyFuzzyParams(fls_x_controllers[i], fls_cfg);
            applyFuzzyParams(fls_y_controllers[i], fls_cfg);
        }
    }

    // --- Formation Definition (Equilateral Triangle) ---
    double formation_side_length = config.formation_side_length;
    std::vector<std::pair<double, double>> formation_offsets(NUM_DRONES);
    formation_offsets[0] = {0.0, (sqrt(3.0) / 3.0) * formation_side_length};
    formation_offsets[1] = {-formation_side_length / 2.0, -(sqrt(3.0) / 6.0) * formation_side_length};
    formation_offsets[2] = {formation_side_length / 2.0, -(sqrt(3.0) / 6.0) * formation_side_length};

    // --- Phase Management & Targets ---
    const int MAX_PHASES = static_cast<int>(config.phases.size());
    std::vector<double> phase_target_center_x;
    std::vector<double> phase_target_center_y;
    std::vector<double> phase_start_times;
    for (const auto& ph : config.phases) {
        phase_target_center_x.push_back(ph.center[0]);
        phase_target_center_y.push_back(ph.center[1]);
        phase_start_times.push_back(ph.start_time);
    }
    int current_phase_idx = 0;

    // --- Performance Metrics Storage ---
    std::vector<std::vector<PerformanceMetrics>> drone_metrics_x(NUM_DRONES, std::vector<PerformanceMetrics>(MAX_PHASES));
    std::vector<std::vector<PerformanceMetrics>> drone_metrics_y(NUM_DRONES, std::vector<PerformanceMetrics>(MAX_PHASES));
    std::vector<bool> phase_has_been_active(MAX_PHASES, false);


    // --- File Naming & Output ---
<<<<<<< HEAD
<<<<<<< HEAD
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

=======
>>>>>>> parent of 2a992f0 (zn_again)
    std::ostringstream oss_filename_suffix;
    oss_filename_suffix << "_Kp" << std::fixed << std::setprecision(2) << kp
                        << "_Ki" << std::fixed << std::setprecision(2) << ki
                        << "_Kd" << std::fixed << std::setprecision(2) << kd
                        << (USE_FLS ? "_FLS_ON" : "_FLS_OFF")
                        << (ENABLE_WIND ? "_WIND_ON" : "_WIND_OFF");
    std::string csv_filename = config.csv_prefix + oss_filename_suffix.str() + ".csv";
    std::ofstream csv_file(csv_filename);

<<<<<<< HEAD
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
=======
    // ... (CSV header writing - same as before) ...
    csv_file << "Time";
    for (int i = 0; i < NUM_DRONES; ++i) {
        csv_file << ",TargetX" << i << ",CurrentX" << i << ",ErrorX" << i << ",PIDOutX" << i
                 << ",FLSCorrX" << i << ",FinalCmdX" << i
                 << ",PTermX" << i << ",ITermX" << i << ",DTermX" << i
                 << ",TargetY" << i << ",CurrentY" << i << ",ErrorY" << i << ",PIDOutY" << i
                 << ",FLSCorrY" << i << ",FinalCmdY" << i
                 << ",PTermY" << i << ",ITermY" << i << ",DTermY" << i;
>>>>>>> parent of 2a992f0 (zn_again)
=======
    std::string csv_filename = config.csv_prefix + (USE_FLS ? "_FLS" : "_NOFLS") +
                               (config.wind_enabled ? "_WIND_ON" : "_WIND_OFF") + ".csv";
    std::ofstream csv_file(csv_filename);

    csv_file << "Time";
    for (int i = 0; i < NUM_DRONES; ++i) {
        csv_file << ",TargetX" << i << ",CurrentX" << i << ",ErrorX" << i << ",PIDOutX" << i << ",PTermX" << i << ",ITermX" << i << ",DTermX" << i
                 << ",TargetY" << i << ",CurrentY" << i << ",ErrorY" << i << ",PIDOutY" << i << ",PTermY" << i << ",ITermY" << i << ",DTermY" << i;
>>>>>>> parent of b1aaa38 (itworks)
    }
    if (config.wind_enabled) csv_file << ",SimWindX,SimWindY";
    csv_file << "\n";

    std::cout << "Starting Multi-Drone PID Test..." << std::endl;
    std::cout << " Gains: Kp=" << kp << ", Ki=" << ki << ", Kd=" << kd << std::endl;
    std::cout << " Wind: " << (config.wind_enabled ? "ON" : "OFF") << std::endl;
    std::cout << " Outputting to: " << csv_filename << std::endl;

    // --- Simulation Loop ---
    double simulated_wind_x = 0.0;
    double simulated_wind_y = 0.0;

    for (double time_now = 0.0; time_now <= simulation_time; time_now += dt) {
        // --- Phase Transition Logic ---
        int new_phase_idx = current_phase_idx;
        for (int p = MAX_PHASES - 1; p >= 0; --p) { // Check from latest possible phase
            if (time_now >= phase_start_times[p]) {
                new_phase_idx = p;
                break;
            }
        }

        if (new_phase_idx != current_phase_idx || time_now < dt) { // If phase changed or it's the very first step
            if (time_now >= dt) { // Don't print for t=0 if it's not really a "change"
                 std::cout << "Time: " << std::fixed << std::setprecision(1) << time_now
                          << "s - Changing to PHASE " << new_phase_idx + 1
                          << " (Target Center: " << phase_target_center_x[new_phase_idx]
                          << ", " << phase_target_center_y[new_phase_idx] << ")" << std::endl;
            }
            current_phase_idx = new_phase_idx;
            phase_has_been_active[current_phase_idx] = true;

            double formation_center_x = phase_target_center_x[current_phase_idx];
            double formation_center_y = phase_target_center_y[current_phase_idx];

            for (int i = 0; i < NUM_DRONES; ++i) {
                double target_x = formation_center_x + formation_offsets[i].first;
                double target_y = formation_center_y + formation_offsets[i].second;
                pid_x_controllers[i].setSetpoint(target_x);
                pid_y_controllers[i].setSetpoint(target_y);
                pid_x_controllers[i].reset(); // Reset PIDs for new phase
                pid_y_controllers[i].reset();

                drone_metrics_x[i][current_phase_idx].reset(drones[i].position_x, target_x);
                drone_metrics_y[i][current_phase_idx].reset(drones[i].position_y, target_y);
            }
        }

        // --- Wind Simulation (Matches agent_control_main.cpp for consistency if needed) ---
        simulated_wind_x = 0.0;
        simulated_wind_y = 0.0;
        if (config.wind_enabled) {
            int phase_for_wind = current_phase_idx + 1;
            for (const auto& wp : config.wind_phases) {
                if (wp.phase_number != phase_for_wind) continue;
                for (const auto& tw : wp.time_windows) {
                    if (time_now >= tw.start_time && time_now < tw.end_time) {
                        if (tw.is_sine_wave) {
                            simulated_wind_x += std::sin(time_now * 2.0);
                            if (tw.force.size() > 1) simulated_wind_y += tw.force[1];
                        } else {
                            if (!tw.force.empty()) simulated_wind_x += tw.force[0];
                            if (tw.force.size() > 1) simulated_wind_y += tw.force[1];
                        }
                    }
                }
            }
        }

        // --- Control and Update Loop ---
        csv_file << time_now;
        for (int i = 0; i < NUM_DRONES; ++i) {
            double error_x = pid_x_controllers[i].getSetpoint() - drones[i].position_x;
            double d_error_x = (dt > 1e-9) ? (error_x - drones[i].prev_error_x) / dt : 0.0;
            agent_control_pkg::PIDController::PIDTerms terms_x = pid_x_controllers[i].calculate_with_terms(drones[i].position_x, dt);
            double fls_x = 0.0;
            if (USE_FLS) {
                fls_x = fls_x_controllers[i].calculateOutput(error_x, d_error_x, simulated_wind_x);
            }
            double final_cmd_x = clamp(terms_x.total_output + fls_x, output_min, output_max);

            double error_y = pid_y_controllers[i].getSetpoint() - drones[i].position_y;
            double d_error_y = (dt > 1e-9) ? (error_y - drones[i].prev_error_y) / dt : 0.0;
            agent_control_pkg::PIDController::PIDTerms terms_y = pid_y_controllers[i].calculate_with_terms(drones[i].position_y, dt);
            double fls_y = 0.0;
            if (USE_FLS) {
                fls_y = fls_y_controllers[i].calculateOutput(error_y, d_error_y, simulated_wind_y);
            }
            double final_cmd_y = clamp(terms_y.total_output + fls_y, output_min, output_max);

            drones[i].update(final_cmd_x, final_cmd_y, dt, simulated_wind_x, simulated_wind_y);
            drones[i].prev_error_x = error_x;
            drones[i].prev_error_y = error_y;

            drone_metrics_x[i][current_phase_idx].update_metrics(drones[i].position_x, time_now);
            drone_metrics_y[i][current_phase_idx].update_metrics(drones[i].position_y, time_now);

            csv_file << "," << pid_x_controllers[i].getSetpoint() << "," << drones[i].position_x << "," << error_x << "," << final_cmd_x
                     << "," << terms_x.p << "," << terms_x.i << "," << terms_x.d
                     << "," << pid_y_controllers[i].getSetpoint() << "," << drones[i].position_y << "," << error_y << "," << final_cmd_y
                     << "," << terms_y.p << "," << terms_y.i << "," << terms_y.d;
        }
        if (config.wind_enabled) csv_file << "," << simulated_wind_x << "," << simulated_wind_y;
        csv_file << "\n";

        // Console output for Drone 0
        if (static_cast<int>(time_now * 1000) % 5000 == 0 && time_now > 0.1) { // Every 5 seconds
            std::cout << "T=" << std::fixed << std::setprecision(1) << time_now
                      << " Ph:" << current_phase_idx + 1
                      << " D0_Pos:(" << std::fixed << std::setprecision(2) << drones[0].position_x << "," << drones[0].position_y << ")"
                      << " D0_Tgt:(" << pid_x_controllers[0].getSetpoint() << "," << pid_y_controllers[0].getSetpoint() << ")"
                      << (config.wind_enabled ? " Wind:(" + std::to_string(simulated_wind_x) + "," + std::to_string(simulated_wind_y) + ")" : "")
                      << std::endl;
        }
    }
    csv_file.close();

    // --- Finalize and Print Metrics ---
    std::cout << "\n--- FINAL PERFORMANCE METRICS ---" << std::endl;
    for (int p_idx = 0; p_idx < MAX_PHASES; ++p_idx) {
        if (!phase_has_been_active[p_idx]) continue; // Skip phases that were not entered

        std::cout << "\n-- METRICS FOR PHASE " << p_idx + 1 << " --" << std::endl;
        std::cout << " Target Center: (" << phase_target_center_x[p_idx] << ", " << phase_target_center_y[p_idx] << ")" << std::endl;
        for (int i = 0; i < NUM_DRONES; ++i) {
            drone_metrics_x[i][p_idx].finalize_metrics_calculation();
            drone_metrics_y[i][p_idx].finalize_metrics_calculation();

            std::cout << " Drone " << i << ":" << std::endl;
            std::cout << "  X-axis: OS=" << std::fixed << std::setprecision(1) << drone_metrics_x[i][p_idx].overshoot_percent << "%"
                      << ", ST(2%)=" << (drone_metrics_x[i][p_idx].settling_time_2percent >=0 ? std::to_string(drone_metrics_x[i][p_idx].settling_time_2percent) : "N/A") << "s"
                      << ", Peak=" << drone_metrics_x[i][p_idx].peak_value << " @ " << drone_metrics_x[i][p_idx].peak_time << "s"
                      << " (Tgt:" << drone_metrics_x[i][p_idx].target_value_for_metrics << ")" << std::endl;
            std::cout << "  Y-axis: OS=" << drone_metrics_y[i][p_idx].overshoot_percent << "%"
                      << ", ST(2%)=" << (drone_metrics_y[i][p_idx].settling_time_2percent >=0 ? std::to_string(drone_metrics_y[i][p_idx].settling_time_2percent) : "N/A") << "s"
                      << ", Peak=" << drone_metrics_y[i][p_idx].peak_value << " @ " << drone_metrics_y[i][p_idx].peak_time << "s"
                      << " (Tgt:" << drone_metrics_y[i][p_idx].target_value_for_metrics << ")" << std::endl;
        }
    }

    std::cout << "\nMulti-Drone PID Test complete. Data in: " << csv_filename << std::endl;
    return 0;
}