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
#include <sstream>   // For std::ostringstream
#include <map>       // For FuzzyParams
#include <array>     // For FuzzyParams

// --- Start: Local FLS helper structs and functions ---
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
        if (!tok.empty()) values.push_back(std::stod(tok));
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
    while (std::getline(in, line)) {
        line = trim_local(line);
        if (line.empty() || line[0] == '#') continue;
        if (line == "membership_functions:") { section = "mf"; current_var = ""; continue; }
        if (line == "rules:") { section = "rules"; current_var = ""; continue; }
        if (section == "mf") {
            if (line.length() > 0 && line.back() == ':') {
                current_var = trim_local(line.substr(0, line.size() - 1));
                fp.sets[current_var];
                continue;
            }
            if (current_var.empty()) continue;
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
            }
        } else if (section == "rules") {
            if (line[0] == '-') {
                auto lb = line.find('[');
                auto rb = line.find(']');
                if (lb == std::string::npos || rb == std::string::npos) continue;
                auto tokens = parseStringList_local(line.substr(lb + 1, rb - lb - 1));
                if (tokens.size() == 4) {
                    fp.rules.push_back({tokens[0], tokens[1], tokens[2], tokens[3]});
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
    double prev_error_x_fls = 0.0;
    double prev_error_y_fls = 0.0;

    void update(double accel_cmd_x, double accel_cmd_y, double dt,
                double external_force_x = 0.0, double external_force_y = 0.0) {
        velocity_x += (accel_cmd_x + external_force_x) * dt;
        velocity_y += (accel_cmd_y + external_force_y) * dt;
        velocity_x *= 0.98;
        velocity_y *= 0.98;
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
        } else { peak_value = initial_value_for_metrics; }

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
            if (!in_settling_band) { time_entered_settling_band = time_now; in_settling_band = true; }
            if (settling_time_2percent < 0.0) { settling_time_2percent = time_entered_settling_band; }
        } else {
            if (in_settling_band) { settling_time_2percent = -1.0; }
            in_settling_band = false; time_entered_settling_band = -1.0;
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
        } else { overshoot_percent = 0.0; }
        if (settling_time_2percent < 0.0 && in_settling_band) { settling_time_2percent = time_entered_settling_band; }
    }
};

template<typename T>
T clamp(T value, T min_val, T max_val) {
    if (value < min_val) return min_val;
    if (value > max_val) return max_val;
    return value;
}

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


int main() {
    // --- Load Configuration ---
    agent_control_pkg::SimulationConfig config;
    bool config_loaded_successfully = false;
    try {
        config = agent_control_pkg::ConfigReader::loadConfig("simulation_params.yaml");
        config_loaded_successfully = true; // Assume success if no exception
         // Check if essential parts were loaded, e.g., phases
        if (config.phases.empty() && config.num_drones > 0) { // If phases are critical and not loaded
            std::cerr << "Warning: Phases not loaded correctly from YAML, even if file was found." << std::endl;
            // config_loaded_successfully = false; // Optionally force fallback
        }
    } catch (const std::runtime_error& e) {
        std::cerr << "Error loading main configuration: " << e.what() << std::endl;
        // Continue with defaults
    }

    if (!config_loaded_successfully || config.phases.empty() && config.num_drones > 0) { // Added check for empty phases
        config = get_default_simulation_config();
    }


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

    const int NUM_DRONES = config.num_drones;
    double dt = config.dt;
    double simulation_time = config.total_time;
    bool ENABLE_WIND = config.wind_enabled;
    bool USE_FLS = config.enable_fls;

    // --- Initialize Drones ---
    std::vector<FakeDrone> drones(NUM_DRONES);
    if (config.initial_positions.size() == NUM_DRONES) {
        for (int i = 0; i < NUM_DRONES; ++i) {
            drones[i].position_x = config.initial_positions[i].first;
            drones[i].position_y = config.initial_positions[i].second;
        }
    } else { // Should be handled by get_default_simulation_config if loading failed
        std::cerr << "CRITICAL: Fallback initial positions should have been set if config load failed." << std::endl;
        // Apply a very basic default if something went extremely wrong
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
        pid_x_controllers.emplace_back(kp, ki, kd, output_min, output_max, drones[i].position_x);
        pid_y_controllers.emplace_back(kp, ki, kd, output_min, output_max, drones[i].position_y);
        if (USE_FLS) {
            applyFuzzyParamsLocal(fls_x_controllers_vec[i], fls_yaml_params);
            applyFuzzyParamsLocal(fls_y_controllers_vec[i], fls_yaml_params);
        }
    }

    // --- Formation Definition ---
    double formation_side_length = config.formation_side_length;
    std::vector<std::pair<double, double>> formation_offsets(NUM_DRONES);
    if (NUM_DRONES > 0) formation_offsets[0] = {0.0, (sqrt(3.0) / 3.0) * formation_side_length};
    if (NUM_DRONES > 1) formation_offsets[1] = {-formation_side_length / 2.0, -(sqrt(3.0) / 6.0) * formation_side_length};
    if (NUM_DRONES > 2) formation_offsets[2] = {formation_side_length / 2.0, -(sqrt(3.0) / 6.0) * formation_side_length};

    // --- Phase Management & Targets ---
    const int MAX_PHASES = static_cast<int>(config.phases.size());
    int current_phase_idx = -1; // Start at -1 to ensure first phase (index 0) is properly activated

    // --- Performance Metrics Storage ---
    std::vector<std::vector<PerformanceMetrics>> drone_metrics_x(NUM_DRONES, std::vector<PerformanceMetrics>(std::max(1, MAX_PHASES)));
    std::vector<std::vector<PerformanceMetrics>> drone_metrics_y(NUM_DRONES, std::vector<PerformanceMetrics>(std::max(1, MAX_PHASES)));
    std::vector<bool> phase_has_been_active(std::max(1,MAX_PHASES), false);

    // --- File Naming & Output ---
    std::ostringstream oss_filename_suffix;
    oss_filename_suffix << "_Kp" << std::fixed << std::setprecision(2) << kp
                        << "_Ki" << std::fixed << std::setprecision(2) << ki
                        << "_Kd" << std::fixed << std::setprecision(2) << kd
                        << (USE_FLS ? "_FLS_ON" : "_FLS_OFF")
                        << (ENABLE_WIND ? "_WIND_ON" : "_WIND_OFF");
    std::string csv_filename = config.csv_prefix + oss_filename_suffix.str() + ".csv";
    std::ofstream csv_file(csv_filename);

    // ... (CSV header writing - same as before) ...
    csv_file << "Time";
    for (int i = 0; i < NUM_DRONES; ++i) {
        csv_file << ",TargetX" << i << ",CurrentX" << i << ",ErrorX" << i << ",PIDOutX" << i
                 << ",FLSCorrX" << i << ",FinalCmdX" << i
                 << ",PTermX" << i << ",ITermX" << i << ",DTermX" << i
                 << ",TargetY" << i << ",CurrentY" << i << ",ErrorY" << i << ",PIDOutY" << i
                 << ",FLSCorrY" << i << ",FinalCmdY" << i
                 << ",PTermY" << i << ",ITermY" << i << ",DTermY" << i;
    }
    if (ENABLE_WIND) csv_file << ",SimWindX,SimWindY";
    csv_file << "\n";


    std::cout << "Starting Multi-Drone Test..." << std::endl;
    std::cout << " Config File: " << (config_loaded_successfully ? "simulation_params.yaml (Loaded)" : "Internal Defaults Used") << std::endl;
    std::cout << " PID Gains: Kp=" << kp << ", Ki=" << ki << ", Kd=" << kd << std::endl;
    std::cout << " FLS: " << (USE_FLS ? "ON" : "OFF") << std::endl;
    std::cout << " Wind: " << (ENABLE_WIND ? "ON" : "OFF") << std::endl;
    std::cout << " Outputting to: " << csv_filename << std::endl;

    // --- Simulation Loop ---
    double simulated_wind_x = 0.0;
    double simulated_wind_y = 0.0;

    for (double time_now = 0.0; time_now <= simulation_time; time_now += dt) {
        // --- Phase Transition Logic ---
        int new_phase_idx_val = current_phase_idx;
        if (!config.phases.empty()) {
            for (int p = static_cast<int>(config.phases.size()) - 1; p >= 0; --p) {
                if (time_now >= config.phases[p].start_time) {
                    new_phase_idx_val = p;
                    break;
                }
            }
        } else if (current_phase_idx == -1) { // No phases from config, ensure we enter a default "phase 0"
            new_phase_idx_val = 0;
        }


        if (new_phase_idx_val != current_phase_idx ) {
            if (current_phase_idx == -1 && new_phase_idx_val == 0) { // Initial setup for Phase 1 (index 0)
                 std::cout << "Time: " << std::fixed << std::setprecision(1) << time_now
                          << "s - Initializing to PHASE " << new_phase_idx_val + 1;
                 if (!config.phases.empty()) {
                     std::cout << " (Target Center: " << config.phases[new_phase_idx_val].center[0]
                               << ", " << config.phases[new_phase_idx_val].center[1] << ")";
                 }
                 std::cout << std::endl;
            } else if (new_phase_idx_val != current_phase_idx) { // Actual phase change
                 std::cout << "Time: " << std::fixed << std::setprecision(1) << time_now
                          << "s - Changing to PHASE " << new_phase_idx_val + 1;
                 if (!config.phases.empty()) {
                     std::cout << " (Target Center: " << config.phases[new_phase_idx_val].center[0]
                               << ", " << config.phases[new_phase_idx_val].center[1] << ")";
                 }
                 std::cout << std::endl;
            }

            current_phase_idx = new_phase_idx_val;
            if (current_phase_idx < MAX_PHASES && MAX_PHASES > 0) phase_has_been_active[current_phase_idx] = true;

            if (!config.phases.empty()) {
                double formation_center_x = config.phases[current_phase_idx].center[0];
                double formation_center_y = config.phases[current_phase_idx].center[1];

                for (int i = 0; i < NUM_DRONES; ++i) {
                    double target_x = formation_center_x + formation_offsets[i].first;
                    double target_y = formation_center_y + formation_offsets[i].second;
                    pid_x_controllers[i].setSetpoint(target_x);
                    pid_y_controllers[i].setSetpoint(target_y);
                    pid_x_controllers[i].reset();
                    pid_y_controllers[i].reset();
                    drones[i].prev_error_x_fls = 0.0;
                    drones[i].prev_error_y_fls = 0.0;

                    if (current_phase_idx < MAX_PHASES) {
                        drone_metrics_x[i][current_phase_idx].reset(drones[i].position_x, target_x);
                        drone_metrics_y[i][current_phase_idx].reset(drones[i].position_y, target_y);
                    }
                }
            }
        }


        // --- Wind Simulation ---
        simulated_wind_x = 0.0;
        simulated_wind_y = 0.0;
        if (ENABLE_WIND && current_phase_idx >=0 && current_phase_idx < config.wind_phases.size()) { // Ensure current_phase_idx is valid for wind_phases
            int active_config_phase_for_wind = current_phase_idx + 1;
            for (const auto& wp_cfg : config.wind_phases) { // Iterate through all configured wind phases
                if (wp_cfg.phase_number == active_config_phase_for_wind) { // Match current sim phase
                    for (const auto& tw : wp_cfg.time_windows) {
                        if (time_now >= tw.start_time && time_now < tw.end_time) {
                            if (tw.is_sine_wave) {
                                simulated_wind_x += tw.force[0] * sin(time_now * 2.0); 
                                if (tw.force.size() > 1) simulated_wind_y += tw.force[1]; 
                            } else {
                                if (tw.force.size() > 0) simulated_wind_x += tw.force[0];
                                if (tw.force.size() > 1) simulated_wind_y += tw.force[1];
                            }
                        }
                    }
                }
            }
        }

        // --- Control and Update Loop ---
        csv_file << time_now;
        for (int i = 0; i < NUM_DRONES; ++i) {
            double error_x = pid_x_controllers[i].getSetpoint() - drones[i].position_x;
            double error_y = pid_y_controllers[i].getSetpoint() - drones[i].position_y;

            double d_error_x_fls = (dt > 1e-9) ? (error_x - drones[i].prev_error_x_fls) / dt : 0.0;
            double d_error_y_fls = (dt > 1e-9) ? (error_y - drones[i].prev_error_y_fls) / dt : 0.0;

            agent_control_pkg::PIDController::PIDTerms terms_x = pid_x_controllers[i].calculate_with_terms(drones[i].position_x, dt);
            agent_control_pkg::PIDController::PIDTerms terms_y = pid_y_controllers[i].calculate_with_terms(drones[i].position_y, dt);

            double fls_correction_x = 0.0;
            double fls_correction_y = 0.0;
            if (USE_FLS) {
                fls_correction_x = fls_x_controllers_vec[i].calculateOutput(error_x, d_error_x_fls, simulated_wind_x);
                fls_correction_y = fls_y_controllers_vec[i].calculateOutput(error_y, d_error_y_fls, simulated_wind_y);
            }

            double final_cmd_x = clamp(terms_x.total_output + fls_correction_x, output_min, output_max);
            double final_cmd_y = clamp(terms_y.total_output + fls_correction_y, output_min, output_max);

            drones[i].update(final_cmd_x, final_cmd_y, dt, simulated_wind_x, simulated_wind_y);

            drones[i].prev_error_x_fls = error_x;
            drones[i].prev_error_y_fls = error_y;

            if(current_phase_idx >= 0 && current_phase_idx < MAX_PHASES) {
                drone_metrics_x[i][current_phase_idx].update_metrics(drones[i].position_x, time_now);
                drone_metrics_y[i][current_phase_idx].update_metrics(drones[i].position_y, time_now);
            }

            csv_file << "," << pid_x_controllers[i].getSetpoint() << "," << drones[i].position_x << "," << error_x << "," << terms_x.total_output
                     << "," << fls_correction_x << "," << final_cmd_x
                     << "," << terms_x.p << "," << terms_x.i << "," << terms_x.d
                     << "," << pid_y_controllers[i].getSetpoint() << "," << drones[i].position_y << "," << error_y << "," << terms_y.total_output
                     << "," << fls_correction_y << "," << final_cmd_y
                     << "," << terms_y.p << "," << terms_y.i << "," << terms_y.d;
        }
        if (ENABLE_WIND) csv_file << "," << simulated_wind_x << "," << simulated_wind_y;
        csv_file << "\n";

        if (config.console_output_enabled && static_cast<int>(time_now * 1000) % static_cast<int>(config.console_update_interval * 1000) == 0 && time_now > 0.1) {
             std::cout << "T=" << std::fixed << std::setprecision(1) << time_now
                      << " Ph:" << current_phase_idx + 1
                      << " D0_Pos:(" << std::fixed << std::setprecision(2) << drones[0].position_x << "," << drones[0].position_y << ")"
                      << (ENABLE_WIND ? " Wind:(" + std::to_string(simulated_wind_x) + "," + std::to_string(simulated_wind_y) + ")" : "")
                      << std::endl;
        }
    }
    csv_file.close();

    // --- Finalize and Print Metrics ---
    std::cout << "\n--- FINAL PERFORMANCE METRICS ---" << std::endl;
    if (MAX_PHASES > 0) {
        for (int p_idx = 0; p_idx < MAX_PHASES; ++p_idx) {
            if (!phase_has_been_active[p_idx] && p_idx > 0) continue;

            std::cout << "\n-- METRICS FOR PHASE " << p_idx + 1 << " --" << std::endl;
            if (p_idx < config.phases.size()) {
                std::cout << " Target Center: (" << config.phases[p_idx].center[0] << ", " << config.phases[p_idx].center[1] << ")" << std::endl;
            }
            for (int i = 0; i < NUM_DRONES; ++i) {
                drone_metrics_x[i][p_idx].finalize_metrics_calculation();
                drone_metrics_y[i][p_idx].finalize_metrics_calculation();

                std::cout << " Drone " << i << ":" << std::endl;
                std::cout << "  X-axis: OS=" << std::fixed << std::setprecision(1) << drone_metrics_x[i][p_idx].overshoot_percent << "%"
                          << ", ST(2%)=" << (drone_metrics_x[i][p_idx].settling_time_2percent >=0 ? std::to_string(drone_metrics_x[i][p_idx].settling_time_2percent) : "N/A") << "s"
                          << ", Peak=" << std::fixed << std::setprecision(2) << drone_metrics_x[i][p_idx].peak_value << " @ " << drone_metrics_x[i][p_idx].peak_time << "s"
                          << " (Tgt:" << drone_metrics_x[i][p_idx].target_value_for_metrics << ")" << std::endl;
                std::cout << "  Y-axis: OS=" << drone_metrics_y[i][p_idx].overshoot_percent << "%"
                          << ", ST(2%)=" << (drone_metrics_y[i][p_idx].settling_time_2percent >=0 ? std::to_string(drone_metrics_y[i][p_idx].settling_time_2percent) : "N/A") << "s"
                          << ", Peak=" << drone_metrics_y[i][p_idx].peak_value << " @ " << drone_metrics_y[i][p_idx].peak_time << "s"
                          << " (Tgt:" << drone_metrics_y[i][p_idx].target_value_for_metrics << ")" << std::endl;
            }
        }
    } else {
        std::cout << "No phases were defined or run. No phase-specific metrics to display." << std::endl;
    }
    std::cout << "\nMulti-Drone Test complete. Data in: " << csv_filename << std::endl;
    return 0;
}