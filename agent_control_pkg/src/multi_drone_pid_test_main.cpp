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

// ------------------------------------------------------------
// Simple helpers for loading FLS configuration
// ------------------------------------------------------------

struct FuzzySetFOU {
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

int main() {
    agent_control_pkg::SimulationConfig config =
        agent_control_pkg::ConfigReader::loadConfig("pid_params.yaml",
                                                   "simulation_params.yaml");

    const bool USE_FLS = config.fls_enabled;

    FuzzyParams fls_cfg;
    if (USE_FLS) {
        loadFuzzyParams("fuzzy_params.yaml", fls_cfg);
    }

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
    std::string csv_filename = config.csv_prefix + (USE_FLS ? "_FLS" : "_NOFLS") +
                               (config.wind_enabled ? "_WIND_ON" : "_WIND_OFF") + ".csv";
    std::ofstream csv_file(csv_filename);

    csv_file << "Time";
    for (int i = 0; i < NUM_DRONES; ++i) {
        csv_file << ",TargetX" << i << ",CurrentX" << i << ",ErrorX" << i << ",PIDOutX" << i << ",PTermX" << i << ",ITermX" << i << ",DTermX" << i
                 << ",TargetY" << i << ",CurrentY" << i << ",ErrorY" << i << ",PIDOutY" << i << ",PTermY" << i << ",ITermY" << i << ",DTermY" << i;
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