// agent_control_main.cpp with FLS Integration and Full Metrics

#include "../include/agent_control_pkg/pid_controller.hpp"         // PID class
#include "../include/agent_control_pkg/gt2_fuzzy_logic_system.hpp" // FLS class
#include "../include/agent_control_pkg/config_reader.hpp"
#include <iostream>
#include <vector>
#include <iomanip>   // For std::fixed, std::setprecision, std::setw (if needed for formatting filenames)
#include <chrono>    // For timestamp generation
#include <ctime>
#include <cmath>     // For sqrt, abs
#include <fstream>   // For file I/O
#include <limits>    // For std::numeric_limits
#include <string>    // For std::to_string
#include <algorithm> // For std::clamp
#include <sstream>   // For std::stringstream
#include <cctype>    // For std::isspace
#include <filesystem>
#include <map>
#include <array>

// ------------------------------------------------------------
// Utility helpers for loading simple YAML parameters
// ------------------------------------------------------------

struct PIDParams {
    double kp{0.0};
    double ki{0.0};
    double kd{0.0};
    double output_min{-10.0};
    double output_max{10.0};
};

struct FuzzySetFOU {
    double l1, l2, l3, u1, u2, u3;
};

struct FuzzyParams {
    std::map<std::string, std::map<std::string, FuzzySetFOU>> sets;
    std::vector<std::array<std::string,4>> rules;
};

static std::string trim(const std::string& s) {
    size_t start = s.find_first_not_of(" \t\r\n");
    size_t end = s.find_last_not_of(" \t\r\n");
    if(start==std::string::npos) return "";
    return s.substr(start, end-start+1);
}

static std::vector<double> parseNumberList(const std::string& in) {
    std::vector<double> values;
    std::stringstream ss(in);
    std::string tok;
    while(std::getline(ss, tok, ',')) {
        tok = trim(tok);
        if(!tok.empty()) values.push_back(std::stod(tok));
    }
    return values;
}

static std::vector<std::string> parseStringList(const std::string& in) {
    std::vector<std::string> values;
    std::stringstream ss(in);
    std::string tok;
    while(std::getline(ss, tok, ',')) {
        values.push_back(trim(tok));
    }
    return values;
}

static std::string getCurrentTimestamp() {
    auto now = std::chrono::system_clock::now();
    std::time_t tt = std::chrono::system_clock::to_time_t(now);
    std::tm tm;
#ifdef _WIN32
    localtime_s(&tm, &tt);
#else
    localtime_r(&tt, &tm);
#endif
    std::ostringstream oss;
    oss << std::put_time(&tm, "%Y%m%d_%H%M%S");
    return oss.str();
}

static std::string findConfigFile(const std::string& filename) {
    std::vector<std::string> candidates = {
        "../config/" + filename,
        "../../config/" + filename,
        "config/" + filename,
    };
    for(const auto& p : candidates) {
        std::ifstream f(p);
        if(f.good()) return p;
    }
    return filename; // fallback - may fail later
}

static bool loadPIDParams(const std::string& file, PIDParams& params) {
    std::ifstream in(findConfigFile(file));
    if(!in.is_open()) {
        std::cerr << "Could not open PID params file: " << file << std::endl;
        return false;
    }
    std::string line;
    while(std::getline(in,line)) {
        line = trim(line);
        if(line.empty() || line[0]=='#') continue;
        if(line.rfind("kp:",0)==0) params.kp = std::stod(trim(line.substr(3)));
        else if(line.rfind("ki:",0)==0) params.ki = std::stod(trim(line.substr(3)));
        else if(line.rfind("kd:",0)==0) params.kd = std::stod(trim(line.substr(3)));
        else if(line.rfind("output_limits:",0)==0) {
            // read next lines for min/max
            std::string sub;
            if(std::getline(in, sub)) {
                sub = trim(sub);
                if(sub.rfind("min:",0)==0) params.output_min = std::stod(trim(sub.substr(4)));
            }
            if(std::getline(in, sub)) {
                sub = trim(sub);
                if(sub.rfind("max:",0)==0) params.output_max = std::stod(trim(sub.substr(4)));
            }
        }
    }
    return true;
}

static bool loadFuzzyParams(const std::string& file, FuzzyParams& fp) {
    std::ifstream in(findConfigFile(file));
    if(!in.is_open()) {
        std::cerr << "Could not open fuzzy params file: " << file << std::endl;
        return false;
    }
    std::string line;
    std::string section;
    std::string current_var;
    while(std::getline(in,line)) {
        line = trim(line);
        if(line.empty() || line[0]=='#') continue;
        if(line == "membership_functions:") { section = "mf"; continue; }
        if(line == "rules:") { section = "rules"; continue; }
        if(section == "mf") {
            if(line.back()==':') {
                current_var = trim(line.substr(0,line.size()-1));
                continue;
            }
            auto pos = line.find(':');
            if(pos==std::string::npos) continue;
            std::string setname = trim(line.substr(0,pos));
            std::string rest = line.substr(pos+1);
            auto lb = rest.find('[');
            auto rb = rest.find(']');
            if(lb==std::string::npos || rb==std::string::npos) continue;
            std::string nums = rest.substr(lb+1, rb-lb-1);
            auto values = parseNumberList(nums);
            if(values.size()==6) {
                fp.sets[current_var][setname] = {values[0],values[1],values[2],values[3],values[4],values[5]};
            }
        } else if(section == "rules") {
            if(line[0]=='-') {
                auto lb=line.find('[');
                auto rb=line.find(']');
                if(lb==std::string::npos || rb==std::string::npos) continue;
                auto tokens = parseStringList(line.substr(lb+1, rb-lb-1));
                if(tokens.size()==4) {
                    fp.rules.push_back({tokens[0],tokens[1],tokens[2],tokens[3]});
                }
            }
        }
    }
    return true;
}

static void applyFuzzyParams(agent_control_pkg::GT2FuzzyLogicSystem& fls, const FuzzyParams& fp) {
    using FOU = agent_control_pkg::GT2FuzzyLogicSystem::IT2TriangularFS_FOU;
    for(const auto& varPair : fp.sets) {
        const std::string& var = varPair.first;
        if(var == "correction") fls.addOutputVariable(var); else fls.addInputVariable(var);
        for(const auto& setPair : varPair.second) {
            const auto& fou = setPair.second;
            fls.addFuzzySetToVariable(var, setPair.first, FOU{fou.l1,fou.l2,fou.l3,fou.u1,fou.u2,fou.u3});
        }
    }
    for(const auto& r : fp.rules) {
        fls.addRule({{{"error",r[0]},{"dError",r[1]},{"wind",r[2]}},{"correction",r[3]}});
    }
}


// FakeDrone struct
struct FakeDrone {
    double position_x = 0.0;
    double position_y = 0.0;
    double velocity_x = 0.0;
    double velocity_y = 0.0;
    // *** ADDED for dError calculation for FLS ***
    double prev_error_x = 0.0;
    double prev_error_y = 0.0;
    // *** END ADDED ***

    // Constructor already initializes to 0.0

    void update(double accel_cmd_x, double accel_cmd_y, double dt,
                double external_force_x = 0.0, double external_force_y = 0.0) { // Add external forces
        // Assume mass = 1 for simplicity, so force = acceleration
        velocity_x += (accel_cmd_x + external_force_x) * dt; // Add external force component
        velocity_y += (accel_cmd_y + external_force_y) * dt; // Add external force component
        velocity_x *= 0.98; // Simple drag
        velocity_y *= 0.98; // Simple drag
        position_x += velocity_x * dt;
        position_y += velocity_y * dt;
    }
};

// PerformanceMetrics struct
struct PerformanceMetrics {
    double peak_value = 0.0;
    double peak_time = 0.0;
    double overshoot_percent = 0.0;
    double settling_time_2percent = -1.0; // -1.0 means not settled
    void reset(double initial_value_for_peak) {
        peak_value = initial_value_for_peak;
        peak_time = 0.0;
        overshoot_percent = 0.0;
        settling_time_2percent = -1.0;
    }
};

// Custom clamp function since std::clamp might not be available
template<typename T>
T clamp(T value, T min, T max) {
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

int main(int argc, char** argv) {
    (void)argc; (void)argv;
    agent_control_pkg::SimulationConfig config =
        agent_control_pkg::ConfigReader::loadConfig("simulation_params.yaml");
    // >>> ZIEGLER-NICHOLS TUNING SECTION - SETTINGS FOR Ku/Pu FINDING <<<
    const bool ZN_TUNING_ACTIVE = false;  // SET TO false FOR NORMAL RUN WITH Z-N GAINS
                                         // WHEN true, FLS WILL BE OFF, AND PID WILL BE P-ONLY

    // Current Kp being tested for Z-N (not used when ZN_TUNING_ACTIVE is false)
    const double ZN_KP_TEST_VALUE = 2.0; // Not used in this run

    // >>> END ZIEGLER-NICHOLS TUNING SECTION <<<

    // Flag to control FLS usage
    const bool USE_FLS = config.enable_fls && !ZN_TUNING_ACTIVE;

    const int NUM_DRONES = config.num_drones;
    FuzzyParams fls_cfg;
    loadFuzzyParams("fuzzy_params.yaml", fls_cfg);
    std::vector<FakeDrone> drones(NUM_DRONES);
    std::vector<agent_control_pkg::PIDController> pid_x_controllers;
    std::vector<agent_control_pkg::PIDController> pid_y_controllers;
    // *** FLS INSTANCES ***
    std::vector<agent_control_pkg::GT2FuzzyLogicSystem> fls_x_controllers(NUM_DRONES);
    std::vector<agent_control_pkg::GT2FuzzyLogicSystem> fls_y_controllers(NUM_DRONES);
    // *** END FLS INSTANCES ***

    // --- PID Controller Gains ---
    double kp, ki, kd;

    if (ZN_TUNING_ACTIVE) {
        std::cout << "!!!!!!!! ZIEGLER-NICHOLS Ku/Pu FINDING MODE ACTIVE !!!!!!!!" << std::endl;
        std::cout << "Testing with Kp = " << ZN_KP_TEST_VALUE << ", Ki = 0, Kd = 0" << std::endl;
        std::cout << "Observe Drone 0 X-axis for sustained oscillations." << std::endl;
        kp = ZN_KP_TEST_VALUE;
        ki = 0.0;  // Must be 0 for Z-N tuning
        kd = 0.0;  // Must be 0 for Z-N tuning
    } else {
        // Load gains from configuration file
        kp = config.pid_params.kp;
        ki = config.pid_params.ki;
        kd = config.pid_params.kd;
        std::cout << "Running with configured PID gains:" << std::endl;
        std::cout << "Kp = " << kp << ", Ki = " << ki << ", Kd = " << kd << std::endl;
        if (USE_FLS) {
            std::cout << "FLS is ENABLED" << std::endl;
        } else {
            std::cout << "FLS is DISABLED (PID-only run)" << std::endl;
        }
    }

    double output_min = config.pid_params.output_min;
    double output_max = config.pid_params.output_max;

    // Initialize controllers with Z-N derived gains
    for (int i = 0; i < NUM_DRONES; ++i) {
        // Initialize drone positions from config
        drones[i].position_x = config.initial_positions[i].first;
        drones[i].position_y = config.initial_positions[i].second;
        
        // Initialize PID controllers
        pid_x_controllers.emplace_back(kp, ki, kd, output_min, output_max, 0.0);
        pid_y_controllers.emplace_back(kp, ki, kd, output_min, output_max, 0.0);
        
        // Initialize FLS if enabled
        if (USE_FLS) {
            applyFuzzyParams(fls_x_controllers[i], fls_cfg);
            applyFuzzyParams(fls_y_controllers[i], fls_cfg);
        }
    }

    // --- File naming using config ---
    std::string controller_type_log_msg = USE_FLS ? "PID+FLS Controller" : "PID-Only Controller";
    std::string timestamp = getCurrentTimestamp();
    std::string csv_file_name = config.csv_prefix + "_" + timestamp +
                               (USE_FLS ? "_FLS" : "_NOFLS") +
                               (config.wind_enabled ? "_WIND_ON" : "_WIND_OFF") + ".csv";
    std::string metrics_file_name = std::string("performance_metrics_") + timestamp +
                                   (USE_FLS ? "_fls" : "_pid") +
                                   (config.wind_enabled ? "_wind" : "_nowind") + ".txt";

    std::filesystem::create_directories(config.output_directory);
    std::filesystem::path csv_path = std::filesystem::path(config.output_directory) / csv_file_name;
    std::filesystem::path metrics_path = std::filesystem::path(config.output_directory) / metrics_file_name;

    std::ofstream csv_file(csv_path);
    if (!csv_file.is_open()) {
        std::cerr << "Error opening CSV file!" << std::endl;
        return 1;
    }

// Write CSV header
csv_file << "Time";
for (int i = 0; i < NUM_DRONES; ++i) {
    csv_file << ",TargetX" << i << ",CurrentX" << i << ",ErrorX" << i
             << ",PIDOutX" << i << ",FLSCorrX" << i << ",FinalCmdX" << i
             << ",PTermX" << i << ",ITermX" << i << ",DTermX" << i   // <<< X-axis PID Term headers
             << ",TargetY" << i << ",CurrentY" << i << ",ErrorY" << i
             << ",PIDOutY" << i << ",FLSCorrY" << i << ",FinalCmdY" << i
             << ",PTermY" << i << ",ITermY" << i << ",DTermY" << i;  // <<< Y-axis PID Term headers
}
csv_file << ",SimWindX,SimWindY" << std::endl; // Make sure this matches what your Python script expects for wind columns

    // Open metrics file
    std::ofstream metrics_file;
    if (!ZN_TUNING_ACTIVE) { // Only create metrics file if not in P-only ZN step-by-step tuning
        metrics_file.open(metrics_path);
        if (!metrics_file.is_open()) {
            std::cerr << "Error: Could not open metrics file: " << metrics_path << std::endl;
            csv_file.close();  // Close CSV file before returning
            return 1;
        }

        // Write metrics file header
        metrics_file << std::fixed << std::setprecision(3);
        metrics_file << controller_type_log_msg << "\n";
        metrics_file << "PID Gains: Kp=" << kp << ", Ki=" << ki << ", Kd=" << kd << "\n";
        metrics_file << "=======================================================================\n";
    }

    // --- Phase Management ---
    int current_phase = 1;
    const int MAX_PHASES = static_cast<int>(config.phases.size());
    std::vector<bool> phase_triggered(MAX_PHASES + 1, false); // To ensure each phase change happens once

    // --- Performance Metric Storage (Extended for more phases) ---
    int num_metric_phases = MAX_PHASES;
    std::vector<std::vector<PerformanceMetrics>> drone_metrics_x(NUM_DRONES, std::vector<PerformanceMetrics>(num_metric_phases));
    std::vector<std::vector<PerformanceMetrics>> drone_metrics_y(NUM_DRONES, std::vector<PerformanceMetrics>(num_metric_phases));
    
    // Store initial and target values for each phase
    std::vector<std::vector<double>> initial_values_x_per_phase(num_metric_phases, std::vector<double>(NUM_DRONES));
    std::vector<std::vector<double>> initial_values_y_per_phase(num_metric_phases, std::vector<double>(NUM_DRONES));
    std::vector<std::vector<double>> target_values_x_per_phase(num_metric_phases, std::vector<double>(NUM_DRONES));
    std::vector<std::vector<double>> target_values_y_per_phase(num_metric_phases, std::vector<double>(NUM_DRONES));
    std::vector<std::vector<bool>> in_settling_band_x(num_metric_phases, std::vector<bool>(NUM_DRONES, false));
    std::vector<std::vector<bool>> in_settling_band_y(num_metric_phases, std::vector<bool>(NUM_DRONES, false));
    std::vector<std::vector<double>> time_entered_settling_x(num_metric_phases, std::vector<double>(NUM_DRONES, -1.0));
    std::vector<std::vector<double>> time_entered_settling_y(num_metric_phases, std::vector<double>(NUM_DRONES, -1.0));
    const double SETTLING_PERCENTAGE = 0.02;

    // Target centers loaded from config
    std::vector<double> phase_target_center_x;
    std::vector<double> phase_target_center_y;
    std::vector<double> phase_start_times;
    for (const auto& ph : config.phases) {
        phase_target_center_x.push_back(ph.center[0]);
        phase_target_center_y.push_back(ph.center[1]);
        phase_start_times.push_back(ph.start_time);
    }

    // Initialize drones, PIDs, FLSs, and initial metrics
    for (int i = 0; i < NUM_DRONES; ++i) {
        drones[i].position_x = config.initial_positions[i].first;
        drones[i].position_y = config.initial_positions[i].second;
        pid_x_controllers[i].setSetpoint(target_values_x_per_phase[0][i]);
        pid_y_controllers[i].setSetpoint(target_values_y_per_phase[0][i]);

        initial_values_x_per_phase[0][i] = drones[i].position_x;
        initial_values_y_per_phase[0][i] = drones[i].position_y;
        drone_metrics_x[i][0].reset(initial_values_x_per_phase[0][i]);
        drone_metrics_y[i][0].reset(initial_values_y_per_phase[0][i]);
    }

    // Define Formation from config
    double formation_center_x = config.phases[0].center[0];
    double formation_center_y = config.phases[0].center[1];
    double side_length = config.formation_side_length;
    std::vector<std::pair<double, double>> formation_offsets(NUM_DRONES);
    formation_offsets[0] = {0.0, (sqrt(3.0) / 2.0) * side_length * (2.0/3.0)};
    formation_offsets[1] = {-side_length / 2.0, -(sqrt(3.0) / 2.0) * side_length * (1.0/3.0)};
    formation_offsets[2] = {side_length / 2.0, -(sqrt(3.0) / 2.0) * side_length * (1.0/3.0)};

    for (int i = 0; i < NUM_DRONES; ++i) {
        target_values_x_per_phase[0][i] = formation_center_x + formation_offsets[i].first;
        target_values_y_per_phase[0][i] = formation_center_y + formation_offsets[i].second;
        pid_x_controllers[i].setSetpoint(target_values_x_per_phase[0][i]);
        pid_y_controllers[i].setSetpoint(target_values_y_per_phase[0][i]);
    }

    double dt = config.dt;
    double simulation_time = config.total_time; // Extended simulation time for more phases

    // --- Simulated Wind Conditions ***
    double simulated_wind_x = 0.0;
    double simulated_wind_y = 0.0;
    // *** End Wind ***

    // --- Simulation Loop ---
    for (double time_now = 0.0; time_now <= simulation_time; time_now += dt) {
        // --- Apply Wind Disturbances (Enhanced Profile) ---
        // Reset wind at the start of each iteration
        simulated_wind_x = 0.0;
        simulated_wind_y = 0.0;

        if (config.wind_enabled && !ZN_TUNING_ACTIVE) {
            for (const auto& wp : config.wind_phases) {
                if (wp.phase_number != current_phase) continue;
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

        // --- Phase Transitions and Target Changes ---
        int desired_phase = current_phase;
        for (int p = MAX_PHASES; p >= 1; --p) {
            if (time_now >= phase_start_times[p - 1]) { desired_phase = p; break; }
        }
        if (desired_phase != current_phase) {
            current_phase = desired_phase;
            phase_triggered[current_phase] = true;

            formation_center_x = phase_target_center_x[current_phase - 1];
            formation_center_y = phase_target_center_y[current_phase - 1];

            int metric_idx = current_phase - 1;
            for (int i = 0; i < NUM_DRONES; ++i) {
                initial_values_x_per_phase[metric_idx][i] = drones[i].position_x;
                initial_values_y_per_phase[metric_idx][i] = drones[i].position_y;
                target_values_x_per_phase[metric_idx][i] = formation_center_x + formation_offsets[i].first;
                target_values_y_per_phase[metric_idx][i] = formation_center_y + formation_offsets[i].second;
                pid_x_controllers[i].setSetpoint(target_values_x_per_phase[metric_idx][i]);
                pid_y_controllers[i].setSetpoint(target_values_y_per_phase[metric_idx][i]);
                drone_metrics_x[i][metric_idx].reset(initial_values_x_per_phase[metric_idx][i]);
                drone_metrics_y[i][metric_idx].reset(initial_values_y_per_phase[metric_idx][i]);
                in_settling_band_x[metric_idx][i] = false;
                in_settling_band_y[metric_idx][i] = false;
                time_entered_settling_x[metric_idx][i] = -1.0;
                time_entered_settling_y[metric_idx][i] = -1.0;
                drones[i].prev_error_x = 0.0;
                drones[i].prev_error_y = 0.0;
                pid_x_controllers[i].reset();
                pid_y_controllers[i].reset();
            }
        }

        csv_file << time_now;

        for (int i = 0; i < NUM_DRONES; ++i) {
            // Calculate current errors
            double error_x = pid_x_controllers[i].getSetpoint() - drones[i].position_x;
            double error_y = pid_y_controllers[i].getSetpoint() - drones[i].position_y;

            // Calculate dError
            double d_error_x = (dt > 1e-9) ? (error_x - drones[i].prev_error_x) / dt : 0.0;
            double d_error_y = (dt > 1e-9) ? (error_y - drones[i].prev_error_y) / dt : 0.0;

            // PID Calculation
            agent_control_pkg::PIDController::PIDTerms pid_terms_x = pid_x_controllers[i].calculate_with_terms(drones[i].position_x, dt);
            agent_control_pkg::PIDController::PIDTerms pid_terms_y = pid_y_controllers[i].calculate_with_terms(drones[i].position_y, dt);

            // *** FLS Calculation ***
            double fls_correction_x = 0.0;
            double fls_correction_y = 0.0;
            if (USE_FLS) { // Correctly uses the effective USE_FLS flag
                fls_correction_x = fls_x_controllers[i].calculateOutput(error_x, d_error_x, simulated_wind_x);
                fls_correction_y = fls_y_controllers[i].calculateOutput(error_y, d_error_y, simulated_wind_y);
            }
            // *** END FLS Calculation ***

            // *** Combine PID + FLS and Clamp ***
            double final_cmd_x = clamp(pid_terms_x.total_output + fls_correction_x, output_min, output_max);
            double final_cmd_y = clamp(pid_terms_y.total_output + fls_correction_y, output_min, output_max);
            // *** END Combine & Clamp ***

            drones[i].update(final_cmd_x, final_cmd_y, dt, simulated_wind_x, simulated_wind_y);

            // Store current error for next iteration's dError calculation
            drones[i].prev_error_x = error_x;
            drones[i].prev_error_y = error_y;

            // --- Update Metrics ---
            int metric_idx = current_phase - 1;
            // Ensure metric_idx is valid (0 to num_metric_phases-1)
            if (metric_idx < 0 || metric_idx >= num_metric_phases) {
                 // This case should ideally not happen if current_phase is managed well.
                 // Default to phase 0 if something is wrong, or log an error.
                 // For ZN tuning focusing on phase 1 (metric_idx 0), this is fine.
                 if (current_phase == 1) metric_idx = 0;
                 // else std::cerr << "Error: Invalid metric_idx " << metric_idx << " at time " << time_now << std::endl;
            }

            PerformanceMetrics& current_drone_metric_x = drone_metrics_x[i][metric_idx];
            PerformanceMetrics& current_drone_metric_y = drone_metrics_y[i][metric_idx];
            
            double current_target_x = target_values_x_per_phase[metric_idx][i];
            double current_target_y = target_values_y_per_phase[metric_idx][i];
            double initial_val_x = initial_values_x_per_phase[metric_idx][i];
            double initial_val_y = initial_values_y_per_phase[metric_idx][i];

            if (current_target_x > initial_val_x) {
                if (drones[i].position_x > current_drone_metric_x.peak_value) {
                    current_drone_metric_x.peak_value = drones[i].position_x;
                    current_drone_metric_x.peak_time = time_now;
                }
            } else {
                if (drones[i].position_x < current_drone_metric_x.peak_value) {
                    current_drone_metric_x.peak_value = drones[i].position_x;
                    current_drone_metric_x.peak_time = time_now;
                }
            }

            if (current_target_y > initial_val_y) {
                if (drones[i].position_y > current_drone_metric_y.peak_value) {
                    current_drone_metric_y.peak_value = drones[i].position_y;
                    current_drone_metric_y.peak_time = time_now;
                }
            } else {
                if (drones[i].position_y < current_drone_metric_y.peak_value) {
                    current_drone_metric_y.peak_value = drones[i].position_y;
                    current_drone_metric_y.peak_time = time_now;
                }
            }

            double settling_tol_x = std::abs(current_target_x * SETTLING_PERCENTAGE);
            if (settling_tol_x < 1e-3) settling_tol_x = 1e-3; // Minimum tolerance if target is near zero
            double settling_tol_y = std::abs(current_target_y * SETTLING_PERCENTAGE);
            if (settling_tol_y < 1e-3) settling_tol_y = 1e-3;

            if (std::abs(drones[i].position_x - current_target_x) <= settling_tol_x) {
                if (!in_settling_band_x[metric_idx][i]) {
                    time_entered_settling_x[metric_idx][i] = time_now;
                    in_settling_band_x[metric_idx][i] = true;
                }
                 // If it has stayed in the band, assign settling time (only once)
                if (current_drone_metric_x.settling_time_2percent < 0.0 && in_settling_band_x[metric_idx][i]) {
                    current_drone_metric_x.settling_time_2percent = time_entered_settling_x[metric_idx][i];
                }
            } else {
                if (in_settling_band_x[metric_idx][i]) { // Left the band after entering
                    current_drone_metric_x.settling_time_2percent = -1.0; // Reset settling time
                }
                in_settling_band_x[metric_idx][i] = false;
                time_entered_settling_x[metric_idx][i] = -1.0; // No longer in band
            }

            if (std::abs(drones[i].position_y - current_target_y) <= settling_tol_y) {
                if (!in_settling_band_y[metric_idx][i]) {
                    time_entered_settling_y[metric_idx][i] = time_now;
                    in_settling_band_y[metric_idx][i] = true;
                }
                if (current_drone_metric_y.settling_time_2percent < 0.0 && in_settling_band_y[metric_idx][i]) {
                    current_drone_metric_y.settling_time_2percent = time_entered_settling_y[metric_idx][i];
                }
            } else {
                 if (in_settling_band_y[metric_idx][i]) {
                    current_drone_metric_y.settling_time_2percent = -1.0;
                }
                in_settling_band_y[metric_idx][i] = false;
                time_entered_settling_y[metric_idx][i] = -1.0;
            }
            // --- End Update Metrics ---

            csv_file << "," << pid_x_controllers[i].getSetpoint() << "," << drones[i].position_x << "," << error_x
                     << "," << pid_terms_x.total_output << "," << fls_correction_x << "," << final_cmd_x
                     << "," << pid_terms_x.p << "," << pid_terms_x.i << "," << pid_terms_x.d
                     << "," << pid_y_controllers[i].getSetpoint() << "," << drones[i].position_y << "," << error_y
                     << "," << pid_terms_y.total_output << "," << fls_correction_y << "," << final_cmd_y
                     << "," << pid_terms_y.p << "," << pid_terms_y.i << "," << pid_terms_y.d;

            if (i==0 && static_cast<int>(time_now * 1000) % 1000 == 0) {
                 std::cout << "T=" << std::fixed << std::setprecision(1) << time_now 
                           << std::fixed << std::setprecision(3)
                           << " D0_X:" << drones[0].position_x << " D0_Y:" << drones[0].position_y
                           << " ErrX:" << error_x << " ErrY:" << error_y
                           << " FLS_X:" << fls_correction_x << " FLS_Y:" << fls_correction_y
                           << " WindX:" << simulated_wind_x << " WindY:" << simulated_wind_y << std::endl;
            }
        }
        csv_file << "," << simulated_wind_x << "," << simulated_wind_y;
        csv_file << "\n";
    } // End simulation loop

    // --- Final Metric Calculation & Printing ---
    std::cout << "\n=== FINAL PERFORMANCE METRICS ===\n";
    
    // Print controller configuration
    std::string config_details;
    std::ostringstream oss_config;
    oss_config << std::fixed << std::setprecision(3)
               << controller_type_log_msg << " Configuration:\n" // controller_type_log_msg is already set
               << "Kp=" << kp << ", Ki=" << ki << ", Kd=" << kd << "\n";
    config_details = oss_config.str();
    
    std::cout << "\n" << config_details << "\n";
    if (metrics_file.is_open()) {
        metrics_file << config_details << "\n";
    }
    
    // Final pass for settling times if system settled at the very end.
    for (int phase_idx_loop = 0; phase_idx_loop < num_metric_phases; ++phase_idx_loop) {
         if (!phase_triggered[phase_idx_loop + 1] && phase_idx_loop > 0) continue; // Skip phases not triggered, except phase 1 (idx 0)
         if (phase_idx_loop == 0 && !phase_triggered[1]) phase_triggered[1] = true; // Phase 1 always considered "triggered"

        for (int i = 0; i < NUM_DRONES; ++i) {
            // X-axis
            if (drone_metrics_x[i][phase_idx_loop].settling_time_2percent < 0 && in_settling_band_x[phase_idx_loop][i]) {
                drone_metrics_x[i][phase_idx_loop].settling_time_2percent = time_entered_settling_x[phase_idx_loop][i];
            }
            // Y-axis
            if (drone_metrics_y[i][phase_idx_loop].settling_time_2percent < 0 && in_settling_band_y[phase_idx_loop][i]) {
                drone_metrics_y[i][phase_idx_loop].settling_time_2percent = time_entered_settling_y[phase_idx_loop][i];
            }
        }
    }


    for (int phase_idx = 0; phase_idx < num_metric_phases; ++phase_idx) {
        if (!phase_triggered[phase_idx + 1] && phase_idx > 0) continue;
        if (phase_idx == 0 && !phase_triggered[1]) phase_triggered[1] = true;


        std::cout << "\n--- METRICS FOR PHASE " << phase_idx + 1 << " ---" << std::endl;
        if (metrics_file.is_open()) {
            metrics_file << "\n--- METRICS FOR PHASE " << phase_idx + 1 << " ---\n";
        }

        for (int i = 0; i < NUM_DRONES; ++i) {
            double initial_x = initial_values_x_per_phase[phase_idx][i];
            double target_x = target_values_x_per_phase[phase_idx][i];
            double initial_y = initial_values_y_per_phase[phase_idx][i];
            double target_y = target_values_y_per_phase[phase_idx][i];

            PerformanceMetrics& mets_x = drone_metrics_x[i][phase_idx];
            PerformanceMetrics& mets_y = drone_metrics_y[i][phase_idx];

            // Calculate overshoot percentage if peak exists and target is different from initial
            if (std::abs(target_x - initial_x) > 1e-6) { // Avoid division by zero
                if (target_x > initial_x) { // Moving in positive direction
                    mets_x.overshoot_percent = ((mets_x.peak_value - target_x) / (target_x - initial_x)) * 100.0;
                } else { // Moving in negative direction
                    mets_x.overshoot_percent = ((target_x - mets_x.peak_value) / (initial_x - target_x)) * 100.0;
                }
                if (mets_x.peak_value == initial_x && target_x != initial_x) mets_x.overshoot_percent = 0; // No movement towards target yet
                if (mets_x.overshoot_percent < 0) mets_x.overshoot_percent = 0; // No overshoot if it didn't pass target
            } else {
                mets_x.overshoot_percent = 0.0; // No change in setpoint, so no overshoot by definition
            }


            if (std::abs(target_y - initial_y) > 1e-6) {
                if (target_y > initial_y) {
                    mets_y.overshoot_percent = ((mets_y.peak_value - target_y) / (target_y - initial_y)) * 100.0;
                } else {
                    mets_y.overshoot_percent = ((target_y - mets_y.peak_value) / (initial_y - target_y)) * 100.0;
                }
                 if (mets_y.peak_value == initial_y && target_y != initial_y) mets_y.overshoot_percent = 0;
                 if (mets_y.overshoot_percent < 0) mets_y.overshoot_percent = 0;
            } else {
                mets_y.overshoot_percent = 0.0;
            }

            // Settling times are already updated.

            std::stringstream ss;
            ss << std::fixed << std::setprecision(3)
               << "Drone " << i << " Metrics (Target X:" << target_x << ", Y:" << target_y << "):\n"
               << "  X-axis:\n"
               << "    Peak Value: " << mets_x.peak_value << " at t=" << mets_x.peak_time << "s\n"
               << "    Overshoot: " << mets_x.overshoot_percent << "%\n"
               << "    Settling Time (2%): " << (mets_x.settling_time_2percent >= 0 ? 
                    std::to_string(mets_x.settling_time_2percent) + "s" : "Did not settle") << "\n"
               << "  Y-axis:\n"
               << "    Peak Value: " << mets_y.peak_value << " at t=" << mets_y.peak_time << "s\n"
               << "    Overshoot: " << mets_y.overshoot_percent << "%\n"
               << "    Settling Time (2%): " << (mets_y.settling_time_2percent >= 0 ? 
                    std::to_string(mets_y.settling_time_2percent) + "s" : "Did not settle") << "\n";

            std::cout << ss.str();
            if (metrics_file.is_open()) {
                metrics_file << ss.str();
            }
        }
    }

    if (metrics_file.is_open()) {
        metrics_file.close();
    }
    csv_file.close();

    std::cout << "\nSimulation complete. Data written to:\n"
              << "- CSV data: " << csv_path.string() << "\n";
    if (!ZN_TUNING_ACTIVE) {
        std::cout << "- Metrics: " << metrics_path.string() << "\n";
    }
    std::cout << "Controller type used: " << controller_type_log_msg 
              << " (Kp=" << kp << ", Ki=" << ki << ", Kd=" << kd << ")" << std::endl;
    return 0;
}