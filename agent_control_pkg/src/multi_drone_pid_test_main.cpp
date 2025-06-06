// agent_control_pkg/src/multi_drone_pid_test_main.cpp
#include "../include/agent_control_pkg/pid_controller.hpp"
#include <iostream>
#include <vector>
#include <iomanip>
#include <cmath>
#include <fstream>
#include <string>
#include <algorithm> // For std::clamp
#include <sstream>   // For std::ostringstream for filename

// --- FakeDrone Struct ---
struct FakeDrone {
    double position_x = 0.0;
    double position_y = 0.0;
    double velocity_x = 0.0;
    double velocity_y = 0.0;

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
    // --- Controller Configuration ---
    double kp = 3.1;   // Your best tuned Kp
    double ki = 0.4;   // Your best tuned Ki
    double kd = 2.2;   // Your best tuned Kd
    double output_min = -10.0;
    double output_max = 10.0;

    // --- Simulation Setup ---
    const int NUM_DRONES = 3;
    double dt = 0.05;
    double simulation_time = 120.0; // Longer for more phases

    // --- Test Scenario Flags ---
    bool ENABLE_WIND = true; // <-- SET TO true TO TEST WITH WIND
                               // <-- SET TO false FOR PID-ONLY NO-WIND BASELINE

    // --- Initial Drone Positions ---
    std::vector<FakeDrone> drones(NUM_DRONES);
    drones[0].position_x = 0.0; drones[0].position_y =  (sqrt(3.0)/3.0) * 4.0 - 2.0; // Approx triangle around (-2,0) initially
    drones[1].position_x = -2.0; drones[1].position_y = -(sqrt(3.0)/6.0) * 4.0 - 2.0;
    drones[2].position_x =  2.0; drones[2].position_y = -(sqrt(3.0)/6.0) * 4.0 - 2.0;
    // Or simpler start:
    // drones[0].position_x = -2.0; drones[0].position_y = 0.0;
    // drones[1].position_x =  0.0; drones[1].position_y = 0.0;
    // drones[2].position_x =  2.0; drones[2].position_y = 0.0;


    // --- PID Controllers ---
    std::vector<agent_control_pkg::PIDController> pid_x_controllers;
    std::vector<agent_control_pkg::PIDController> pid_y_controllers;
    for (int i = 0; i < NUM_DRONES; ++i) {
        pid_x_controllers.emplace_back(kp, ki, kd, output_min, output_max, drones[i].position_x); // Set initial setpoint to current pos
        pid_y_controllers.emplace_back(kp, ki, kd, output_min, output_max, drones[i].position_y);
    }

    // --- Formation Definition (Equilateral Triangle) ---
    double formation_side_length = 4.0;
    std::vector<std::pair<double, double>> formation_offsets(NUM_DRONES);
    formation_offsets[0] = {0.0, (sqrt(3.0) / 3.0) * formation_side_length};       // Top vertex
    formation_offsets[1] = {-formation_side_length / 2.0, -(sqrt(3.0) / 6.0) * formation_side_length}; // Bottom-left
    formation_offsets[2] = {formation_side_length / 2.0, -(sqrt(3.0) / 6.0) * formation_side_length};  // Bottom-right

    // --- Phase Management & Targets ---
    // Similar to agent_control_main.cpp's phase structure
    const int MAX_PHASES = 4;
    std::vector<double> phase_target_center_x = {  5.0, -5.0,  0.0,  5.0};
    std::vector<double> phase_target_center_y = {  5.0,  0.0, -5.0,  0.0};
    // Phase change times (start time of each phase)
    std::vector<double> phase_start_times = {0.0, 30.0, 60.0, 90.0};
    int current_phase_idx = 0; // 0-indexed

    // --- Performance Metrics Storage ---
    std::vector<std::vector<PerformanceMetrics>> drone_metrics_x(NUM_DRONES, std::vector<PerformanceMetrics>(MAX_PHASES));
    std::vector<std::vector<PerformanceMetrics>> drone_metrics_y(NUM_DRONES, std::vector<PerformanceMetrics>(MAX_PHASES));
    std::vector<bool> phase_has_been_active(MAX_PHASES, false);


    // --- File Naming & Output ---
    std::ostringstream oss_filename_suffix;
    oss_filename_suffix << "_Kp" << std::fixed << std::setprecision(2) << kp
                        << "_Ki" << ki << "_Kd" << kd
                        << (ENABLE_WIND ? "_WIND_ON" : "_WIND_OFF");
    std::string csv_filename = "multi_drone_pid_test" + oss_filename_suffix.str() + ".csv";
    std::ofstream csv_file(csv_filename);

    csv_file << "Time";
    for (int i = 0; i < NUM_DRONES; ++i) {
        csv_file << ",TargetX" << i << ",CurrentX" << i << ",ErrorX" << i << ",PIDOutX" << i << ",PTermX" << i << ",ITermX" << i << ",DTermX" << i
                 << ",TargetY" << i << ",CurrentY" << i << ",ErrorY" << i << ",PIDOutY" << i << ",PTermY" << i << ",ITermY" << i << ",DTermY" << i;
    }
    if (ENABLE_WIND) csv_file << ",SimWindX,SimWindY";
    csv_file << "\n";

    std::cout << "Starting Multi-Drone PID Test..." << std::endl;
    std::cout << " Gains: Kp=" << kp << ", Ki=" << ki << ", Kd=" << kd << std::endl;
    std::cout << " Wind: " << (ENABLE_WIND ? "ON" : "OFF") << std::endl;
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
        if (ENABLE_WIND) {
            int phase_for_wind = current_phase_idx + 1; // 1-based phase
            if (phase_for_wind == 1) { // Phase 1 Winds (5-20s relative to phase start, but here using global time for simplicity)
                if (time_now >= 5.0 && time_now < 15.0) simulated_wind_x = 3.0;
                if (time_now >= 10.0 && time_now < 20.0) simulated_wind_y = -1.0;
            } else if (phase_for_wind == 2) { // Phase 2 Winds (35-50s)
                if (time_now >= 35.0 && time_now < 45.0) simulated_wind_x = -2.0;
                if (time_now >= 40.0 && time_now < 50.0) simulated_wind_y = 1.5;
            } else if (phase_for_wind == 3) { // Phase 3 Winds (65-75s)
                if (time_now >= 65.0 && time_now < 70.0) simulated_wind_x = 1.0 * sin(time_now * 2.0);
                if (time_now >= 70.0 && time_now < 72.0) simulated_wind_y = 4.0;
                else if (time_now >= 72.0 && time_now < 75.0) simulated_wind_y = -2.0;
            } else if (phase_for_wind == 4) { // Phase 4 Winds (95-105s)
                if (time_now >= 95.0 && time_now < 105.0) { simulated_wind_x = 2.5; simulated_wind_y = -2.5; }
            }
        }

        // --- Control and Update Loop ---
        csv_file << time_now;
        for (int i = 0; i < NUM_DRONES; ++i) {
            double error_x = pid_x_controllers[i].getSetpoint() - drones[i].position_x;
            agent_control_pkg::PIDController::PIDTerms terms_x = pid_x_controllers[i].calculate_with_terms(drones[i].position_x, dt);
            double cmd_x = terms_x.total_output;

            double error_y = pid_y_controllers[i].getSetpoint() - drones[i].position_y;
            agent_control_pkg::PIDController::PIDTerms terms_y = pid_y_controllers[i].calculate_with_terms(drones[i].position_y, dt);
            double cmd_y = terms_y.total_output;

            drones[i].update(cmd_x, cmd_y, dt, simulated_wind_x, simulated_wind_y);

            drone_metrics_x[i][current_phase_idx].update_metrics(drones[i].position_x, time_now);
            drone_metrics_y[i][current_phase_idx].update_metrics(drones[i].position_y, time_now);

            csv_file << "," << pid_x_controllers[i].getSetpoint() << "," << drones[i].position_x << "," << error_x << "," << cmd_x
                     << "," << terms_x.p << "," << terms_x.i << "," << terms_x.d
                     << "," << pid_y_controllers[i].getSetpoint() << "," << drones[i].position_y << "," << error_y << "," << cmd_y
                     << "," << terms_y.p << "," << terms_y.i << "," << terms_y.d;
        }
        if (ENABLE_WIND) csv_file << "," << simulated_wind_x << "," << simulated_wind_y;
        csv_file << "\n";

        // Console output for Drone 0
        if (static_cast<int>(time_now * 1000) % 5000 == 0 && time_now > 0.1) { // Every 5 seconds
            std::cout << "T=" << std::fixed << std::setprecision(1) << time_now
                      << " Ph:" << current_phase_idx + 1
                      << " D0_Pos:(" << std::fixed << std::setprecision(2) << drones[0].position_x << "," << drones[0].position_y << ")"
                      << " D0_Tgt:(" << pid_x_controllers[0].getSetpoint() << "," << pid_y_controllers[0].getSetpoint() << ")"
                      << (ENABLE_WIND ? " Wind:(" + std::to_string(simulated_wind_x) + "," + std::to_string(simulated_wind_y) + ")" : "")
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