// agent_control_pkg/src/simple_pid_tuning_main.cpp
#include "../include/agent_control_pkg/pid_controller.hpp"
#include <iostream>
#include <vector>
#include <iomanip>
#include <cmath>
#include <fstream>
#include <string>
#include <algorithm> // For std::clamp

// --- FakeDrone Struct (Simplified: no prev_error for FLS needed here) ---
struct FakeDrone {
    double position_x = 0.0;
    double position_y = 0.0; // Keep Y for potential 2D testing, but start with 1D
    double velocity_x = 0.0;
    double velocity_y = 0.0;

    void update(double accel_cmd_x, double accel_cmd_y, double dt) { // No external forces
        velocity_x += accel_cmd_x * dt;
        velocity_y += accel_cmd_y * dt;
        velocity_x *= 0.98; // Simple drag
        velocity_y *= 0.98; // Simple drag
        position_x += velocity_x * dt;
        position_y += velocity_y * dt;
    }
};

// --- PerformanceMetrics Struct (Can be simplified or kept as is) ---
struct PerformanceMetrics {
    double peak_value = 0.0;
    double peak_time = 0.0;
    double overshoot_percent = 0.0;
    double settling_time_2percent = -1.0;
    double initial_value_for_peak = 0.0; // Store initial val for overshoot calc
    double target_value = 0.0;           // Store target val for overshoot/settling
    bool in_settling_band = false;
    double time_entered_settling = -1.0;

    void reset(double initial_val, double target_val_for_metrics) {
        peak_value = initial_val; // Start peak at initial value
        peak_time = 0.0;
        overshoot_percent = 0.0;
        settling_time_2percent = -1.0;
        initial_value_for_peak = initial_val;
        target_value = target_val_for_metrics;
        in_settling_band = false;
        time_entered_settling = -1.0;
    }

    void update(double current_value, double time_now, double dt) {
        // Peak Detection (handles both positive and negative steps)
        if (target_value > initial_value_for_peak) { // Moving positive
            if (current_value > peak_value) {
                peak_value = current_value;
                peak_time = time_now;
            }
        } else { // Moving negative (or target is same as initial)
            if (current_value < peak_value) {
                peak_value = current_value;
                peak_time = time_now;
            }
        }

        // Settling Time (2% criterion)
        const double SETTLING_PERCENTAGE = 0.02;
        double settling_tol = std::abs(target_value * SETTLING_PERCENTAGE);
        settling_tol = (settling_tol < 1e-3 && std::abs(target_value) > 1e-9) ? std::abs(target_value * 0.1) : std::max(settling_tol, 1e-3); // Use 10% if target is very small but not zero


        if (std::abs(current_value - target_value) <= settling_tol) {
            if (!in_settling_band) {
                time_entered_settling = time_now;
                in_settling_band = true;
            }
            // If it has stayed in the band, assign settling time (only once)
            // This logic is slightly simplified: it takes the first time it enters.
            // For robust settling time, you'd check if it STAYS in the band for some duration.
            // For this initial tuning, first entry is often good enough.
            if (settling_time_2percent < 0.0 && in_settling_band) {
                 settling_time_2percent = time_entered_settling;
            }
        } else {
            if (in_settling_band) { // Left the band after entering
                settling_time_2percent = -1.0; // Reset settling time if it leaves
            }
            in_settling_band = false;
            time_entered_settling = -1.0;
        }
    }

    void finalize_metrics() {
        if (std::abs(target_value - initial_value_for_peak) > 1e-6) {
            if (target_value > initial_value_for_peak) {
                overshoot_percent = ((peak_value - target_value) / (target_value - initial_value_for_peak)) * 100.0;
            } else {
                overshoot_percent = ((target_value - peak_value) / (initial_value_for_peak - target_value)) * 100.0;
            }
            if (peak_value == initial_value_for_peak && target_value != initial_value_for_peak) overshoot_percent = 0.0;
            if (overshoot_percent < 0) overshoot_percent = 0.0;
        } else {
            overshoot_percent = 0.0;
        }

        // If it was still in settling band at the end of sim
        if (settling_time_2percent < 0.0 && in_settling_band) {
            settling_time_2percent = time_entered_settling;
        }
    }
};


// --- Custom clamp function ---
template<typename T>
T clamp(T value, T min_val, T max_val) {
    if (value < min_val) return min_val;
    if (value > max_val) return max_val;
    return value;
}

int main() {
    // --- PID GAINS - MODIFY THESE FOR TUNING ---
    double kp = 3.10;  // Start with values from your previous detuning or lower
    double ki = 0.4;
    double kd = 2.2;
    // --- END PID GAINS ---

    double output_min = -10.0;
    double output_max = 10.0;

    // --- Simulation Parameters ---
    const int NUM_DRONES = 1; // Start with one drone for simplicity
    double dt = 0.05;         // Simulation time step
    double simulation_time = 30.0; // Shorter simulation for quick tests

    // --- Target ---
    double initial_pos_x = 0.0;
    double target_pos_x = 5.0; // Simple step input
    // double initial_pos_y = 0.0; // If testing 2D
    // double target_pos_y = 0.0;  // If testing 2D

    // --- Initialize Drone(s) and PID Controller(s) ---
    std::vector<FakeDrone> drones(NUM_DRONES);
    std::vector<agent_control_pkg::PIDController> pid_x_controllers;
    // std::vector<agent_control_pkg::PIDController> pid_y_controllers; // If testing 2D
    std::vector<PerformanceMetrics> metrics_x(NUM_DRONES);
    // std::vector<PerformanceMetrics> metrics_y(NUM_DRONES); // If testing 2D


    for (int i = 0; i < NUM_DRONES; ++i) {
        drones[i].position_x = initial_pos_x;
        // drones[i].position_y = initial_pos_y; // If testing 2D

        pid_x_controllers.emplace_back(kp, ki, kd, output_min, output_max, target_pos_x);
        // pid_y_controllers.emplace_back(kp, ki, kd, output_min, output_max, target_pos_y); // If testing 2D
        
        metrics_x[i].reset(initial_pos_x, target_pos_x);
        // metrics_y[i].reset(initial_pos_y, target_pos_y); // If testing 2D
    }

    // --- File Naming & Output ---
    // Create a unique filename for each run (optional, or just overwrite)
    std::string csv_filename = "simple_pid_tune_kp" + std::to_string(kp) +
                               "_ki" + std::to_string(ki) +
                               "_kd" + std::to_string(kd) + ".csv";
    std::ofstream csv_file(csv_filename);
    if (!csv_file.is_open()) {
        std::cerr << "Error opening CSV file: " << csv_filename << std::endl;
        return 1;
    }

    // Write CSV header
    csv_file << "Time,TargetX0,CurrentX0,ErrorX0,PIDOutX0,PTermX0,ITermX0,DTermX0\n"; // Simplified for 1 drone

    std::cout << "Starting simple PID tuning simulation..." << std::endl;
    std::cout << "Kp=" << kp << ", Ki=" << ki << ", Kd=" << kd << std::endl;
    std::cout << "Outputting to: " << csv_filename << std::endl;

    // --- Simulation Loop ---
    for (double time_now = 0.0; time_now <= simulation_time; time_now += dt) {
        csv_file << time_now;

        for (int i = 0; i < NUM_DRONES; ++i) {
            // X-axis control
            double error_x = pid_x_controllers[i].getSetpoint() - drones[i].position_x;
            agent_control_pkg::PIDController::PIDTerms pid_terms_x = pid_x_controllers[i].calculate_with_terms(drones[i].position_x, dt);
            double final_cmd_x = pid_terms_x.total_output; // Already clamped by PID controller

            // Y-axis control (dummy for now if NUM_DRONES=1 and only X target changes)
            double final_cmd_y = 0.0;
            // if (test_2d) {
            //    double error_y = pid_y_controllers[i].getSetpoint() - drones[i].position_y;
            //    agent_control_pkg::PIDController::PIDTerms pid_terms_y = pid_y_controllers[i].calculate_with_terms(drones[i].position_y, dt);
            //    final_cmd_y = pid_terms_y.total_output;
            // }

            drones[i].update(final_cmd_x, final_cmd_y, dt); // No wind

            // Update metrics
            metrics_x[i].update(drones[i].position_x, time_now, dt);

            // Write to CSV
            csv_file << "," << pid_x_controllers[i].getSetpoint() << "," << drones[i].position_x << "," << error_x
                     << "," << pid_terms_x.total_output
                     << "," << pid_terms_x.p << "," << pid_terms_x.i << "," << pid_terms_x.d;

            // Print to console occasionally
            if (i == 0 && static_cast<int>(time_now * 1000) % 1000 == 0) { // Every second
                std::cout << "T=" << std::fixed << std::setprecision(1) << time_now
                          << " X0_Pos:" << std::fixed << std::setprecision(3) << drones[0].position_x
                          << " X0_Err:" << error_x
                          << " X0_Cmd:" << final_cmd_x << std::endl;
            }
        }
        csv_file << "\n";
    }
    csv_file.close();

    // --- Finalize and Print Metrics ---
    std::cout << "\n--- FINAL METRICS ---" << std::endl;
    for (int i = 0; i < NUM_DRONES; ++i) {
        metrics_x[i].finalize_metrics();
        std::cout << "Drone " << i << " X-axis:" << std::endl;
        std::cout << "  Peak Value: " << metrics_x[i].peak_value << " at t=" << metrics_x[i].peak_time << "s" << std::endl;
        std::cout << "  Overshoot: " << metrics_x[i].overshoot_percent << "%" << std::endl;
        std::cout << "  Settling Time (2%): "
                  << (metrics_x[i].settling_time_2percent >= 0 ? std::to_string(metrics_x[i].settling_time_2percent) + "s" : "Did not settle")
                  << std::endl;
    }

    std::cout << "\nSimple PID tuning simulation complete. Data in: " << csv_filename << std::endl;
    return 0;
}