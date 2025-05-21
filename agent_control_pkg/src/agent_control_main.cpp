#include "agent_control_pkg/pid_controller.hpp" // Your updated PID class
#include <iostream>
#include <vector>
#include <iomanip>
#include <cmath> // For sqrt
#include <fstream>
#include <limits> // For std::numeric_limits to initialize peak values
#include <string> // For std::to_string

// FakeDrone struct (remains the same)
struct FakeDrone {
    double position_x;
    double position_y;
    double velocity_x;
    double velocity_y;

    FakeDrone() : position_x(0.0), position_y(0.0), velocity_x(0.0), velocity_y(0.0) {}

    void update(double accel_cmd_x, double accel_cmd_y, double dt) {
        velocity_x += accel_cmd_x * dt;
        velocity_y += accel_cmd_y * dt;
        velocity_x *= 0.98; // Simple drag
        velocity_y *= 0.98; // Simple drag
        position_x += velocity_x * dt;
        position_y += velocity_y * dt;
    }
};

// PerformanceMetrics struct (remains the same)
struct PerformanceMetrics {
    double peak_value = 0.0;
    double peak_time = 0.0;
    double overshoot_percent = 0.0;
    double settling_time_2percent = -1.0; 
    void reset(double initial_value_for_peak) {
        peak_value = initial_value_for_peak;
        peak_time = 0.0;
        overshoot_percent = 0.0;
        settling_time_2percent = -1.0;
    }
};

int main() {
    const int NUM_DRONES = 3;
    std::vector<FakeDrone> drones(NUM_DRONES);
    std::vector<agent_control_pkg::PIDController> pid_x_controllers;
    std::vector<agent_control_pkg::PIDController> pid_y_controllers;

    double kp = 1.5; 
    double ki = 0.05;
    double kd = 1.2;
    double output_min = -10.0;
    double output_max = 10.0;

    std::vector<std::vector<PerformanceMetrics>> drone_metrics_x(NUM_DRONES, std::vector<PerformanceMetrics>(2));
    std::vector<std::vector<PerformanceMetrics>> drone_metrics_y(NUM_DRONES, std::vector<PerformanceMetrics>(2));

    std::vector<double> initial_values_x_phase1(NUM_DRONES);
    std::vector<double> initial_values_y_phase1(NUM_DRONES);
    std::vector<double> target_values_x_phase1(NUM_DRONES);
    std::vector<double> target_values_y_phase1(NUM_DRONES);
    
    std::vector<double> initial_values_x_phase2(NUM_DRONES);
    std::vector<double> initial_values_y_phase2(NUM_DRONES);
    std::vector<double> target_values_x_phase2(NUM_DRONES);
    std::vector<double> target_values_y_phase2(NUM_DRONES);

    std::vector<bool> in_settling_band_x_phase1(NUM_DRONES, false);
    std::vector<bool> in_settling_band_y_phase1(NUM_DRONES, false);
    std::vector<bool> in_settling_band_x_phase2(NUM_DRONES, false);
    std::vector<bool> in_settling_band_y_phase2(NUM_DRONES, false);
    
    std::vector<double> time_entered_settling_x_phase1(NUM_DRONES, -1.0);
    std::vector<double> time_entered_settling_y_phase1(NUM_DRONES, -1.0);
    std::vector<double> time_entered_settling_x_phase2(NUM_DRONES, -1.0);
    std::vector<double> time_entered_settling_y_phase2(NUM_DRONES, -1.0);

    bool phase2_active = false;
    const double SETTLING_PERCENTAGE = 0.02;

    for (int i = 0; i < NUM_DRONES; ++i) {
        drones[i].position_x = static_cast<double>(i) * 2.0 - 2.0;
        drones[i].position_y = 0.0; 
        pid_x_controllers.emplace_back(kp, ki, kd, output_min, output_max, 0.0);
        pid_y_controllers.emplace_back(kp, ki, kd, output_min, output_max, 0.0);
        
        initial_values_x_phase1[i] = drones[i].position_x;
        initial_values_y_phase1[i] = drones[i].position_y;
        drone_metrics_x[i][0].reset(initial_values_x_phase1[i]);
        drone_metrics_y[i][0].reset(initial_values_y_phase1[i]);
    }

    double formation_center_x = 5.0;
    double formation_center_y = 5.0;
    double side_length = 4.0;

    std::vector<std::pair<double, double>> formation_offsets(NUM_DRONES);
    formation_offsets[0] = {0.0, (sqrt(3.0) / 2.0) * side_length * (2.0/3.0)};
    formation_offsets[1] = {-side_length / 2.0, -(sqrt(3.0) / 2.0) * side_length * (1.0/3.0)};
    formation_offsets[2] = {side_length / 2.0, -(sqrt(3.0) / 2.0) * side_length * (1.0/3.0)};

    for (int i = 0; i < NUM_DRONES; ++i) {
        target_values_x_phase1[i] = formation_center_x + formation_offsets[i].first;
        target_values_y_phase1[i] = formation_center_y + formation_offsets[i].second;
        pid_x_controllers[i].setSetpoint(target_values_x_phase1[i]);
        pid_y_controllers[i].setSetpoint(target_values_y_phase1[i]);
    }

    double dt = 0.05;
    double simulation_time = 60.0;

    std::ofstream csv_file("multi_drone_sim_data.csv");
    if (!csv_file.is_open()) { /* ... error handling ... */ return 1; }
    csv_file << "Time"; // ... (rest of CSV header as before) ...
    for (int i = 0; i < NUM_DRONES; ++i) {
        csv_file << ",TargetX" << i << ",CurrentX" << i << ",VelX" << i << ",ErrorX" << i << ",PIDOutX" << i
                 << ",PTermX" << i << ",ITermX" << i << ",DTermX" << i 
                 << ",TargetY" << i << ",CurrentY" << i << ",VelY" << i << ",ErrorY" << i << ",PIDOutY" << i
                 << ",PTermY" << i << ",ITermY" << i << ",DTermY" << i; 
    }
    csv_file << "\n";

    std::cout << std::fixed << std::setprecision(3);

    for (double time_now = 0.0; time_now <= simulation_time; time_now += dt) {
        if (time_now > 29.9 && time_now < 30.1 && !phase2_active) {
            // ... (Phase 2 activation logic as before, including resetting metrics) ...
            std::cout << "---- Changing FORMATION CENTER to (-5.0, 0.0) ----\n";
            formation_center_x = -5.0;
            formation_center_y = 0.0;
            phase2_active = true; 
            for (int i = 0; i < NUM_DRONES; ++i) {
                target_values_x_phase2[i] = formation_center_x + formation_offsets[i].first;
                target_values_y_phase2[i] = formation_center_y + formation_offsets[i].second;
                pid_x_controllers[i].setSetpoint(target_values_x_phase2[i]);
                pid_y_controllers[i].setSetpoint(target_values_y_phase2[i]);

                initial_values_x_phase2[i] = drones[i].position_x;
                initial_values_y_phase2[i] = drones[i].position_y;
                drone_metrics_x[i][1].reset(initial_values_x_phase2[i]);
                drone_metrics_y[i][1].reset(initial_values_y_phase2[i]);
                in_settling_band_x_phase2[i] = false; time_entered_settling_x_phase2[i] = -1.0;
                in_settling_band_y_phase2[i] = false; time_entered_settling_y_phase2[i] = -1.0;
            }
        }

        csv_file << time_now;

        for (int i = 0; i < NUM_DRONES; ++i) {
            agent_control_pkg::PIDController::PIDTerms terms_x = pid_x_controllers[i].calculate_with_terms(drones[i].position_x, dt);
            agent_control_pkg::PIDController::PIDTerms terms_y = pid_y_controllers[i].calculate_with_terms(drones[i].position_y, dt);
            
            drones[i].update(terms_x.total_output, terms_y.total_output, dt);

            // --- Update Metrics for drone i ---
            int phase_idx = phase2_active ? 1 : 0;
            double current_target_x = phase2_active ? target_values_x_phase2[i] : target_values_x_phase1[i];
            double initial_val_x    = phase2_active ? initial_values_x_phase2[i] : initial_values_x_phase1[i];
            PerformanceMetrics& current_drone_metric_x = drone_metrics_x[i][phase_idx];
            // ** MODIFIED BOOL HANDLING **
            std::vector<bool>& current_in_band_vec_x = phase2_active ? in_settling_band_x_phase2 : in_settling_band_x_phase1;
            std::vector<double>& current_time_entered_vec_x = phase2_active ? time_entered_settling_x_phase2 : time_entered_settling_x_phase1;

            double current_target_y = phase2_active ? target_values_y_phase2[i] : target_values_y_phase1[i];
            double initial_val_y    = phase2_active ? initial_values_y_phase2[i] : initial_values_y_phase1[i];
            PerformanceMetrics& current_drone_metric_y = drone_metrics_y[i][phase_idx];
            // ** MODIFIED BOOL HANDLING **
            std::vector<bool>& current_in_band_vec_y = phase2_active ? in_settling_band_y_phase2 : in_settling_band_y_phase1;
            std::vector<double>& current_time_entered_vec_y = phase2_active ? time_entered_settling_y_phase2 : time_entered_settling_y_phase1;

            // X-axis metrics update
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
            double settling_tol_x = std::abs(current_target_x * SETTLING_PERCENTAGE);
            if (std::abs(drones[i].position_x - current_target_x) <= settling_tol_x) {
                if (!current_in_band_vec_x[i]) { // Read from vector
                    current_time_entered_vec_x[i] = time_now; 
                    current_in_band_vec_x[i] = true; // Assign to vector
                }
                if (current_drone_metric_x.settling_time_2percent < 0 && current_in_band_vec_x[i]) {
                     current_drone_metric_x.settling_time_2percent = current_time_entered_vec_x[i];
                }
            } else {
                if (current_in_band_vec_x[i]) { current_drone_metric_x.settling_time_2percent = -1.0; }
                current_in_band_vec_x[i] = false; // Assign to vector
            }

            // Y-axis metrics update (similar direct vector access)
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
            double settling_tol_y = std::abs(current_target_y * SETTLING_PERCENTAGE);
            if (std::abs(drones[i].position_y - current_target_y) <= settling_tol_y) {
                if (!current_in_band_vec_y[i]) { 
                    current_time_entered_vec_y[i] = time_now; 
                    current_in_band_vec_y[i] = true; 
                }
                 if (current_drone_metric_y.settling_time_2percent < 0 && current_in_band_vec_y[i]) {
                     current_drone_metric_y.settling_time_2percent = current_time_entered_vec_y[i];
                }
            } else {
                 if (current_in_band_vec_y[i]) { current_drone_metric_y.settling_time_2percent = -1.0; }
                current_in_band_vec_y[i] = false;
            }
            // --- End Update Metrics ---

            double error_x = pid_x_controllers[i].getSetpoint() - drones[i].position_x;
            double error_y = pid_y_controllers[i].getSetpoint() - drones[i].position_y;

            // ... (CSV writing for terms_x, terms_y as before) ...
            csv_file << "," << pid_x_controllers[i].getSetpoint() << "," << drones[i].position_x << "," << drones[i].velocity_x << "," << error_x << "," << terms_x.total_output
                     << "," << terms_x.p << "," << terms_x.i << "," << terms_x.d
                     << "," << pid_y_controllers[i].getSetpoint() << "," << drones[i].position_y << "," << drones[i].velocity_y << "," << error_y << "," << terms_y.total_output
                     << "," << terms_y.p << "," << terms_y.i << "," << terms_y.d;
        
            if (i==0 && static_cast<int>(time_now * 1000) % 500 == 0) { 
                 std::cout << "T=" << time_now << " D0_X:" << drones[0].position_x << " D0_Y:" << drones[0].position_y << std::endl;
            }
        }
        csv_file << "\n";
    } // End simulation loop

    // ... (CSV close and metrics file open as before) ...
    csv_file.close();
    std::cout << "\nSimulation data written to multi_drone_sim_data.csv" << std::endl;

    std::ofstream metrics_file("pid_performance_metrics.txt");
    if (!metrics_file.is_open()) { /* ... error handling ... */ }
    else {
        metrics_file << std::fixed << std::setprecision(3);
        metrics_file << "PID Performance Metrics (Kp=" << kp << ", Ki=" << ki << ", Kd=" << kd << ")\n";
        metrics_file << "=============================================================\n";
    }


    // ... (Final metric calculation and printing to console AND metrics_file as before) ...
    // ... (Ensure this part correctly uses the new vector access if it was also using references before) ...
    // For brevity, I'm not repeating the full print section, but ensure it uses
    // drone_metrics_x[i][phase_idx], drone_metrics_y[i][phase_idx], etc. correctly.

    for (int phase_idx = 0; phase_idx < 2; ++phase_idx) {
        if (phase_idx == 1 && !phase2_active) continue; 

        std::cout << "\n--- METRICS FOR PHASE " << phase_idx + 1 << " ---" << std::endl;
        if (metrics_file.is_open()) metrics_file << "\n--- METRICS FOR PHASE " << phase_idx + 1 << " ---\n";

        for (int i = 0; i < NUM_DRONES; ++i) {
            double initial_x = (phase_idx == 0) ? initial_values_x_phase1[i] : initial_values_x_phase2[i];
            double target_x  = (phase_idx == 0) ? target_values_x_phase1[i] : target_values_x_phase2[i];
            double initial_y = (phase_idx == 0) ? initial_values_y_phase1[i] : initial_values_y_phase2[i];
            double target_y  = (phase_idx == 0) ? target_values_y_phase1[i] : target_values_y_phase2[i]; // Corrected this line

            PerformanceMetrics& mets_x = drone_metrics_x[i][phase_idx]; // Use reference
            PerformanceMetrics& mets_y = drone_metrics_y[i][phase_idx]; // Use reference

            double step_mag_x = std::abs(target_x - initial_x);
            if (step_mag_x > 1e-6) {
                mets_x.overshoot_percent = (target_x > initial_x) ? 
                    ((mets_x.peak_value - target_x) / step_mag_x) * 100.0 :
                    ((target_x - mets_x.peak_value) / step_mag_x) * 100.0;
            } else { mets_x.overshoot_percent = 0.0; }

            double step_mag_y = std::abs(target_y - initial_y);
            if (step_mag_y > 1e-6) {
                 mets_y.overshoot_percent = (target_y > initial_y) ?
                    ((mets_y.peak_value - target_y) / step_mag_y) * 100.0 :
                    ((target_y - mets_y.peak_value) / step_mag_y) * 100.0;
            } else { mets_y.overshoot_percent = 0.0; }
            
            std::string x_settling_str = (mets_x.settling_time_2percent >=0 ? std::to_string(mets_x.settling_time_2percent) + "s" : "Not Settled");
            std::string y_settling_str = (mets_y.settling_time_2percent >=0 ? std::to_string(mets_y.settling_time_2percent) + "s" : "Not Settled");

            // Console Output
            std::cout << "\n  --- Drone " << i << " (Phase " << phase_idx + 1 << ") ---" << std::endl;
            std::cout << "    X-Axis (Target: " << target_x << " from " << initial_x << "):" << std::endl;
            std::cout << "      Peak: " << mets_x.peak_value << " at T=" << mets_x.peak_time << "s, Overshoot: " << mets_x.overshoot_percent << "%" << std::endl;
            std::cout << "      Settling Time (2%): " << x_settling_str << std::endl;
            std::cout << "    Y-Axis (Target: " << target_y << " from " << initial_y << "):" << std::endl;
            std::cout << "      Peak: " << mets_y.peak_value << " at T=" << mets_y.peak_time << "s, Overshoot: " << mets_y.overshoot_percent << "%" << std::endl;
            std::cout << "      Settling Time (2%): " << y_settling_str << std::endl;

            // File Output
            if (metrics_file.is_open()) {
                metrics_file << "\n  --- Drone " << i << " (Phase " << phase_idx + 1 << ") ---\n";
                metrics_file << "    X-Axis (Target: " << target_x << " from " << initial_x << "):\n";
                metrics_file << "      Peak: " << mets_x.peak_value << " at T=" << mets_x.peak_time << "s, Overshoot: " << mets_x.overshoot_percent << "%\n";
                metrics_file << "      Settling Time (2%): " << x_settling_str << "\n";
                metrics_file << "    Y-Axis (Target: " << target_y << " from " << initial_y << "):\n";
                metrics_file << "      Peak: " << mets_y.peak_value << " at T=" << mets_y.peak_time << "s, Overshoot: " << mets_y.overshoot_percent << "%\n";
                metrics_file << "      Settling Time (2%): " << y_settling_str << "\n";
            }
        }
    }


    if (metrics_file.is_open()) {
        metrics_file.close();
        std::cout << "\nPerformance metrics also written to pid_performance_metrics.txt" << std::endl;
    }
    return 0;
}