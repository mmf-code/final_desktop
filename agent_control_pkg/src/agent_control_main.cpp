#include "agent_control_pkg/pid_controller.hpp" // Your updated PID class
#include "agent_control_pkg/gt2_fuzzy_logic_system.hpp" // Include FLS header
#include <iostream>
#include <vector>
#include <iomanip>
#include <cmath> // For sqrt, abs
#include <fstream>
#include <limits> // For std::numeric_limits to initialize peak values
#include <string> // For std::to_string
#include <algorithm> // For std::clamp

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

// Helper function to initialize an FLS instance with your sets and rules
// This avoids repeating the setup code for each FLS object
void setup_fls_instance(agent_control_pkg::GT2FuzzyLogicSystem& fls) {
    // Define Input/Output Variables
    fls.addInputVariable("error");
    fls.addInputVariable("dError");
    fls.addInputVariable("wind");
    fls.addOutputVariable("correction");

    // Define Fuzzy Sets for "error" (Example: 5 sets)
    fls.addFuzzySetToVariable("error", "NB", {-10.0, -10.0, -5.0,  -11.0, -10.0, -4.0});
    fls.addFuzzySetToVariable("error", "NS", {-6.0, -3.0, 0.0,    -7.0, -3.0, 1.0});
    fls.addFuzzySetToVariable("error", "ZE", {-1.0, 0.0, 1.0,    -2.0, 0.0, 2.0});
    fls.addFuzzySetToVariable("error", "PS", {0.0, 3.0, 6.0,     -1.0, 3.0, 7.0});
    fls.addFuzzySetToVariable("error", "PB", {5.0, 10.0, 10.0,   4.0, 10.0, 11.0});

    // Define Fuzzy Sets for "dError" (Example: 3 sets)
    fls.addFuzzySetToVariable("dError", "NE", {-5.0, -5.0, 0.0,    -6.0, -5.0, 1.0});
    fls.addFuzzySetToVariable("dError", "ZE", {-1.0, 0.0, 1.0,    -1.5, 0.0, 1.5});
    fls.addFuzzySetToVariable("dError", "PO", {0.0, 5.0, 5.0,     -1.0, 5.0, 6.0});

    // Define Fuzzy Sets for "wind" (Example: 5 sets)
    fls.addFuzzySetToVariable("wind", "SN", {-10.0, -10.0, -5.0,  -11.0, -10.0, -4.0}); // Strong Negative
    fls.addFuzzySetToVariable("wind", "WN", {-6.0, -3.0, 0.0,    -7.0, -3.0, 1.0});    // Weak Negative
    fls.addFuzzySetToVariable("wind", "NW", {-1.0, 0.0, 1.0,    -1.5, 0.0, 1.5});    // No Wind
    fls.addFuzzySetToVariable("wind", "WP", {0.0, 3.0, 6.0,     -1.0, 3.0, 7.0});    // Weak Positive
    fls.addFuzzySetToVariable("wind", "SP", {5.0, 10.0, 10.0,   4.0, 10.0, 11.0});   // Strong Positive

    // Define Fuzzy Sets for "correction" (Output - Example: 5 sets)
    fls.addFuzzySetToVariable("correction", "LNC", {-5.0, -5.0, -2.5,   -5.5, -5.0, -2.0}); // Large Negative
    fls.addFuzzySetToVariable("correction", "SNC", {-3.0, -1.5, 0.0,    -3.5, -1.5, 0.5});  // Small Negative
    fls.addFuzzySetToVariable("correction", "NC",  {-0.5, 0.0, 0.5,    -1.0, 0.0, 1.0});   // No Correction
    fls.addFuzzySetToVariable("correction", "SPC", {0.0, 1.5, 3.0,     -0.5, 1.5, 3.5});  // Small Positive
    fls.addFuzzySetToVariable("correction", "LPC", {2.5, 5.0, 5.0,      2.0, 5.0, 5.5});  // Large Positive

    // Define Rules (These are illustrative, you'll need a comprehensive set)
    // Rule 1: If no error, no dError, no wind -> no correction
    fls.addRule({ {{"error", "ZE"}, {"dError", "ZE"}, {"wind", "NW"}}, {"correction", "NC"} });
    // Rule 2: If on target, but strong positive wind (headwind) -> make large positive correction to PID output
    fls.addRule({ {{"error", "ZE"}, {"dError", "ZE"}, {"wind", "SP"}}, {"correction", "LPC"} });
    // Rule 3: If on target, but strong negative wind (tailwind) -> make large negative correction
    fls.addRule({ {{"error", "ZE"}, {"dError", "ZE"}, {"wind", "SN"}}, {"correction", "LNC"} });
    // Rule 4: If positive small error, no wind -> small negative correction
    fls.addRule({ {{"error", "PS"}, {"dError", "ZE"}, {"wind", "NW"}}, {"correction", "SNC"} });
    // Rule 5: If negative small error, no wind -> small positive correction
    fls.addRule({ {{"error", "NS"}, {"dError", "ZE"}, {"wind", "NW"}}, {"correction", "SPC"} });
    // Add more rules for other combinations, especially involving dError and wind affecting errors
}


int main() {
    const int NUM_DRONES = 3;
    std::vector<FakeDrone> drones(NUM_DRONES);
    std::vector<agent_control_pkg::PIDController> pid_x_controllers;
    std::vector<agent_control_pkg::PIDController> pid_y_controllers;
    // --- FLS Instances ---
    std::vector<agent_control_pkg::GT2FuzzyLogicSystem> fls_x_controllers(NUM_DRONES);
    std::vector<agent_control_pkg::GT2FuzzyLogicSystem> fls_y_controllers(NUM_DRONES);
    // --- End FLS Instances ---

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

    // Initialize drones, PIDs, FLSs, and initial metrics
    for (int i = 0; i < NUM_DRONES; ++i) {
        drones[i].position_x = static_cast<double>(i) * 2.0 - 2.0;
        drones[i].position_y = 0.0;
        pid_x_controllers.emplace_back(kp, ki, kd, output_min, output_max, 0.0);
        pid_y_controllers.emplace_back(kp, ki, kd, output_min, output_max, 0.0);

        // Setup FLS instances
        setup_fls_instance(fls_x_controllers[i]);
        setup_fls_instance(fls_y_controllers[i]);

        initial_values_x_phase1[i] = drones[i].position_x;
        initial_values_y_phase1[i] = drones[i].position_y;
        drone_metrics_x[i][0].reset(initial_values_x_phase1[i]);
        drone_metrics_y[i][0].reset(initial_values_y_phase1[i]);
        // No need to reset in_settling_band or time_entered_settling here as they are initialized above
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
    double simulation_time = 60.0; // Total simulation time

    // --- Simulated Wind ---
    double simulated_wind_x = 0.0;
    double simulated_wind_y = 0.0;
    // --- End Simulated Wind ---

    std::string csv_file_name = "multi_drone_pid_fls_sim_data.csv"; // New CSV name
    std::ofstream csv_file(csv_file_name);
    if (!csv_file.is_open()) {
        std::cerr << "Error opening CSV file: " << csv_file_name << std::endl;
        return 1;
    }

    // --- MODIFIED CSV HEADER (add FLS terms and wind) ---
    csv_file << "Time";
    for (int i = 0; i < NUM_DRONES; ++i) {
        csv_file << ",TargetX" << i << ",CurrentX" << i << ",ErrorX" << i
                 << ",PIDOutX" << i << ",FLSCorrX" << i << ",FinalCmdX" << i // PID, FLS, Final for X
                 << ",PTermX" << i << ",ITermX" << i << ",DTermX" << i
                 << ",TargetY" << i << ",CurrentY" << i << ",ErrorY" << i
                 << ",PIDOutY" << i << ",FLSCorrY" << i << ",FinalCmdY" << i // PID, FLS, Final for Y
                 << ",PTermY" << i << ",ITermY" << i << ",DTermY" << i;
    }
    // Add wind to CSV header (once per timestep)
    csv_file << ",SimWindX,SimWindY";
    csv_file << "\n";
    // --- END MODIFIED CSV HEADER ---

    std::cout << std::fixed << std::setprecision(3);
    std::vector<double> prev_error_x(NUM_DRONES, 0.0); // For calculating dError
    std::vector<double> prev_error_y(NUM_DRONES, 0.0); // For calculating dError

    // --- Simulation Loop ---
    for (double time_now = 0.0; time_now <= simulation_time; time_now += dt) {
        // --- Simulate Wind Change ---
        if (time_now > 5.0 && time_now <= 15.0) { // Wind active between 5s and 15s (inclusive of 15.0)
            simulated_wind_x = 3.0;
            simulated_wind_y = -1.0;
        } else {
            simulated_wind_x = 0.0;
            simulated_wind_y = 0.0;
        }
        // --- End Simulate Wind Change ---

        // Target change around 15s (Original request)
        // The original base code had target change at 30s. Using 15s as per FLS modification request.
        if (time_now > 14.9 && time_now < 15.1 && !phase2_active) {
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
                prev_error_x[i] = 0.0; // Reset dError history for new phase
                prev_error_y[i] = 0.0;
            }
        }

        csv_file << time_now;

        for (int i = 0; i < NUM_DRONES; ++i) {
            // Calculate current errors
            double error_x = pid_x_controllers[i].getSetpoint() - drones[i].position_x;
            double error_y = pid_y_controllers[i].getSetpoint() - drones[i].position_y;

            // Calculate dError (simple numerical derivative)
            double d_error_x = (dt > 1e-9) ? (error_x - prev_error_x[i]) / dt : 0.0; // Avoid division by zero if dt is tiny
            double d_error_y = (dt > 1e-9) ? (error_y - prev_error_y[i]) / dt : 0.0;

            // PID Calculation (gets raw PID output and terms)
            agent_control_pkg::PIDController::PIDTerms pid_terms_x = pid_x_controllers[i].calculate_with_terms(drones[i].position_x, dt);
            agent_control_pkg::PIDController::PIDTerms pid_terms_y = pid_y_controllers[i].calculate_with_terms(drones[i].position_y, dt);

            // FLS Calculation
            double fls_correction_x = fls_x_controllers[i].calculateOutput(error_x, d_error_x, simulated_wind_x);
            double fls_correction_y = fls_y_controllers[i].calculateOutput(error_y, d_error_y, simulated_wind_y);

            // Combine and Clamp
            double final_cmd_x = std::clamp(pid_terms_x.total_output + fls_correction_x, output_min, output_max);
            double final_cmd_y = std::clamp(pid_terms_y.total_output + fls_correction_y, output_min, output_max);

            drones[i].update(final_cmd_x, final_cmd_y, dt);

            // Store current error for next iteration's dError calculation
            prev_error_x[i] = error_x;
            prev_error_y[i] = error_y;

            // --- Update Metrics for drone i ---
            int phase_idx = phase2_active ? 1 : 0;
            double current_target_x = phase2_active ? target_values_x_phase2[i] : target_values_x_phase1[i];
            double initial_val_x    = phase2_active ? initial_values_x_phase2[i] : initial_values_x_phase1[i];
            PerformanceMetrics& current_drone_metric_x = drone_metrics_x[i][phase_idx];
            std::vector<bool>& current_in_band_vec_x = phase2_active ? in_settling_band_x_phase2 : in_settling_band_x_phase1;
            std::vector<double>& current_time_entered_vec_x = phase2_active ? time_entered_settling_x_phase2 : time_entered_settling_x_phase1;

            double current_target_y = phase2_active ? target_values_y_phase2[i] : target_values_y_phase1[i];
            double initial_val_y    = phase2_active ? initial_values_y_phase2[i] : initial_values_y_phase1[i];
            PerformanceMetrics& current_drone_metric_y = drone_metrics_y[i][phase_idx];
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
                if (!current_in_band_vec_x[i]) {
                    current_time_entered_vec_x[i] = time_now;
                    current_in_band_vec_x[i] = true;
                }
                if (current_drone_metric_x.settling_time_2percent < 0 && current_in_band_vec_x[i]) {
                     current_drone_metric_x.settling_time_2percent = current_time_entered_vec_x[i];
                }
            } else {
                if (current_in_band_vec_x[i]) { current_drone_metric_x.settling_time_2percent = -1.0; }
                current_in_band_vec_x[i] = false;
            }

            // Y-axis metrics update
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

            // --- MODIFIED CSV DATA ROW ---
            csv_file << "," << pid_x_controllers[i].getSetpoint() << "," << drones[i].position_x << "," << error_x
                     << "," << pid_terms_x.total_output << "," << fls_correction_x << "," << final_cmd_x
                     << "," << pid_terms_x.p << "," << pid_terms_x.i << "," << pid_terms_x.d
                     << "," << pid_y_controllers[i].getSetpoint() << "," << drones[i].position_y << "," << error_y
                     << "," << pid_terms_y.total_output << "," << fls_correction_y << "," << final_cmd_y
                     << "," << pid_terms_y.p << "," << pid_terms_y.i << "," << pid_terms_y.d;

            if (i==0 && static_cast<int>(time_now * 1000) % 500 == 0) { // Print status every 0.5s for Drone 0
                 std::cout << "T=" << time_now << " D0_X:" << drones[0].position_x << " D0_Y:" << drones[0].position_y
                           << " ErrX:" << error_x << " ErrY:" << error_y
                           << " FLS_X:" << fls_correction_x << " FLS_Y:" << fls_correction_y
                           << " WindX:" << simulated_wind_x << " WindY:" << simulated_wind_y << std::endl;
            }
        }
        // Add wind data to CSV once per time step
        csv_file << "," << simulated_wind_x << "," << simulated_wind_y;
        csv_file << "\n";

    } // End simulation loop

    csv_file.close();
    std::cout << "\nSimulation data written to " << csv_file_name << std::endl;

    std::ofstream metrics_file("pid_fls_performance_metrics.txt"); // Changed metrics file name slightly
    if (!metrics_file.is_open()) {
        std::cerr << "Error opening metrics file!" << std::endl;
    } else {
        metrics_file << std::fixed << std::setprecision(3);
        metrics_file << "PID+FLS Performance Metrics (Kp=" << kp << ", Ki=" << ki << ", Kd=" << kd << ")\n";
        metrics_file << "=======================================================================\n";
    }

    for (int phase_idx = 0; phase_idx < 2; ++phase_idx) {
        if (phase_idx == 1 && !phase2_active && simulation_time < 15.1) continue; // Skip phase 2 if not activated or sim ended before

        std::cout << "\n--- METRICS FOR PHASE " << phase_idx + 1 << " ---" << std::endl;
        if (metrics_file.is_open()) metrics_file << "\n--- METRICS FOR PHASE " << phase_idx + 1 << " ---\n";

        for (int i = 0; i < NUM_DRONES; ++i) {
            double initial_x = (phase_idx == 0) ? initial_values_x_phase1[i] : initial_values_x_phase2[i];
            double target_x  = (phase_idx == 0) ? target_values_x_phase1[i] : target_values_x_phase2[i];
            double initial_y = (phase_idx == 0) ? initial_values_y_phase1[i] : initial_values_y_phase2[i];
            double target_y  = (phase_idx == 0) ? target_values_y_phase1[i] : target_values_y_phase2[i];

            PerformanceMetrics& mets_x = drone_metrics_x[i][phase_idx];
            PerformanceMetrics& mets_y = drone_metrics_y[i][phase_idx];

            double step_mag_x = std::abs(target_x - initial_x);
            if (step_mag_x > 1e-6) { // Avoid division by zero if target equals initial
                mets_x.overshoot_percent = (target_x > initial_x) ?
                    ((mets_x.peak_value - target_x) / step_mag_x) * 100.0 :
                    ((target_x - mets_x.peak_value) / step_mag_x) * 100.0; // Peak is more 'extreme' than target
                 // Ensure overshoot is non-negative
                if ((target_x > initial_x && mets_x.peak_value < target_x) || (target_x < initial_x && mets_x.peak_value > target_x)) {
                    mets_x.overshoot_percent = 0.0; // No overshoot if peak didn't pass target
                }
            } else { mets_x.overshoot_percent = 0.0; }


            double step_mag_y = std::abs(target_y - initial_y);
            if (step_mag_y > 1e-6) {
                 mets_y.overshoot_percent = (target_y > initial_y) ?
                    ((mets_y.peak_value - target_y) / step_mag_y) * 100.0 :
                    ((target_y - mets_y.peak_value) / step_mag_y) * 100.0;
                if ((target_y > initial_y && mets_y.peak_value < target_y) || (target_y < initial_y && mets_y.peak_value > target_y)) {
                    mets_y.overshoot_percent = 0.0;
                }
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
        std::cout << "\nPerformance metrics also written to pid_fls_performance_metrics.txt" << std::endl;
    }
    return 0;
}