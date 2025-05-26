// agent_control_main.cpp with FLS Integration and Full Metrics

#include "agent_control_pkg/pid_controller.hpp"         // Your PID class
#include "agent_control_pkg/gt2_fuzzy_logic_system.hpp" // Your FLS class
#include <iostream>
#include <vector>
#include <iomanip>
#include <cmath>     // For sqrt, abs
#include <fstream>   // For file I/O
#include <limits>    // For std::numeric_limits
#include <string>    // For std::to_string
#include <algorithm> // For std::clamp

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

    void update(double accel_cmd_x, double accel_cmd_y, double dt) {
        velocity_x += accel_cmd_x * dt;
        velocity_y += accel_cmd_y * dt;
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

// *** ADDED Helper function to initialize an FLS instance ***
void setup_fls_instance(agent_control_pkg::GT2FuzzyLogicSystem& fls) {
    using FOU = agent_control_pkg::GT2FuzzyLogicSystem::IT2TriangularFS_FOU;

    // This is called for each FLS (X and Y for each drone)
    // Ensure variable names here match what FLS expects internally
    fls.addInputVariable("error");    // FLS will expect this name for the error input
    fls.addInputVariable("dError");   // FLS will expect this name for the change in error
    fls.addInputVariable("wind");     // FLS will expect this name for the wind input
    fls.addOutputVariable("correction"); // FLS will produce an output named "correction"

    // Define Fuzzy Sets for "error"
    fls.addFuzzySetToVariable("error", "NB", FOU{-10.0, -8.0, -6.0,  -11.0, -8.5, -5.5});
    fls.addFuzzySetToVariable("error", "NS", FOU{ -7.0, -4.0, -1.0,   -8.0, -4.5,  0.0});
    fls.addFuzzySetToVariable("error", "ZE", FOU{ -1.0,  0.0,  1.0,   -2.0,  0.0,  2.0});
    fls.addFuzzySetToVariable("error", "PS", FOU{  1.0,  4.0,  7.0,    0.0,  4.5,  8.0});
    fls.addFuzzySetToVariable("error", "PB", FOU{  6.0,  8.0, 10.0,    5.5,  8.5, 11.0});

    // Define Fuzzy Sets for "dError"
    // Note: Your previous main used "NE", "ZE", "PO". The FLS class might expect "DN", "DZ", "DP".
    // Let's use DN, DZ, DP to match your FLS class test and rule list.
    fls.addFuzzySetToVariable("dError", "DN", FOU{-5.0, -3.0,  0.0,   -6.0, -3.5,  1.0});
    fls.addFuzzySetToVariable("dError", "DZ", FOU{-1.0,  0.0,  1.0,   -1.5,  0.0,  1.5});
    fls.addFuzzySetToVariable("dError", "DP", FOU{ 0.0,  3.0,  5.0,   -1.0,  3.0,  6.0});

    // Define Fuzzy Sets for "wind"
    // Using your original set names from the FLS test: SNW, WNW, NWN, WPW, SPW
    fls.addFuzzySetToVariable("wind", "SNW", FOU{-10.0, -8.0, -5.0,  -11.0, -8.5, -4.5});
    fls.addFuzzySetToVariable("wind", "WNW", FOU{ -7.0, -4.0, -1.0,   -8.0, -4.5,  0.0});
    fls.addFuzzySetToVariable("wind", "NWN", FOU{ -1.0,  0.0,  1.0,   -1.5,  0.0,  1.5});
    fls.addFuzzySetToVariable("wind", "WPW", FOU{  1.0,  4.0,  7.0,    0.0,  4.5,  8.0});
    fls.addFuzzySetToVariable("wind", "SPW", FOU{  6.0,  8.0, 10.0,    5.5,  8.5, 11.0});

    // Define Fuzzy Sets for "correction" (Output)
    fls.addFuzzySetToVariable("correction", "XLNC",FOU{-7.5, -6.0, -4.5,  -8.5, -6.5, -4.0});
    fls.addFuzzySetToVariable("correction", "LNC", FOU{-5.0, -4.0, -2.5,  -5.5, -4.5, -2.0});
    fls.addFuzzySetToVariable("correction", "SNC", FOU{-3.0, -1.5,  0.0,  -3.5, -1.5,  0.5});
    fls.addFuzzySetToVariable("correction", "NC",  FOU{-0.5,  0.0,  0.5,  -1.0,  0.0,  1.0});
    fls.addFuzzySetToVariable("correction", "SPC", FOU{ 0.0,  1.5,  3.0,  -0.5,  1.5,  3.5});
    fls.addFuzzySetToVariable("correction", "LPC", FOU{ 2.5,  4.0,  5.0,   2.0,  4.5,  5.5});
    fls.addFuzzySetToVariable("correction", "XLPC",FOU{ 4.5,  6.0,  7.5,   4.0,  6.5,  8.5});


    // Define Rules (using your 21 rules, adjust consequents for wind if needed)
    auto R = [&](const std::string& e, const std::string& de, const std::string& w, const std::string& out){
        fls.addRule({{{"error",e},{"dError",de},{"wind",w}}, {"correction",out}});
    };
        // No wind (NWN) - Damping rules & basic error correction
    R("PB","DZ","NWN","LPC"); R("PS","DZ","NWN","SPC"); R("ZE","DZ","NWN","NC");  R("NS","DZ","NWN","SNC"); R("NB","DZ","NWN","LNC");
    R("PB","DN","NWN","LPC"); R("PS","DN","NWN","SPC"); R("ZE","DN","NWN","SNC"); // MODIFIED: Was NC, now SNC for damping
    R("NS","DN","NWN","NC");  R("NB","DN","NWN","SNC");
    R("PB","DP","NWN","NC");  R("PS","DP","NWN","SNC"); R("ZE","DP","NWN","SPC"); // MODIFIED: Was NC, now SPC for damping
    R("NS","DP","NWN","SNC"); R("NB","DP","NWN","LNC");
  // Wind scenarios
    // When error is ZE (Zero Error) and dError is DZ (Zero Change in Error)
    // MODIFIED: Stronger direct counteraction for wind
    R("ZE","DZ","SPW","LNC");  // Strong Positive Wind -> Large Negative Correction (was SNC in your original)
    R("ZE","DZ","SNW","LPC");  // Strong Negative Wind -> Large Positive Correction (was SPC in your original)
    // MODIFIED: Weaker direct counteraction for weak wind (original was SNC/SPC, keeping those)
    R("ZE","DZ","WPW","SNC");  // Weak Positive Wind -> Small Negative Correction
    R("ZE","DZ","WNW","SPC");  // Weak Negative Wind -> Small Positive Correction

    // Rules when error is present WITH wind (These were your original, keeping them for now,
    // but they might need tuning based on how wind interacts with an existing error)
    R("PB","DZ","SPW","XLPC");
    R("NB","DZ","SNW","XLNC");
}
// *** END Added Helper Function ***


int main() {
    const int NUM_DRONES = 3;
    std::vector<FakeDrone> drones(NUM_DRONES);
    std::vector<agent_control_pkg::PIDController> pid_x_controllers;
    std::vector<agent_control_pkg::PIDController> pid_y_controllers;
    // *** FLS INSTANCES ***
    std::vector<agent_control_pkg::GT2FuzzyLogicSystem> fls_x_controllers(NUM_DRONES);
    std::vector<agent_control_pkg::GT2FuzzyLogicSystem> fls_y_controllers(NUM_DRONES);
    // *** END FLS INSTANCES ***

    double kp = 1.5; // Your tuned gains
    double ki = 0.05;
    double kd = 1.2;
    double output_min = -10.0;
    double output_max = 10.0;

    // --- Performance Metric Storage (same as your last working version) ---
    std::vector<std::vector<PerformanceMetrics>> drone_metrics_x(NUM_DRONES, std::vector<PerformanceMetrics>(2));
    std::vector<std::vector<PerformanceMetrics>> drone_metrics_y(NUM_DRONES, std::vector<PerformanceMetrics>(2));
    std::vector<double> initial_values_x_phase1(NUM_DRONES), initial_values_y_phase1(NUM_DRONES);
    std::vector<double> target_values_x_phase1(NUM_DRONES), target_values_y_phase1(NUM_DRONES);
    std::vector<double> initial_values_x_phase2(NUM_DRONES), initial_values_y_phase2(NUM_DRONES);
    std::vector<double> target_values_x_phase2(NUM_DRONES), target_values_y_phase2(NUM_DRONES);
    std::vector<bool> in_settling_band_x_phase1(NUM_DRONES, false), in_settling_band_y_phase1(NUM_DRONES, false);
    std::vector<bool> in_settling_band_x_phase2(NUM_DRONES, false), in_settling_band_y_phase2(NUM_DRONES, false);
    std::vector<double> time_entered_settling_x_phase1(NUM_DRONES, -1.0), time_entered_settling_y_phase1(NUM_DRONES, -1.0);
    std::vector<double> time_entered_settling_x_phase2(NUM_DRONES, -1.0), time_entered_settling_y_phase2(NUM_DRONES, -1.0);
    bool phase2_active = false;
    const double SETTLING_PERCENTAGE = 0.02;
    // --- End Metric Storage ---

    // Initialize drones, PIDs, FLSs, and initial metrics
    for (int i = 0; i < NUM_DRONES; ++i) {
        drones[i].position_x = static_cast<double>(i) * 2.0 - 2.0;
        drones[i].position_y = 0.0;
        pid_x_controllers.emplace_back(kp, ki, kd, output_min, output_max, 0.0);
        pid_y_controllers.emplace_back(kp, ki, kd, output_min, output_max, 0.0);

        // *** SETUP FLS INSTANCES ***
        setup_fls_instance(fls_x_controllers[i]);
        setup_fls_instance(fls_y_controllers[i]); // Using same MFs/Rules for Y-axis FLS for now
        // *** END SETUP FLS ***

        initial_values_x_phase1[i] = drones[i].position_x;
        initial_values_y_phase1[i] = drones[i].position_y;
        drone_metrics_x[i][0].reset(initial_values_x_phase1[i]);
        drone_metrics_y[i][0].reset(initial_values_y_phase1[i]);
    }

    // Define Formation (same as before)
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
    double simulation_time = 60.0; // Extended simulation time

    // *** Simulated Wind Conditions ***
    double simulated_wind_x = 0.0;
    double simulated_wind_y = 0.0;
    // *** End Wind ***

    std::string csv_file_name = "multi_drone_pid_fls_sim_data.csv"; // New CSV name
    std::ofstream csv_file(csv_file_name);
    if (!csv_file.is_open()) {
        std::cerr << "Error opening CSV file: " << csv_file_name << std::endl;
        return 1;
    }

    // --- UPDATED CSV HEADER ---
    csv_file << "Time";
    for (int i = 0; i < NUM_DRONES; ++i) {
        csv_file << ",TargetX" << i << ",CurrentX" << i << ",ErrorX" << i
                 << ",PIDOutX" << i << ",FLSCorrX" << i << ",FinalCmdX" << i // Added FLS
                 << ",PTermX" << i << ",ITermX" << i << ",DTermX" << i
                 << ",TargetY" << i << ",CurrentY" << i << ",ErrorY" << i
                 << ",PIDOutY" << i << ",FLSCorrY" << i << ",FinalCmdY" << i // Added FLS
                 << ",PTermY" << i << ",ITermY" << i << ",DTermY" << i;
    }
    csv_file << ",SimWindX,SimWindY\n"; // Added wind
    // --- END UPDATED CSV HEADER ---

    std::cout << std::fixed << std::setprecision(3);
    // No need to initialize prev_error here, will be set in first iteration of drone loop

    // --- Simulation Loop ---
    for (double time_now = 0.0; time_now <= simulation_time; time_now += dt) {
        // --- Apply Wind Disturbances (Example Profile) ---
        if (time_now >= 5.0 && time_now < 15.0) {
            simulated_wind_x = 3.0;
        } else if (time_now >= 35.0 && time_now < 45.0) { // Second wind phase for X
            simulated_wind_x = -2.0;
        } else {
            simulated_wind_x = 0.0;
        }

        if (time_now >= 10.0 && time_now < 20.0) {
            simulated_wind_y = -1.0;
        } else if (time_now >= 40.0 && time_now < 50.0) { // Second wind phase for Y
            simulated_wind_y = 1.5;
        } else {
            simulated_wind_y = 0.0;
        }
        // --- End Wind Disturbances ---

        // Target change (around 30s)
        if (time_now > 29.9 && time_now < 30.1 && !phase2_active) {
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
                drones[i].prev_error_x = 0.0; // Reset dError history for new phase
                drones[i].prev_error_y = 0.0;
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
            double fls_correction_x = fls_x_controllers[i].calculateOutput(error_x, d_error_x, simulated_wind_x);
            double fls_correction_y = fls_y_controllers[i].calculateOutput(error_y, d_error_y, simulated_wind_y);
            // *** END FLS Calculation ***

            // *** Combine PID + FLS and Clamp ***
            double final_cmd_x = std::clamp(pid_terms_x.total_output + fls_correction_x, output_min, output_max);
            double final_cmd_y = std::clamp(pid_terms_y.total_output + fls_correction_y, output_min, output_max);
            // *** END Combine & Clamp ***

            drones[i].update(final_cmd_x, final_cmd_y, dt);

            // Store current error for next iteration's dError calculation
            drones[i].prev_error_x = error_x;
            drones[i].prev_error_y = error_y;

            // --- Update Metrics (same logic as your working version) ---
            int phase_idx = phase2_active ? 1 : 0;
            // (Copy your working metric update logic here for X and Y, for drone_metrics_x[i][phase_idx] and drone_metrics_y[i][phase_idx])
            // For X-axis
            double current_target_x = phase2_active ? target_values_x_phase2[i] : target_values_x_phase1[i];
            double initial_val_x    = phase2_active ? initial_values_x_phase2[i] : initial_values_x_phase1[i];
            PerformanceMetrics& current_drone_metric_x = drone_metrics_x[i][phase_idx];
            if (current_target_x > initial_val_x) { if (drones[i].position_x > current_drone_metric_x.peak_value) { current_drone_metric_x.peak_value = drones[i].position_x; current_drone_metric_x.peak_time = time_now; }}
            else { if (drones[i].position_x < current_drone_metric_x.peak_value) { current_drone_metric_x.peak_value = drones[i].position_x; current_drone_metric_x.peak_time = time_now; }}
            double settling_tol_x = std::abs(current_target_x * SETTLING_PERCENTAGE);
            std::vector<bool>& current_in_band_vec_x_ref = phase2_active ? in_settling_band_x_phase2 : in_settling_band_x_phase1;
            std::vector<double>& current_time_entered_vec_x_ref = phase2_active ? time_entered_settling_x_phase2 : time_entered_settling_x_phase1;
            if (std::abs(drones[i].position_x - current_target_x) <= settling_tol_x) {
                if (!current_in_band_vec_x_ref[i]) { current_time_entered_vec_x_ref[i] = time_now; current_in_band_vec_x_ref[i] = true; }
                if (current_drone_metric_x.settling_time_2percent < 0 && current_in_band_vec_x_ref[i]) { current_drone_metric_x.settling_time_2percent = current_time_entered_vec_x_ref[i]; }
            } else {
                if (current_in_band_vec_x_ref[i]) { current_drone_metric_x.settling_time_2percent = -1.0; } current_in_band_vec_x_ref[i] = false;
            }
            // For Y-axis
            double current_target_y = phase2_active ? target_values_y_phase2[i] : target_values_y_phase1[i];
            double initial_val_y    = phase2_active ? initial_values_y_phase2[i] : initial_values_y_phase1[i];
            PerformanceMetrics& current_drone_metric_y = drone_metrics_y[i][phase_idx];
            std::vector<bool>& current_in_band_vec_y_ref = phase2_active ? in_settling_band_y_phase2 : in_settling_band_y_phase1;
            std::vector<double>& current_time_entered_vec_y_ref = phase2_active ? time_entered_settling_y_phase2 : time_entered_settling_y_phase1;
            if (current_target_y > initial_val_y) { if (drones[i].position_y > current_drone_metric_y.peak_value) { current_drone_metric_y.peak_value = drones[i].position_y; current_drone_metric_y.peak_time = time_now; }}
            else { if (drones[i].position_y < current_drone_metric_y.peak_value) { current_drone_metric_y.peak_value = drones[i].position_y; current_drone_metric_y.peak_time = time_now; }}
            double settling_tol_y = std::abs(current_target_y * SETTLING_PERCENTAGE);
            if (std::abs(drones[i].position_y - current_target_y) <= settling_tol_y) {
                if (!current_in_band_vec_y_ref[i]) { current_time_entered_vec_y_ref[i] = time_now; current_in_band_vec_y_ref[i] = true; }
                 if (current_drone_metric_y.settling_time_2percent < 0 && current_in_band_vec_y_ref[i]) { current_drone_metric_y.settling_time_2percent = current_time_entered_vec_y_ref[i]; }
            } else {
                 if (current_in_band_vec_y_ref[i]) { current_drone_metric_y.settling_time_2percent = -1.0; } current_in_band_vec_y_ref[i] = false;
            }
            // --- End Update Metrics ---


            // --- MODIFIED CSV DATA ROW ---
            csv_file << "," << pid_x_controllers[i].getSetpoint() << "," << drones[i].position_x << "," << error_x
                     << "," << pid_terms_x.total_output << "," << fls_correction_x << "," << final_cmd_x
                     << "," << pid_terms_x.p << "," << pid_terms_x.i << "," << pid_terms_x.d
                     << "," << pid_y_controllers[i].getSetpoint() << "," << drones[i].position_y << "," << error_y
                     << "," << pid_terms_y.total_output << "," << fls_correction_y << "," << final_cmd_y
                     << "," << pid_terms_y.p << "," << pid_terms_y.i << "," << pid_terms_y.d;

            // Console output for Drone 0 (less frequent)
            if (i==0 && static_cast<int>(time_now * 1000) % 1000 == 0) { // Every 1 second for Drone 0
                 std::cout << "T=" << time_now 
                           << " D0_X:" << drones[0].position_x << " D0_Y:" << drones[0].position_y
                           << " FLS_X:" << fls_correction_x << " FLS_Y:" << fls_correction_y
                           << " WindX:" << simulated_wind_x << " WindY:" << simulated_wind_y << std::endl;
            }
        }
        // Add wind data to CSV once per time step, at the end of the row
        csv_file << "," << simulated_wind_x << "," << simulated_wind_y;
        csv_file << "\n";

    } // End simulation loop

    // ... (CSV close, Metrics file open, Final Metric Calculation & Printing as before) ...
    // (This part of your code should mostly work, just ensure filenames and variables are consistent)
    csv_file.close();
    std::cout << "\nSimulation data written to " << csv_file_name << std::endl;

    std::ofstream metrics_file("pid_fls_performance_metrics.txt");
    if (!metrics_file.is_open()) {
        std::cerr << "Error opening metrics file!" << std::endl;
    } else {
        metrics_file << std::fixed << std::setprecision(3);
        metrics_file << "PID+FLS Performance Metrics (Kp=" << kp << ", Ki=" << ki << ", Kd=" << kd << ")\n";
        metrics_file << "=======================================================================\n";
    }

    for (int phase_idx = 0; phase_idx < 2; ++phase_idx) {
        if (phase_idx == 1 && !phase2_active && simulation_time < 30.1) continue; // Adjusted target change time
        // ... (The rest of your metric calculation and printing logic, ensure variable names match) ...
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
            if (step_mag_x > 1e-6) { mets_x.overshoot_percent = (target_x > initial_x) ? ((mets_x.peak_value - target_x) / step_mag_x) * 100.0 : ((target_x - mets_x.peak_value) / step_mag_x) * 100.0;
                if ((target_x > initial_x && mets_x.peak_value < target_x) || (target_x < initial_x && mets_x.peak_value > target_x) || mets_x.overshoot_percent < 0) { mets_x.overshoot_percent = 0.0; }
            } else { mets_x.overshoot_percent = 0.0; }
            double step_mag_y = std::abs(target_y - initial_y);
            if (step_mag_y > 1e-6) { mets_y.overshoot_percent = (target_y > initial_y) ? ((mets_y.peak_value - target_y) / step_mag_y) * 100.0 : ((target_y - mets_y.peak_value) / step_mag_y) * 100.0;
                if ((target_y > initial_y && mets_y.peak_value < target_y) || (target_y < initial_y && mets_y.peak_value > target_y) || mets_y.overshoot_percent < 0) { mets_y.overshoot_percent = 0.0; }
            } else { mets_y.overshoot_percent = 0.0; }
            
            std::string x_settling_str = (mets_x.settling_time_2percent >=0 ? std::to_string(mets_x.settling_time_2percent) + "s" : "Not Settled");
            std::string y_settling_str = (mets_y.settling_time_2percent >=0 ? std::to_string(mets_y.settling_time_2percent) + "s" : "Not Settled");

            std::cout << "\n  --- Drone " << i << " (Phase " << phase_idx + 1 << ") ---" << std::endl;
            std::cout << "    X-Axis (Target: " << target_x << " from " << initial_x << "):" << std::endl;
            std::cout << "      Peak: " << mets_x.peak_value << " at T=" << mets_x.peak_time << "s, Overshoot: " << mets_x.overshoot_percent << "%" << std::endl;
            std::cout << "      Settling Time (2%): " << x_settling_str << std::endl;
            std::cout << "    Y-Axis (Target: " << target_y << " from " << initial_y << "):" << std::endl;
            std::cout << "      Peak: " << mets_y.peak_value << " at T=" << mets_y.peak_time << "s, Overshoot: " << mets_y.overshoot_percent << "%" << std::endl;
            std::cout << "      Settling Time (2%): " << y_settling_str << std::endl;

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