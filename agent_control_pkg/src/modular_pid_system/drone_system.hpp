#pragma once

#include "../include/agent_control_pkg/config_reader.hpp"
#include "../include/agent_control_pkg/gt2_fuzzy_logic_system.hpp"
#include "../include/agent_control_pkg/pid_controller.hpp"
#include <array>
#include <vector>
#include <map>
#include <string>

namespace modular_pid_system {

// System Configuration Structure
struct SystemConfig {
    // Control Features - Enable/Disable flags
    bool enable_pid = true;
    bool enable_wind = false;
    bool enable_fuzzy_logic = false;
    
    // Test Scenarios
    enum class TestScenario {
        PID_ONLY,           // Pure PID control
        PID_WITH_WIND,      // PID + Wind disturbance
        PID_WIND_FLC        // PID + Wind + Fuzzy Logic Controller
    };
    TestScenario current_scenario = TestScenario::PID_ONLY;
    
    // Simulation Parameters
    int num_drones = 3;
    double simulation_time = 60.0;
    double dt = 0.01;
    
    // Output Configuration
    bool enable_csv_output = true;
    bool enable_console_output = true;
    bool enable_metrics_analysis = true;
    std::string output_directory = "simulation_outputs";
    
    // Test Name for file naming
    std::string test_name = "modular_test";
};

// Drone State Structure
struct DroneState {
    double position_x = 0.0;
    double position_y = 0.0;
    double velocity_x = 0.0;
    double velocity_y = 0.0;
    double prev_error_x_fls = 0.0;
    double prev_error_y_fls = 0.0;
    
    void update(double accel_cmd_x, double accel_cmd_y, double dt,
                double external_force_x = 0.0, double external_force_y = 0.0);
    void reset(double init_x = 0.0, double init_y = 0.0);
};

// Performance Metrics Structure
struct PerformanceMetrics {
    double peak_value = 0.0;
    double peak_time = 0.0;
    double overshoot_percent = 0.0;
    double settling_time_2percent = -1.0;
    double initial_value = 0.0;
    double target_value = 0.0;
    bool is_active = false;
    
    void reset(double initial_val, double target_val);
    void update(double current_value, double time_now);
    void finalize(double phase_start_time);
};

// Wind Disturbance Model
struct WindModel {
    double wind_x = 0.0;
    double wind_y = 0.0;
    bool is_sine_wave = false;
    double sine_frequency = 1.0;  // rad/s
    double amplitude_x = 1.0;
    double amplitude_y = 1.0;
    
    void update(double time_now);
    void reset();
};

// Single Drone Controller
class DroneController {
private:
    agent_control_pkg::PIDController pid_x_;
    agent_control_pkg::PIDController pid_y_;
    agent_control_pkg::GT2FuzzyLogicSystem fls_x_;
    agent_control_pkg::GT2FuzzyLogicSystem fls_y_;
    
    bool use_fuzzy_logic_ = false;
    
    // Helper method for configuring fuzzy logic systems
    void configureFuzzyLogicSystem(agent_control_pkg::GT2FuzzyLogicSystem& fls, 
                                 const agent_control_pkg::FuzzyParams& params);
    void configureFuzzyLogicSystemDirect(agent_control_pkg::GT2FuzzyLogicSystem& fls);
    
public:
    DroneController();
    
    void initialize(const agent_control_pkg::SimPIDParams& pid_params);
    void initializeFuzzyLogic(const std::string& fuzzy_config_file);
    void setUsesFuzzyLogic(bool use_fls);
    
    void setSetpoint(double target_x, double target_y);
    void reset();
    
    std::pair<double, double> calculateControl(
        const DroneState& drone_state, 
        double dt,
        double wind_x = 0.0, 
        double wind_y = 0.0);
        
    // Getters for analysis
    agent_control_pkg::PIDController::PIDTerms getLastPIDTermsX() const;
    agent_control_pkg::PIDController::PIDTerms getLastPIDTermsY() const;
    std::pair<double, double> getSetpoints() const;
};

// Multi-Drone System
class MultiDroneSystem {
private:
    std::vector<DroneState> drones_;
    std::vector<DroneController> controllers_;
    std::vector<std::pair<double, double>> formation_offsets_;
    WindModel wind_model_;
    
    SystemConfig config_;
    agent_control_pkg::SimulationConfig sim_config_;
    
    // Performance tracking
    std::vector<std::vector<PerformanceMetrics>> metrics_x_;
    std::vector<std::vector<PerformanceMetrics>> metrics_y_;
    
public:
    MultiDroneSystem(const SystemConfig& config);
    
    bool initialize();
    void setFormationOffsets(const std::vector<std::pair<double, double>>& offsets);
    void setScenario(SystemConfig::TestScenario scenario);
    
    // Main simulation step
    void update(double time_now, double dt);
    
    // Control and reset
    void setFormationCenter(double center_x, double center_y);
    void resetSystem();
    void resetPhase(int phase_idx, double center_x, double center_y);
    
    // Getters
    const std::vector<DroneState>& getDrones() const { return drones_; }
    const SystemConfig& getConfig() const { return config_; }
    WindModel& getWindModel() { return wind_model_; }
    
    // Performance analysis
    void startMetricsPhase(int phase_idx, double center_x, double center_y);
    void finalizeMetrics(int phase_idx, double phase_start_time);
    void printMetrics(int phase_idx) const;
    void saveMetricsToFile(const std::string& filename, int phase_idx) const;
};

// Utility Functions
std::string scenarioToString(SystemConfig::TestScenario scenario);
SystemConfig createPIDOnlyConfig();
SystemConfig createPIDWindConfig();
SystemConfig createPIDWindFLCConfig();
std::vector<std::pair<double, double>> createTriangleFormation(double spacing = 2.0);
std::vector<std::pair<double, double>> createLineFormation(double spacing = 2.0);

} // namespace modular_pid_system 