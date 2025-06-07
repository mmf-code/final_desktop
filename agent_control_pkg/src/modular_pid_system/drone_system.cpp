#include "drone_system.hpp"
#include "../include/agent_control_pkg/config_reader.hpp"
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>

namespace modular_pid_system {

// Use std::clamp for value clamping (C++17)

// DroneState Implementation
void DroneState::update(double accel_cmd_x, double accel_cmd_y, double dt,
                       double external_force_x, double external_force_y) {
    // Update velocities with acceleration commands and external forces
    velocity_x += (accel_cmd_x + external_force_x) * dt;
    velocity_y += (accel_cmd_y + external_force_y) * dt;
    
    // Apply damping (air resistance simulation)
    velocity_x *= 0.98;
    velocity_y *= 0.98;
    
    // Update positions
    position_x += velocity_x * dt;
    position_y += velocity_y * dt;
}

void DroneState::reset(double init_x, double init_y) {
    position_x = init_x;
    position_y = init_y;
    velocity_x = 0.0;
    velocity_y = 0.0;
    prev_error_x_fls = 0.0;
    prev_error_y_fls = 0.0;
}

// PerformanceMetrics Implementation
void PerformanceMetrics::reset(double initial_val, double target_val) {
    peak_value = initial_val;
    peak_time = 0.0;
    overshoot_percent = 0.0;
    settling_time_2percent = -1.0;
    initial_value = initial_val;
    target_value = target_val;
    is_active = true;
}

void PerformanceMetrics::update(double current_value, double time_now) {
    if (!is_active) return;
    
    // Track peak value
    if (target_value > initial_value) {
        if (current_value > peak_value) {
            peak_value = current_value;
            peak_time = time_now;
        }
    } else if (target_value < initial_value) {
        if (current_value < peak_value) {
            peak_value = current_value;
            peak_time = time_now;
        }
    }
    
    // Check settling criteria (2% tolerance)
    const double SETTLING_PERCENTAGE = 0.02;
    double settling_range = std::abs(target_value - initial_value);
    double settling_tolerance = settling_range * SETTLING_PERCENTAGE;
    
    settling_tolerance = std::max(settling_tolerance, 1e-4);
    
    if (std::abs(current_value - target_value) <= settling_tolerance) {
        if (settling_time_2percent < 0.0) {
            settling_time_2percent = time_now;
        }
    } else {
        settling_time_2percent = -1.0;  // Reset if we leave settling band
    }
}

void PerformanceMetrics::finalize(double phase_start_time) {
    if (!is_active) return;
    
    // Calculate overshoot percentage
    if (std::abs(target_value - initial_value) > 1e-6) {
        if (target_value > initial_value) {
            overshoot_percent = ((peak_value - target_value) / 
                               (target_value - initial_value)) * 100.0;
        } else {
            overshoot_percent = ((target_value - peak_value) / 
                               (initial_value - target_value)) * 100.0;
        }
        
        if (overshoot_percent < 0) overshoot_percent = 0.0;
    }
    
    // Adjust times relative to phase start
    if (peak_time >= phase_start_time) {
        peak_time -= phase_start_time;
    } else {
        peak_time = 0.0;
    }
    
    if (settling_time_2percent >= phase_start_time) {
        settling_time_2percent -= phase_start_time;
    } else if (settling_time_2percent >= 0.0) {
        settling_time_2percent = 0.0;
    }
}

// WindModel Implementation
void WindModel::update(double time_now) {
    if (is_sine_wave) {
        wind_x = amplitude_x * sin(time_now * sine_frequency);
        wind_y = amplitude_y * sin(time_now * sine_frequency);
    }
    // For constant wind, values are already set
}

void WindModel::reset() {
    wind_x = 0.0;
    wind_y = 0.0;
}

// DroneController Implementation
DroneController::DroneController() : 
    pid_x_(1.0, 0.0, 0.0, -10.0, 10.0),  // Default PID parameters
    pid_y_(1.0, 0.0, 0.0, -10.0, 10.0),
    use_fuzzy_logic_(false) {}

void DroneController::initialize(const agent_control_pkg::SimPIDParams& pid_params) {
    pid_x_.setTunings(pid_params.kp, pid_params.ki, pid_params.kd);
    pid_y_.setTunings(pid_params.kp, pid_params.ki, pid_params.kd);
    
    pid_x_.setOutputLimits(pid_params.output_min, pid_params.output_max);
    pid_y_.setOutputLimits(pid_params.output_min, pid_params.output_max);
}

void DroneController::initializeFuzzyLogic(const std::string& fuzzy_config_file) {
    use_fuzzy_logic_ = true;
    
    std::cout << "DEBUG: Initializing fuzzy logic directly with proven configuration..." << std::endl;
    
    // Configure FLS for X-axis with proven working configuration
    DroneController::configureFuzzyLogicSystemDirect(fls_x_);
    
    // Configure FLS for Y-axis (same rules and sets)
    DroneController::configureFuzzyLogicSystemDirect(fls_y_);
    
    std::cout << "Fuzzy Logic System initialized with 21 rules using direct configuration" << std::endl;
}

void DroneController::configureFuzzyLogicSystemDirect(agent_control_pkg::GT2FuzzyLogicSystem& fls) {
    using FOU = agent_control_pkg::GT2FuzzyLogicSystem::IT2TriangularFS_FOU;
    
    // Define Input/Output Variables
    fls.addInputVariable("error");
    fls.addInputVariable("dError");
    fls.addInputVariable("wind");
    fls.addOutputVariable("correction");
    
    // Define Fuzzy Sets (exact same as working fuzzy_test_main.cpp)
    // Error sets
    fls.addFuzzySetToVariable("error","NB",FOU{-10.0, -8.0, -6.0,  -11.0, -8.5, -5.5});
    fls.addFuzzySetToVariable("error","NS",FOU{ -7.0, -4.0, -1.0,   -8.0, -4.5,  0.0});
    fls.addFuzzySetToVariable("error","ZE",FOU{ -1.0,  0.0,  1.0,   -2.0,  0.0,  2.0});
    fls.addFuzzySetToVariable("error","PS",FOU{  1.0,  4.0,  7.0,    0.0,  4.5,  8.0});
    fls.addFuzzySetToVariable("error","PB",FOU{  6.0,  8.0, 10.0,    5.5,  8.5, 11.0});
    
    // dError sets
    fls.addFuzzySetToVariable("dError","DN",FOU{-5.0, -3.0,  0.0,   -6.0, -3.5,  1.0});
    fls.addFuzzySetToVariable("dError","DZ",FOU{-1.0,  0.0,  1.0,   -1.5,  0.0,  1.5});
    fls.addFuzzySetToVariable("dError","DP",FOU{ 0.0,  3.0,  5.0,   -1.0,  3.0,  6.0});
    
    // Wind sets
    fls.addFuzzySetToVariable("wind","SNW",FOU{-10.0, -8.0, -5.0,  -11.0, -8.5, -4.5}); // Strong Negative Wind
    fls.addFuzzySetToVariable("wind","WNW",FOU{ -7.0, -4.0, -1.0,   -8.0, -4.5,  0.0}); // Weak Negative Wind
    fls.addFuzzySetToVariable("wind","NWN",FOU{ -1.0,  0.0,  1.0,   -1.5,  0.0,  1.5}); // No Wind
    fls.addFuzzySetToVariable("wind","WPW",FOU{  1.0,  4.0,  7.0,    0.0,  4.5,  8.0}); // Weak Positive Wind
    fls.addFuzzySetToVariable("wind","SPW",FOU{  6.0,  8.0, 10.0,    5.5,  8.5, 11.0}); // Strong Positive Wind
    
    // Correction sets
    fls.addFuzzySetToVariable("correction","XLNC",FOU{-7.5, -6.0, -4.5,  -8.5, -6.5, -4.0}); // Extra Large Negative
    fls.addFuzzySetToVariable("correction","LNC", FOU{-5.0, -4.0, -2.5,  -5.5, -4.5, -2.0}); // Large Negative
    fls.addFuzzySetToVariable("correction","SNC", FOU{-3.0, -1.5,  0.0,  -3.5, -1.5,  0.5}); // Small Negative
    fls.addFuzzySetToVariable("correction","NC",  FOU{-0.5,  0.0,  0.5,  -1.0,  0.0,  1.0}); // No Change / Zero
    fls.addFuzzySetToVariable("correction","SPC", FOU{ 0.0,  1.5,  3.0,  -0.5,  1.5,  3.5}); // Small Positive
    fls.addFuzzySetToVariable("correction","LPC", FOU{ 2.5,  4.0,  5.0,   2.0,  4.5,  5.5}); // Large Positive
    fls.addFuzzySetToVariable("correction","XLPC",FOU{ 4.5,  6.0,  7.5,   4.0,  6.5,  8.5}); // Extra Large Positive

    // Define FLS Rules (exact same 21 rules as working fuzzy_test_main.cpp)
    auto R = [&](const std::string& e, const std::string& de, const std::string& w, const std::string& out){
        fls.addRule({{{"error",e},{"dError",de},{"wind",w}}, {"correction",out}});
    };
    
    // No wind (NWN) - Basic control rules
    R("PB","DZ","NWN","LPC"); R("PS","DZ","NWN","SPC"); R("ZE","DZ","NWN","NC");  R("NS","DZ","NWN","SNC"); R("NB","DZ","NWN","LNC");
    
    // Modified rules: Reduced strength when dError helps reduce error
    R("PB","DN","NWN","SPC"); // Changed from LPC to SPC (error is positive, DN helps reduce it)
    R("PS","DN","NWN","NC");  // Changed from SPC to NC (error is positive, DN helps reduce it)
    R("ZE","DN","NWN","SNC"); // Keep as is (already modified)
    R("NS","DN","NWN","NC");  // Keep as is
    R("NB","DN","NWN","SNC"); // Keep as is

    // Modified rules: Reduced strength when dError helps reduce error
    R("PB","DP","NWN","NC");  // Keep as is
    R("PS","DP","NWN","SNC"); // Keep as is
    R("ZE","DP","NWN","SPC"); // Keep as is (already modified)
    R("NS","DP","NWN","NC");  // Changed from SNC to NC (error is negative, DP helps reduce it)
    R("NB","DP","NWN","SNC"); // Changed from LNC to SNC (error is negative, DP helps reduce it)

    // Wind scenario rules
    R("ZE","DZ","SPW","LNC");  // Strong Positive Wind -> Large Negative Correction
    R("ZE","DZ","SNW","LPC");  // Strong Negative Wind -> Large Positive Correction
    R("ZE","DZ","WPW","SNC");  // Weak Positive Wind -> Small Negative Correction
    R("ZE","DZ","WNW","SPC");  // Weak Negative Wind -> Small Positive Correction
    R("PB","DZ","SPW","XLPC"); // Error PB, Strong Positive Wind -> XLPC
    R("NB","DZ","SNW","XLNC"); // Error NB, Strong Negative Wind -> XLNC
}

void DroneController::configureFuzzyLogicSystem(agent_control_pkg::GT2FuzzyLogicSystem& fls, 
                                               const agent_control_pkg::FuzzyParams& params) {
    // This method is kept for compatibility but will use direct configuration
    // if the params are empty or invalid
    
    if (params.sets.empty() || params.rules.empty()) {
        std::cout << "YAML params empty, using direct configuration..." << std::endl;
        DroneController::configureFuzzyLogicSystemDirect(fls);
        return;
    }
    
    // Add input variables
    fls.addInputVariable("error");
    fls.addInputVariable("dError");
    fls.addInputVariable("wind");
    
    // Add output variable
    fls.addOutputVariable("correction");
    
    // Add fuzzy sets for each variable
    for (const auto& var_pair : params.sets) {
        const std::string& var_name = var_pair.first;
        const auto& sets = var_pair.second;
        
        for (const auto& set_pair : sets) {
            const std::string& set_name = set_pair.first;
            const auto& fou = set_pair.second;
            
            agent_control_pkg::GT2FuzzyLogicSystem::IT2TriangularFS_FOU triangular_fou{
                fou.l1, fou.l2, fou.l3,  // Lower membership function
                fou.u1, fou.u2, fou.u3   // Upper membership function
            };
            
            fls.addFuzzySetToVariable(var_name, set_name, triangular_fou);
        }
    }
    
    // Add fuzzy rules
    for (const auto& rule : params.rules) {
        if (rule.size() >= 4) {
            agent_control_pkg::GT2FuzzyLogicSystem::FuzzyRule fuzzy_rule;
            fuzzy_rule.antecedents["error"] = rule[0];
            fuzzy_rule.antecedents["dError"] = rule[1];
            fuzzy_rule.antecedents["wind"] = rule[2];
            fuzzy_rule.consequent = {"correction", rule[3]};
            
            fls.addRule(fuzzy_rule);
        }
    }
}

void DroneController::setUsesFuzzyLogic(bool use_fls) {
    use_fuzzy_logic_ = use_fls;
}

void DroneController::setSetpoint(double target_x, double target_y) {
    pid_x_.setSetpoint(target_x);
    pid_y_.setSetpoint(target_y);
}

void DroneController::reset() {
    pid_x_.reset();
    pid_y_.reset();
}

std::pair<double, double> DroneController::calculateControl(
    const DroneState& drone_state, double dt, double wind_x, double wind_y) {
    
    // Calculate PID control outputs
    auto terms_x = pid_x_.calculate_with_terms(drone_state.position_x, dt);
    auto terms_y = pid_y_.calculate_with_terms(drone_state.position_y, dt);
    
    double control_x = terms_x.total_output;
    double control_y = terms_y.total_output;
    
    // Add fuzzy logic correction if enabled
    if (use_fuzzy_logic_) {
        double error_x = pid_x_.getSetpoint() - drone_state.position_x;
        double error_y = pid_y_.getSetpoint() - drone_state.position_y;
        
        double d_error_x = (dt > 1e-9) ? 
            (error_x - drone_state.prev_error_x_fls) / dt : 0.0;
        double d_error_y = (dt > 1e-9) ? 
            (error_y - drone_state.prev_error_y_fls) / dt : 0.0;
        
        double fls_correction_x = fls_x_.calculateOutput(error_x, d_error_x, wind_x);
        double fls_correction_y = fls_y_.calculateOutput(error_y, d_error_y, wind_y);
        
        control_x += fls_correction_x;
        control_y += fls_correction_y;
    }
    
    // Apply output limits
    control_x = std::clamp(control_x, -10.0, 10.0);  // Default limits
    control_y = std::clamp(control_y, -10.0, 10.0);
    
    return {control_x, control_y};
}

agent_control_pkg::PIDController::PIDTerms DroneController::getLastPIDTermsX() const {
    return pid_x_.getLastTerms();
}

agent_control_pkg::PIDController::PIDTerms DroneController::getLastPIDTermsY() const {
    return pid_y_.getLastTerms();
}

std::pair<double, double> DroneController::getSetpoints() const {
    return {pid_x_.getSetpoint(), pid_y_.getSetpoint()};
}

// MultiDroneSystem Implementation
MultiDroneSystem::MultiDroneSystem(const SystemConfig& config) : config_(config) {
    drones_.resize(config_.num_drones);
    controllers_.resize(config_.num_drones);
    formation_offsets_.resize(config_.num_drones);
    
    // Initialize default formation (triangle)
    if (config_.num_drones <= 3) {
        formation_offsets_ = createTriangleFormation();
    } else {
        formation_offsets_ = createLineFormation();
    }
    
    // Initialize metrics storage (assuming 5 phases max)
    int num_phases = 5;
    metrics_x_.resize(config_.num_drones, std::vector<PerformanceMetrics>(num_phases));
    metrics_y_.resize(config_.num_drones, std::vector<PerformanceMetrics>(num_phases));
}

bool MultiDroneSystem::initialize() {
    try {
        // Load simulation configuration
        sim_config_ = agent_control_pkg::ConfigReader::loadConfig("simulation_params.yaml");
        
        // Initialize controllers
        for (auto& controller : controllers_) {
            controller.initialize(sim_config_.pid_params);
            
            if (config_.enable_fuzzy_logic) {
                controller.initializeFuzzyLogic("fuzzy_params.yaml");
                controller.setUsesFuzzyLogic(true);
            }
        }
        
        // Configure wind model based on scenario
        switch (config_.current_scenario) {
            case SystemConfig::TestScenario::PID_WITH_WIND:
            case SystemConfig::TestScenario::PID_WIND_FLC:
                wind_model_.amplitude_x = 1.0;
                wind_model_.amplitude_y = 1.0;
                wind_model_.is_sine_wave = true;
                wind_model_.sine_frequency = 1.0;
                break;
            default:
                wind_model_.reset();
                break;
        }
        
        return true;
    } catch (const std::exception& e) {
        std::cerr << "Failed to initialize MultiDroneSystem: " << e.what() << std::endl;
        return false;
    }
}

void MultiDroneSystem::setFormationOffsets(const std::vector<std::pair<double, double>>& offsets) {
    if (offsets.size() == formation_offsets_.size()) {
        formation_offsets_ = offsets;
    }
}

void MultiDroneSystem::setScenario(SystemConfig::TestScenario scenario) {
    config_.current_scenario = scenario;
    
    // Update system configuration based on scenario
    switch (scenario) {
        case SystemConfig::TestScenario::PID_ONLY:
            config_.enable_wind = false;
            config_.enable_fuzzy_logic = false;
            break;
        case SystemConfig::TestScenario::PID_WITH_WIND:
            config_.enable_wind = true;
            config_.enable_fuzzy_logic = false;
            break;
        case SystemConfig::TestScenario::PID_WIND_FLC:
            config_.enable_wind = true;
            config_.enable_fuzzy_logic = true;
            break;
    }
    
    // Update controllers
    for (auto& controller : controllers_) {
        controller.setUsesFuzzyLogic(config_.enable_fuzzy_logic);
    }
}

void MultiDroneSystem::update(double time_now, double dt) {
    // Update wind model
    if (config_.enable_wind) {
        wind_model_.update(time_now);
    }
    
    // Update each drone
    for (std::size_t i = 0; i < drones_.size(); ++i) {
        // Calculate control commands
        auto control = controllers_[i].calculateControl(
            drones_[i], dt, wind_model_.wind_x, wind_model_.wind_y);
        
        // Update drone state
        drones_[i].update(control.first, control.second, dt, 
                         wind_model_.wind_x, wind_model_.wind_y);
        
        // Update error history for FLS
        auto setpoints = controllers_[i].getSetpoints();
        drones_[i].prev_error_x_fls = setpoints.first - drones_[i].position_x;
        drones_[i].prev_error_y_fls = setpoints.second - drones_[i].position_y;
    }
}

void MultiDroneSystem::setFormationCenter(double center_x, double center_y) {
    for (std::size_t i = 0; i < controllers_.size(); ++i) {
        double target_x = center_x + formation_offsets_[i].first;
        double target_y = center_y + formation_offsets_[i].second;
        controllers_[i].setSetpoint(target_x, target_y);
    }
}

void MultiDroneSystem::resetSystem() {
    for (auto& drone : drones_) {
        drone.reset();
    }
    for (auto& controller : controllers_) {
        controller.reset();
    }
    wind_model_.reset();
}

void MultiDroneSystem::resetPhase(int phase_idx, double center_x, double center_y) {
    setFormationCenter(center_x, center_y);
    
    for (auto& controller : controllers_) {
        controller.reset();
    }
    
    // Reset drone FLS error history
    for (auto& drone : drones_) {
        drone.prev_error_x_fls = 0.0;
        drone.prev_error_y_fls = 0.0;
    }
}

void MultiDroneSystem::startMetricsPhase(int phase_idx, double center_x, double center_y) {
    if (phase_idx >= 0 && phase_idx < static_cast<int>(metrics_x_[0].size())) {
        for (std::size_t i = 0; i < drones_.size(); ++i) {
            double target_x = center_x + formation_offsets_[i].first;
            double target_y = center_y + formation_offsets_[i].second;
            
            metrics_x_[i][static_cast<std::size_t>(phase_idx)].reset(drones_[i].position_x, target_x);
            metrics_y_[i][static_cast<std::size_t>(phase_idx)].reset(drones_[i].position_y, target_y);
        }
    }
}

void MultiDroneSystem::finalizeMetrics(int phase_idx, double phase_start_time) {
    if (phase_idx >= 0 && phase_idx < static_cast<int>(metrics_x_[0].size())) {
        for (std::size_t i = 0; i < drones_.size(); ++i) {
            metrics_x_[i][static_cast<std::size_t>(phase_idx)].finalize(phase_start_time);
            metrics_y_[i][static_cast<std::size_t>(phase_idx)].finalize(phase_start_time);
        }
    }
}

void MultiDroneSystem::printMetrics(int phase_idx) const {
    if (phase_idx < 0 || phase_idx >= static_cast<int>(metrics_x_[0].size())) return;
    
    std::cout << "\n--- PHASE " << phase_idx + 1 << " METRICS ---" << std::endl;
    
    for (std::size_t i = 0; i < drones_.size(); ++i) {
        const auto& mx = metrics_x_[i][static_cast<std::size_t>(phase_idx)];
        const auto& my = metrics_y_[i][static_cast<std::size_t>(phase_idx)];
        
        if (mx.is_active) {
            std::cout << "Drone " << i << ":" << std::endl;
            std::cout << "  X-axis: OS=" << std::fixed << std::setprecision(1) 
                      << mx.overshoot_percent << "%, ST(2%)=" 
                      << (mx.settling_time_2percent >= 0 ? 
                          std::to_string(mx.settling_time_2percent) : "N/A")
                      << "s, Peak=" << std::setprecision(2) << mx.peak_value 
                      << " @ " << mx.peak_time << "s" << std::endl;
            std::cout << "  Y-axis: OS=" << std::fixed << std::setprecision(1) 
                      << my.overshoot_percent << "%, ST(2%)=" 
                      << (my.settling_time_2percent >= 0 ? 
                          std::to_string(my.settling_time_2percent) : "N/A")
                      << "s, Peak=" << std::setprecision(2) << my.peak_value 
                      << " @ " << my.peak_time << "s" << std::endl;
        }
    }
}

void MultiDroneSystem::saveMetricsToFile(const std::string& filename, int phase_idx) const {
    std::ofstream file(filename, std::ios::app);
    if (!file.is_open()) return;
    
    file << "\n--- PHASE " << phase_idx + 1 << " METRICS ---\n";
    
    for (std::size_t i = 0; i < drones_.size(); ++i) {
        if (phase_idx >= 0 && phase_idx < static_cast<int>(metrics_x_[0].size())) {
            const auto& mx = metrics_x_[i][static_cast<std::size_t>(phase_idx)];
            const auto& my = metrics_y_[i][static_cast<std::size_t>(phase_idx)];
            
            if (mx.is_active) {
                file << "Drone " << i << ":\n";
                file << "  X-axis: OS=" << std::fixed << std::setprecision(1) 
                     << mx.overshoot_percent << "%, ST(2%)=" 
                     << (mx.settling_time_2percent >= 0 ? 
                         std::to_string(mx.settling_time_2percent) : "N/A")
                     << "s, Peak=" << std::setprecision(2) << mx.peak_value 
                     << " @ " << mx.peak_time << "s\n";
                file << "  Y-axis: OS=" << std::fixed << std::setprecision(1) 
                     << my.overshoot_percent << "%, ST(2%)=" 
                     << (my.settling_time_2percent >= 0 ? 
                         std::to_string(my.settling_time_2percent) : "N/A")
                     << "s, Peak=" << std::setprecision(2) << my.peak_value 
                     << " @ " << my.peak_time << "s\n";
            }
        }
    }
}

// Utility Functions Implementation
std::string scenarioToString(SystemConfig::TestScenario scenario) {
    switch (scenario) {
        case SystemConfig::TestScenario::PID_ONLY:
            return "PID_ONLY";
        case SystemConfig::TestScenario::PID_WITH_WIND:
            return "PID_WITH_WIND";
        case SystemConfig::TestScenario::PID_WIND_FLC:
            return "PID_WIND_FLC";
        default:
            return "UNKNOWN";
    }
}

SystemConfig createPIDOnlyConfig() {
    SystemConfig config;
    config.current_scenario = SystemConfig::TestScenario::PID_ONLY;
    config.enable_pid = true;
    config.enable_wind = false;
    config.enable_fuzzy_logic = false;
    config.test_name = "pid_only_test";
    return config;
}

SystemConfig createPIDWindConfig() {
    SystemConfig config;
    config.current_scenario = SystemConfig::TestScenario::PID_WITH_WIND;
    config.enable_pid = true;
    config.enable_wind = true;
    config.enable_fuzzy_logic = false;
    config.test_name = "pid_wind_test";
    return config;
}

SystemConfig createPIDWindFLCConfig() {
    SystemConfig config;
    config.current_scenario = SystemConfig::TestScenario::PID_WIND_FLC;
    config.enable_pid = true;
    config.enable_wind = true;
    config.enable_fuzzy_logic = true;
    config.test_name = "pid_wind_flc_test";
    return config;
}

std::vector<std::pair<double, double>> createTriangleFormation(double spacing) {
    return {
        {0.0, 0.0},                    // Leader drone at center
        {-spacing, -spacing * 0.866},  // Left rear
        {spacing, -spacing * 0.866}    // Right rear
    };
}

std::vector<std::pair<double, double>> createLineFormation(double spacing) {
    return {
        {0.0, 0.0},      // Center
        {-spacing, 0.0}, // Left
        {spacing, 0.0}   // Right
    };
}

} // namespace modular_pid_system 