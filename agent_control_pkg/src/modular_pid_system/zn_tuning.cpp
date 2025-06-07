#include "zn_tuning.hpp"
#include "drone_system.hpp"
#include <iostream>
#include <fstream>
#include <iomanip>
#include <cmath>
#include <algorithm>
#include <numeric>
#include <filesystem>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace modular_pid_system {

// ZNTuningEngine Implementation
ZNTuningEngine::ZNTuningEngine(MultiDroneSystem* system, const ZNTuningConfig& config)
    : config_(config), system_(system) {}

ZNAnalysisResult ZNTuningEngine::performAutoTuning() {
    std::vector<ZNAnalysisResult> all_results;
    ZNAnalysisResult best_result;
    
    std::cout << "Starting ZN auto-tuning..." << std::endl;
    std::cout << "Testing Kp from " << config_.kp_start << " to " << config_.kp_end 
              << " (step: " << config_.kp_step << ")" << std::endl;
    
    for (double kp = config_.kp_start; kp <= config_.kp_end; kp += config_.kp_step) {
        std::cout << "Testing Kp = " << kp << "..." << std::endl;
        
        ZNAnalysisResult result = runSingleKpTest(kp);
        all_results.push_back(result);
        
        if (result.is_oscillating && !result.is_unstable) {
            best_result = result;
            best_result.ultimate_gain = kp;
            std::cout << "  Found oscillation! Ku = " << kp << ", Tu = " << result.period << "s" << std::endl;
            break;
        }
        if (result.is_unstable) {
            std::cout << "  System became unstable at Kp = " << kp << std::endl;
            break;
        }
        std::cout << "  No oscillation detected" << std::endl;
    }
    
    if (best_result.is_oscillating) {
        calculateRecommendedGains(best_result);
        
        // Save detailed results if enabled
        if (config_.save_detailed_results) {
            std::filesystem::create_directories(config_.output_directory);
            generateTuningReport(all_results, config_.output_directory + "/detailed_tuning_report.txt");
        }
    }
    
    return best_result;
}

ZNAnalysisResult ZNTuningEngine::testSpecificKp(double kp_value) {
    return runSingleKpTest(kp_value);
}

ZNAnalysisResult ZNTuningEngine::runSingleKpTest(double kp_test) {
    ZNAnalysisResult result;
    
    // Reset system to initial state
    system_->resetSystem();
    system_->setScenario(SystemConfig::TestScenario::PID_ONLY);  // PID only for tuning
    
    // Set test setpoint (step input)
    system_->setFormationCenter(config_.test_center_x, config_.test_center_y);
    
    // Collect error data during simulation
    auto error_data = collectErrorData(kp_test);
    
    // Analyze the collected data
    result = analyzeOscillations(error_data);
    result.ultimate_gain = kp_test;
    
    return result;
}

std::vector<std::pair<double, double>> ZNTuningEngine::collectErrorData(double kp_test) {
    std::vector<std::pair<double, double>> error_history;
    
    // TODO: Set the specific Kp value for testing
    // This would require modifying the PID controller gains temporarily
    // For now, we'll simulate the data collection
    
    double dt = system_->getConfig().dt;
    double sim_time = config_.test_duration;
    
    // Pre-allocate vector capacity to avoid repeated allocations
    std::size_t expected_size = static_cast<std::size_t>(sim_time / dt) + 1;
    error_history.reserve(expected_size);
    
    for (double t = 0; t < sim_time; t += dt) {
        system_->update(t, dt);
        
        // Get error from target drone
        const auto& drones = system_->getDrones();
        if (config_.target_drone_id < static_cast<int>(drones.size())) {
            const auto& drone = drones[config_.target_drone_id];
            
            double error_x = config_.test_center_x - drone.position_x;
            double error_y = config_.test_center_y - drone.position_y;
            
            double error = (config_.tuning_axis == ZNTuningConfig::TuningAxis::X_AXIS) ? 
                          error_x : error_y;
            
            error_history.push_back({t, error});
        }
    }
    
    return error_history;
}

ZNAnalysisResult ZNTuningEngine::analyzeOscillations(
    const std::vector<std::pair<double, double>>& error_history) {
    
    ZNAnalysisResult result;
    
    if (error_history.size() < 10) {
        return result; // Not enough data
    }
    
    // Extract error values
    std::vector<double> errors;
    errors.reserve(error_history.size());
    for (const auto& point : error_history) {
        errors.push_back(point.second);
    }
    
    // Detect oscillation
    bool oscillating = detectOscillation(errors, result.period, result.overshoot_percent);
    result.is_oscillating = oscillating;
    
    if (!oscillating) {
        return result;
    }
    
    // Calculate additional metrics for first third and last third
    std::size_t third_size = errors.size() / 3;
    (void)calculateOvershoot(
        std::vector<double>(errors.begin(), errors.begin() + static_cast<std::ptrdiff_t>(third_size)), 0.0);
    (void)calculateOvershoot(
        std::vector<double>(errors.end() - static_cast<std::ptrdiff_t>(third_size), errors.end()), 0.0);
    
    // Calculate settling metrics
    double target_value = 0.0;  // Error should settle to zero
    result.settling_time_2percent = calculateSettlingTime(errors, target_value);
    
    return result;
}

bool ZNTuningEngine::detectOscillation(const std::vector<double>& signal, double& period, double& amplitude) const {
    if (signal.size() < 10) {
        return false;
    }
    
    // Find peaks and valleys
    std::vector<std::size_t> peaks;
    std::vector<std::size_t> valleys;
    
    for (std::size_t i = 1; i < signal.size() - 1; ++i) {
        if (signal[i] > signal[i-1] && signal[i] > signal[i+1]) {
            if (signal[i] > config_.oscillation_threshold) {
                peaks.push_back(i);
            }
        } else if (signal[i] < signal[i-1] && signal[i] < signal[i+1]) {
            if (std::abs(signal[i]) > config_.oscillation_threshold) {
                valleys.push_back(i);
            }
        }
    }
    
    // Need at least config_.min_oscillation_cycles peaks for oscillation
    if (peaks.size() < static_cast<std::size_t>(config_.min_oscillation_cycles)) {
        return false;
    }
    
    // Calculate average period from peak-to-peak intervals
    double total_period = 0.0;
    std::size_t period_count = 0;
    
    for (std::size_t i = 1; i < peaks.size(); ++i) {
        double peak_interval = static_cast<double>(peaks[i] - peaks[i-1]) * config_.test_duration / signal.size();
        total_period += peak_interval;
        period_count++;
    }
    
    if (period_count == 0) {
        return false;
    }
    
    period = total_period / static_cast<double>(period_count);
    
    // Calculate amplitude as average peak magnitude
    double total_amplitude = 0.0;
    for (std::size_t peak_idx : peaks) {
        total_amplitude += std::abs(signal[peak_idx]);
    }
    amplitude = total_amplitude / static_cast<double>(peaks.size());
    
    return amplitude > config_.oscillation_threshold;
}

double ZNTuningEngine::calculateOvershoot(const std::vector<double>& response, double target_value) {
    if (response.empty()) {
        return 0.0;
    }
    
    double max_value = *std::max_element(response.begin(), response.end());
    double min_value = *std::min_element(response.begin(), response.end());
    
    double initial_value = response[0];
    double overshoot = 0.0;
    
    if (target_value > initial_value) {
        // Step up
        overshoot = (max_value - target_value) / (target_value - initial_value) * 100.0;
    } else if (target_value < initial_value) {
        // Step down
        overshoot = (target_value - min_value) / (initial_value - target_value) * 100.0;
    }
    
    return std::max(0.0, overshoot);
}

double ZNTuningEngine::calculateSettlingTime(const std::vector<double>& response, double target_value, double tolerance) const {
    if (response.empty()) {
        return -1.0;
    }
    
    double settling_band = std::abs(response[0] - target_value) * tolerance;
    settling_band = std::max(settling_band, 1e-6);
    
    // Find last time the response was outside the settling band
    std::ptrdiff_t last_outside = -1;
    for (std::ptrdiff_t i = static_cast<std::ptrdiff_t>(response.size()) - 1; i >= 0; --i) {
        if (std::abs(response[static_cast<std::size_t>(i)] - target_value) > settling_band) {
            last_outside = i;
            break;
        }
    }
    
    if (last_outside < 0) {
        return 0.0; // Always within band
    }
    
    return static_cast<double>(last_outside) * config_.test_duration / static_cast<double>(response.size());
}

void ZNTuningEngine::calculateRecommendedGains(ZNAnalysisResult& result) {
    if (!result.is_oscillating) {
        return;
    }
    
    result.ultimate_period = result.period;
    double ku = result.ultimate_gain;
    double tu = result.ultimate_period;
    
    result.recommended_gains.clear();
    result.recommended_gains.push_back(ZNTuningUtils::calculateClassicPID(ku, tu));
    result.recommended_gains.push_back(ZNTuningUtils::calculatePessenIntegralRule(ku, tu));
    result.recommended_gains.push_back(ZNTuningUtils::calculateSomeOvershootRule(ku, tu));
    result.recommended_gains.push_back(ZNTuningUtils::calculateNoOvershootRule(ku, tu));
}

void ZNTuningEngine::printResults(const ZNAnalysisResult& result) {
    std::cout << "\n--- ZN ANALYSIS RESULTS ---" << std::endl;
    std::cout << "Oscillating: " << (result.is_oscillating ? "Yes" : "No") << std::endl;
    std::cout << "Unstable: " << (result.is_unstable ? "Yes" : "No") << std::endl;
    
    if (result.is_oscillating) {
        std::cout << "Ultimate Gain (Ku): " << std::fixed << std::setprecision(3) 
                  << result.ultimate_gain << std::endl;
        std::cout << "Ultimate Period (Tu): " << std::setprecision(3) 
                  << result.ultimate_period << "s" << std::endl;
        std::cout << "Overshoot: " << std::setprecision(1) 
                  << result.overshoot_percent << "%" << std::endl;
        
        if (result.settling_time_2percent >= 0) {
            std::cout << "Settling Time (2%): " << std::setprecision(2) 
                      << result.settling_time_2percent << "s" << std::endl;
        }
        
        std::cout << "\nRecommended PID gains:" << std::endl;
        for (const auto& gains : result.recommended_gains) {
            std::cout << "  " << gains.method_name << ": Kp=" << std::setprecision(3) 
                      << gains.kp << ", Ki=" << gains.ki << ", Kd=" << gains.kd << std::endl;
        }
    } else {
        std::cout << "No sustained oscillation detected." << std::endl;
        std::cout << "Consider increasing Kp or checking system dynamics." << std::endl;
    }
}

void ZNTuningEngine::saveResults(const ZNAnalysisResult& result, const std::string& filename) const {
    std::ofstream file(filename);
    if (!file.is_open()) {
        return;
    }
    
    file << "Ziegler-Nichols Tuning Results\n";
    file << "==============================\n\n";
    file << "Test Configuration:\n";
    file << "  Kp Range: " << config_.kp_start << " to " << config_.kp_end 
         << " (step: " << config_.kp_step << ")\n";
    file << "  Test Duration: " << config_.test_duration << "s\n";
    file << "  Target Drone: " << config_.target_drone_id << "\n";
    
    // Avoid nested conditional operators
    std::string axis_name;
    if (config_.tuning_axis == ZNTuningConfig::TuningAxis::X_AXIS) {
        axis_name = "X";
    } else if (config_.tuning_axis == ZNTuningConfig::TuningAxis::Y_AXIS) {
        axis_name = "Y";
    } else {
        axis_name = "Both";
    }
    file << "  Tuning Axis: " << axis_name << "\n\n";
    
    file << "Results:\n";
    file << "  Oscillating: " << (result.is_oscillating ? "Yes" : "No") << "\n";
    file << "  Unstable: " << (result.is_unstable ? "Yes" : "No") << "\n";
    
    if (result.is_oscillating) {
        file << "  Ultimate Gain (Ku): " << std::fixed << std::setprecision(3) 
             << result.ultimate_gain << "\n";
        file << "  Ultimate Period (Tu): " << std::setprecision(3) 
             << result.ultimate_period << "s\n";
        file << "  Overshoot: " << std::setprecision(1) 
             << result.overshoot_percent << "%\n";
        
        if (result.settling_time_2percent >= 0) {
            file << "  Settling Time (2%): " << std::setprecision(2) 
                 << result.settling_time_2percent << "s\n";
        }
        
        file << "\nRecommended PID gains:\n";
        for (const auto& gains : result.recommended_gains) {
            file << "  " << gains.method_name << ": Kp=" << std::setprecision(3) 
                 << gains.kp << ", Ki=" << gains.ki << ", Kd=" << gains.kd << "\n";
        }
    }
    
    file.close();
}

void ZNTuningEngine::generateTuningReport(const std::vector<ZNAnalysisResult>& all_results, const std::string& filename) {
    std::ofstream file(filename);
    if (!file.is_open()) return;
    
    file << "Detailed ZN Tuning Report\n";
    file << "=========================\n\n";
    
    for (std::size_t i = 0; i < all_results.size(); ++i) {
        const auto& result = all_results[i];
        file << "Test " << i + 1 << ":\n";
        file << "  Kp: " << result.ultimate_gain << "\n";
        file << "  Oscillating: " << (result.is_oscillating ? "Yes" : "No") << "\n";
        if (result.is_oscillating) {
            file << "  Period: " << result.period << "s\n";
            file << "  Overshoot: " << result.overshoot_percent << "%\n";
        }
        file << "\n";
    }
}

// ZNTuningUtils Implementation
ZNAnalysisResult::RecommendedGains ZNTuningUtils::calculateClassicPID(double ku, double tu) {
    ZNAnalysisResult::RecommendedGains gains;
    gains.method_name = "Classical ZN";
    gains.kp = 0.6 * ku;
    gains.ki = (1.2 * ku) / tu;
    gains.kd = 0.075 * ku * tu;
    return gains;
}

ZNAnalysisResult::RecommendedGains ZNTuningUtils::calculatePessenIntegralRule(double ku, double tu) {
    ZNAnalysisResult::RecommendedGains gains;
    gains.method_name = "Pessen Integral";
    gains.kp = 0.7 * ku;
    gains.ki = (1.75 * ku) / tu;
    gains.kd = 0.105 * ku * tu;
    return gains;
}

ZNAnalysisResult::RecommendedGains ZNTuningUtils::calculateSomeOvershootRule(double ku, double tu) {
    ZNAnalysisResult::RecommendedGains gains;
    gains.method_name = "Some Overshoot";
    gains.kp = 0.33 * ku;
    gains.ki = (0.66 * ku) / tu;
    gains.kd = 0.11 * ku * tu;
    return gains;
}

ZNAnalysisResult::RecommendedGains ZNTuningUtils::calculateNoOvershootRule(double ku, double tu) {
    ZNAnalysisResult::RecommendedGains gains;
    gains.method_name = "No Overshoot";
    gains.kp = 0.2 * ku;
    gains.ki = (0.4 * ku) / tu;
    gains.kd = 0.066 * ku * tu;
    return gains;
}

std::vector<double> ZNTuningUtils::findPeaks(const std::vector<double>& signal, double min_height) {
    std::vector<double> peaks;
    
    if (signal.size() < 3) return peaks;
    
    for (std::size_t i = 1; i < signal.size() - 1; ++i) {
        if (signal[i] > signal[i-1] && signal[i] > signal[i+1] && signal[i] > min_height) {
            peaks.push_back(static_cast<double>(i));
        }
    }
    
    return peaks;
}

std::vector<double> ZNTuningUtils::findValleys(const std::vector<double>& signal, double max_depth) {
    std::vector<double> valleys;
    
    if (signal.size() < 3) return valleys;
    
    for (std::size_t i = 1; i < signal.size() - 1; ++i) {
        if (signal[i] < signal[i-1] && signal[i] < signal[i+1] && signal[i] < max_depth) {
            valleys.push_back(static_cast<double>(i));
        }
    }
    
    return valleys;
}

double ZNTuningUtils::calculateRMS(const std::vector<double>& signal) {
    if (signal.empty()) return 0.0;
    
    double sum_squares = 0.0;
    for (double val : signal) {
        sum_squares += val * val;
    }
    
    return std::sqrt(sum_squares / signal.size());
}

double ZNTuningUtils::calculateMean(const std::vector<double>& signal) {
    if (signal.empty()) return 0.0;
    
    return std::accumulate(signal.begin(), signal.end(), 0.0) / signal.size();
}

std::vector<double> ZNTuningUtils::applyMovingAverage(const std::vector<double>& signal, int window_size) {
    std::vector<double> result;
    if (signal.empty() || window_size <= 0) return result;
    
    result.reserve(signal.size());
    
    for (std::size_t i = 0; i < signal.size(); ++i) {
        double sum = 0.0;
        int count = 0;
        
        int start = std::max(0, static_cast<int>(i) - window_size / 2);
        int end = std::min(static_cast<int>(signal.size()), static_cast<int>(i) + window_size / 2 + 1);
        
        for (int j = start; j < end; ++j) {
            sum += signal[static_cast<std::size_t>(j)];
            count++;
        }
        
        result.push_back(count > 0 ? sum / count : 0.0);
    }
    
    return result;
}

double ZNTuningUtils::estimateDominantFrequency(const std::vector<double>& signal, double sample_rate) {
    if (signal.size() < 4) return 0.0;
    
    // Simple peak counting method for frequency estimation
    std::vector<double> peaks = findPeaks(signal, 0.0);
    
    if (peaks.size() < 2) return 0.0;
    
    // Calculate average period from peaks
    double total_period = 0.0;
    for (std::size_t i = 1; i < peaks.size(); ++i) {
        total_period += (peaks[i] - peaks[i-1]) / sample_rate;
    }
    
    double avg_period = total_period / (peaks.size() - 1);
    return avg_period > 0.0 ? 1.0 / avg_period : 0.0;
}

std::vector<double> ZNTuningUtils::calculatePowerSpectrum(const std::vector<double>& signal) {
    // Simplified power spectrum calculation (magnitude squared)
    std::vector<double> spectrum;
    if (signal.empty()) return spectrum;
    
    // For simplicity, return the signal magnitude squared
    spectrum.reserve(signal.size());
    for (double val : signal) {
        spectrum.push_back(val * val);
    }
    
    return spectrum;
}

bool ZNTuningUtils::isSystemStable(const std::vector<double>& response, double tolerance) {
    if (response.size() < 10) return false;
    
    // Check if the last 20% of the response is within tolerance of the final value
    std::size_t check_start = response.size() * 4 / 5;
    double final_value = response.back();
    
    for (std::size_t i = check_start; i < response.size(); ++i) {
        if (std::abs(response[i] - final_value) > tolerance) {
            return false;
        }
    }
    
    return true;
}

double ZNTuningUtils::calculateDampingRatio(const std::vector<double>& step_response) {
    if (step_response.size() < 10) return 0.0;
    
    // Find peaks to estimate damping ratio
    std::vector<double> peaks = findPeaks(step_response, 0.0);
    
    if (peaks.size() < 2) return 1.0; // Overdamped
    
    // Calculate logarithmic decrement
    double peak1 = step_response[static_cast<std::size_t>(peaks[0])];
    double peak2 = step_response[static_cast<std::size_t>(peaks[1])];
    
    if (peak1 <= 0.0 || peak2 <= 0.0) return 0.0;
    
    double log_decrement = std::log(peak1 / peak2);
    double damping_ratio = log_decrement / std::sqrt((4.0 * M_PI * M_PI) + (log_decrement * log_decrement));
    
    return std::max(0.0, std::min(1.0, damping_ratio));
}

double ZNTuningUtils::calculateNaturalFrequency(const std::vector<double>& step_response, double sample_rate) {
    if (step_response.size() < 10) return 0.0;
    
    // Estimate natural frequency from oscillation period
    std::vector<double> peaks = findPeaks(step_response, 0.0);
    
    if (peaks.size() < 2) return 0.0;
    
    // Calculate average period between peaks
    double total_period = 0.0;
    for (std::size_t i = 1; i < peaks.size(); ++i) {
        total_period += (peaks[i] - peaks[i-1]) / sample_rate;
    }
    
    double avg_period = total_period / (peaks.size() - 1);
    return avg_period > 0.0 ? (2.0 * M_PI) / avg_period : 0.0;
}

// BatchZNTuning Implementation
BatchZNTuning::BatchZNTuning(const std::vector<SystemConfig>& scenarios, 
                           const ZNTuningConfig& tuning_config,
                           const std::string& output_dir)
    : test_scenarios_(scenarios), base_tuning_config_(tuning_config), output_base_directory_(output_dir) {}

std::vector<ZNAnalysisResult> BatchZNTuning::runBatchTuning() {
    std::vector<ZNAnalysisResult> results;
    
    for (const auto& scenario : test_scenarios_) {
        std::cout << "Running tuning for scenario: " << scenarioToString(scenario.current_scenario) << std::endl;
        
        ZNAnalysisResult result = runScenarioTuning(scenario);
        results.push_back(result);
    }
    
    return results;
}

ZNAnalysisResult BatchZNTuning::runScenarioTuning(const SystemConfig& scenario) {
    MultiDroneSystem system(scenario);
    if (!system.initialize()) {
        return ZNAnalysisResult();  // Return empty result on failure
    }
    
    ZNTuningEngine tuning(&system, base_tuning_config_);
    return tuning.performAutoTuning();
}

void BatchZNTuning::printComparisonSummary(const std::vector<ZNAnalysisResult>& results) {
    std::cout << "\n=== BATCH TUNING SUMMARY ===" << std::endl;
    
    for (std::size_t i = 0; i < results.size() && i < test_scenarios_.size(); ++i) {
        const auto& result = results[i];
        const auto& scenario = test_scenarios_[i];
        
        std::cout << scenarioToString(scenario.current_scenario) << ": ";
        
        if (result.is_oscillating) {
            std::cout << "Ku = " << std::fixed << std::setprecision(2) << result.ultimate_gain
                      << ", Tu = " << std::setprecision(2) << result.period << "s";
        } else {
            std::cout << "No oscillation found";
        }
        
        std::cout << std::endl;
    }
}

void BatchZNTuning::generateComparisonReport(const std::vector<ZNAnalysisResult>& results, 
                                           const std::string& filename) {
    std::ofstream file(filename);
    if (!file.is_open()) return;
    
    file << "Batch ZN Tuning Comparison Report\n";
    file << "=================================\n\n";
    
    for (std::size_t i = 0; i < results.size() && i < test_scenarios_.size(); ++i) {
        const auto& result = results[i];
        const auto& scenario = test_scenarios_[i];
        
        file << "Scenario: " << scenarioToString(scenario.current_scenario) << "\n";
        
        if (result.is_oscillating) {
            file << "  Ultimate Gain: " << std::fixed << std::setprecision(3) << result.ultimate_gain << "\n";
            file << "  Ultimate Period: " << std::setprecision(3) << result.period << " seconds\n";
            
            if (!result.recommended_gains.empty()) {
                file << "  Recommended (Classical): Kp=" << std::setprecision(3) 
                     << result.recommended_gains[0].kp << ", Ki=" << result.recommended_gains[0].ki 
                     << ", Kd=" << result.recommended_gains[0].kd << "\n";
            }
        } else {
            file << "  No oscillation detected\n";
        }
        
        file << "\n";
    }
}

void BatchZNTuning::addScenario(const SystemConfig& scenario) {
    test_scenarios_.push_back(scenario);
}

// Factory functions
ZNTuningConfig createQuickTuningConfig() {
    ZNTuningConfig config;
    config.kp_start = 0.5;
    config.kp_end = 10.0;
    config.kp_step = 0.5;
    config.test_duration = 20.0;
    return config;
}

ZNTuningConfig createDetailedTuningConfig() {
    ZNTuningConfig config;
    config.kp_start = 0.1;
    config.kp_end = 15.0;
    config.kp_step = 0.2;
    config.test_duration = 30.0;
    config.save_detailed_results = true;
    return config;
}

ZNTuningConfig createConservativeTuningConfig() {
    ZNTuningConfig config;
    config.kp_start = 0.1;
    config.kp_end = 8.0;
    config.kp_step = 0.1;
    config.test_duration = 40.0;
    config.stability_tolerance = 0.02;
    return config;
}

ZNTuningConfig createAggressiveTuningConfig() {
    ZNTuningConfig config;
    config.kp_start = 1.0;
    config.kp_end = 20.0;
    config.kp_step = 0.5;
    config.test_duration = 15.0;
    config.oscillation_threshold = 0.05;
    return config;
}

} // namespace modular_pid_system 