#pragma once

#include "drone_system.hpp"
#include <vector>
#include <string>
#include <functional>

namespace modular_pid_system {

// Ziegler-Nichols Analysis Results
struct ZNAnalysisResult {
    bool is_unstable = false;
    bool is_oscillating = false;
    double period = 0.0;
    double overshoot_percent = 0.0;
    double settling_time_2percent = -1.0;
    double ultimate_gain = 0.0;
    double ultimate_period = 0.0;
    
    // Recommended PID gains based on ZN method
    struct RecommendedGains {
        double kp = 0.0;
        double ki = 0.0;
        double kd = 0.0;
        std::string method_name;
    };
    
    std::vector<RecommendedGains> recommended_gains;
};

// ZN Tuning Configuration
struct ZNTuningConfig {
    // Test parameters
    double kp_start = 0.1;
    double kp_end = 10.0;
    double kp_step = 0.1;
    double test_duration = 30.0;
    
    // Oscillation detection parameters
    double oscillation_threshold = 0.1;
    int min_oscillation_cycles = 3;
    double stability_tolerance = 0.05;
    
    // Target drone and axis for tuning
    int target_drone_id = 0;
    enum class TuningAxis { X_AXIS, Y_AXIS, BOTH };
    TuningAxis tuning_axis = TuningAxis::X_AXIS;
    
    // Formation center for testing
    double test_center_x = 5.0;
    double test_center_y = 5.0;
    
    // Output configuration
    bool save_detailed_results = true;
    std::string output_directory = "zn_tuning_results";
};

// ZN Tuning Engine
class ZNTuningEngine {
private:
    ZNTuningConfig config_;
    MultiDroneSystem* system_;
    
    // Analysis functions
    ZNAnalysisResult analyzeOscillations(const std::vector<std::pair<double, double>>& error_history);
    ZNAnalysisResult runSingleKpTest(double kp_test);
    void calculateRecommendedGains(ZNAnalysisResult& result);
    
    // Data collection
    std::vector<std::pair<double, double>> collectErrorData(double kp_test);
    
    // Utility functions
    bool detectOscillation(const std::vector<double>& signal, double& period, double& amplitude) const;
    static double calculateOvershoot(const std::vector<double>& response, double target_value);
    double calculateSettlingTime(const std::vector<double>& response, double target_value, double tolerance = 0.02) const;
    
public:
    ZNTuningEngine(MultiDroneSystem* system, const ZNTuningConfig& config);
    
    // Main tuning functions
    ZNAnalysisResult performAutoTuning();
    ZNAnalysisResult testSpecificKp(double kp_value);
    
    // Manual step-by-step tuning
    void startManualTuning();
    ZNAnalysisResult testNextKpValue();
    bool hasMoreKpValues() const;
    
    // Results and reporting
    void saveResults(const ZNAnalysisResult& result, const std::string& filename) const;
    static void printResults(const ZNAnalysisResult& result);
    static void generateTuningReport(const std::vector<ZNAnalysisResult>& all_results, const std::string& filename);
    
    // Configuration
    void setConfig(const ZNTuningConfig& config) { config_ = config; }
    const ZNTuningConfig& getConfig() const { return config_; }
};

// ZN Tuning Utilities
class ZNTuningUtils {
public:
    // Standard ZN tuning rules
    static ZNAnalysisResult::RecommendedGains calculateClassicPID(double ku, double tu);
    static ZNAnalysisResult::RecommendedGains calculatePessenIntegralRule(double ku, double tu);
    static ZNAnalysisResult::RecommendedGains calculateSomeOvershootRule(double ku, double tu);
    static ZNAnalysisResult::RecommendedGains calculateNoOvershootRule(double ku, double tu);
    
    // Signal analysis utilities
    static std::vector<double> findPeaks(const std::vector<double>& signal, double min_height = 0.0);
    static std::vector<double> findValleys(const std::vector<double>& signal, double max_depth = 0.0);
    static double calculateRMS(const std::vector<double>& signal);
    static double calculateMean(const std::vector<double>& signal);
    static std::vector<double> applyMovingAverage(const std::vector<double>& signal, int window_size);
    
    // Frequency domain analysis
    static double estimateDominantFrequency(const std::vector<double>& signal, double sample_rate);
    static std::vector<double> calculatePowerSpectrum(const std::vector<double>& signal);
    
    // Stability analysis
    static bool isSystemStable(const std::vector<double>& response, double tolerance = 0.05);
    static double calculateDampingRatio(const std::vector<double>& step_response);
    static double calculateNaturalFrequency(const std::vector<double>& step_response, double sample_rate);
};

// Batch Tuning for Multiple Scenarios
class BatchZNTuning {
private:
    std::vector<SystemConfig> test_scenarios_;
    ZNTuningConfig base_tuning_config_;
    std::string output_base_directory_;
    
public:
    BatchZNTuning(const std::vector<SystemConfig>& scenarios, 
                  const ZNTuningConfig& tuning_config,
                  const std::string& output_dir);
    
    // Run tuning for all scenarios
    std::vector<ZNAnalysisResult> runBatchTuning();
    
    // Run tuning for specific scenario
    ZNAnalysisResult runScenarioTuning(const SystemConfig& scenario);
    
    // Comparison and analysis
    void generateComparisonReport(const std::vector<ZNAnalysisResult>& results, 
                                const std::string& filename);
    void printComparisonSummary(const std::vector<ZNAnalysisResult>& results);
    
    // Configuration
    void addScenario(const SystemConfig& scenario);
    void setTuningConfig(const ZNTuningConfig& config) { base_tuning_config_ = config; }
};

// Factory functions for common tuning configurations
ZNTuningConfig createQuickTuningConfig();
ZNTuningConfig createDetailedTuningConfig();
ZNTuningConfig createConservativeTuningConfig();
ZNTuningConfig createAggressiveTuningConfig();

} // namespace modular_pid_system 