/**
 * fuzzy_test_main.cpp - Standalone test for GT2FuzzyLogicSystem
 * Focus: Defining Fuzzy Sets, a comprehensive Rule Base, testing Fuzzification,
 *        and generating data for plotting Membership Functions & a Control Surface.
 */
#include "agent_control_pkg/gt2_fuzzy_logic_system.hpp" // Your FLS header
#include <iostream>
#include <iomanip>    // For std::fixed, std::setprecision
#include <cmath>      // For std::sqrt (if used in FOU definitions)
#include <vector>     // For storing scenarios
#include <map>        // For std::map
#include <string>     // For std::string
#include <fstream>    // For CSV file output

// Helper function to print the results of fuzzification for a given input variable
void print_fuzzified_variable_results(
    const std::string& input_variable_name,
    double crisp_value,
    const std::map<std::string, agent_control_pkg::GT2FuzzyLogicSystem::MembershipInterval>& fuzzified_sets_for_variable)
{
    std::cout << "  Fuzzification for Input Variable: '" << input_variable_name << "' (Crisp Value: " << crisp_value << ")" << std::endl;
    if (fuzzified_sets_for_variable.empty()) {
        std::cout << "    No fuzzified sets found for this variable." << std::endl;
        return;
    }
    for (const auto& set_pair : fuzzified_sets_for_variable) {
        std::cout << "    Set '" << set_pair.first << "': mu_interval = ["
                  << set_pair.second.first << ", "
                  << set_pair.second.second << "]" << std::endl;
    }
}

int main() {
    std::cout << std::fixed << std::setprecision(3)
              << "--- GT2-FLS Test with CSV Data Generation ---" << std::endl;

    using FOU = agent_control_pkg::GT2FuzzyLogicSystem::IT2TriangularFS_FOU;
    agent_control_pkg::GT2FuzzyLogicSystem fls;

    // --- Define Input/Output Variables ---
    fls.addInputVariable ("error");
    fls.addInputVariable ("dError");
    fls.addInputVariable ("wind");
    fls.addOutputVariable("correction");

    // --- Define Fuzzy Sets (as you had them) ---
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


    // --- Define FLS Rules (Your 21 rules, with modifications for wind) ---
    std::cout << "\n--- Defining FLS Rules ---" << std::endl;
    auto R = [&](const std::string& e, const std::string& de, const std::string& w, const std::string& out){
        fls.addRule({{{"error",e},{"dError",de},{"wind",w}}, {"correction",out}});
    };
    // No wind (NWN) - Damping rules
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

    // Wind scenarios
    // When error is ZE (Zero Error) and dError is DZ (Zero Change in Error)
    // Stronger direct counteraction for wind:
    R("ZE","DZ","SPW","LNC");  // Strong Positive Wind -> Large Negative Correction
    R("ZE","DZ","SNW","LPC");  // Strong Negative Wind -> Large Positive Correction
    // Weaker direct counteraction for weak wind (could also be SNC/SPC if LNC/LPC is too much for weak wind)
    R("ZE","DZ","WPW","SNC");  // Weak Positive Wind -> Small Negative Correction
    R("ZE","DZ","WNW","SPC");  // Weak Negative Wind -> Small Positive Correction

    // Rules when error is present WITH wind (these might need more thought/tuning)
    // Example: If drone is already far behind target (PB) and wind pushes it further (SPW),
    // then an even stronger positive command might be needed than just XLPC.
    // Or if wind helps, maybe less than XLPC. For now, keeping your original:
    R("PB","DZ","SPW","XLPC"); // Error PB, Strong Positive Wind -> XLPC
    R("NB","DZ","SNW","XLNC"); // Error NB, Strong Negative Wind -> XLNC

    std::cout << "Total rules defined: " << fls.getRuleCount() << std::endl;

    // --- 1. Generate Data for Plotting Membership Functions ---
    // ... (This part of your code remains the same) ...
    std::ofstream mf_data_file("fls_mf_data.csv");
    if (!mf_data_file.is_open()) {
        std::cerr << "Error: Could not open fls_mf_data.csv" << std::endl;
    } else {
        mf_data_file << "InputVar,SetName,CrispInput,LowerMu,UpperMu\n";
        std::vector<std::string> input_vars_for_mf_plot = {"error", "dError", "wind"};
        std::map<std::string, std::pair<double, double>> var_plot_ranges = {
            {"error", {-12.0, 12.0}}, {"dError", {-7.0, 7.0}}, {"wind", {-12.0, 12.0}}
        };
        double mf_plot_step = 0.1;

        std::cout << "\n--- Generating Membership Function Data for CSV ---" << std::endl;
        for (const std::string& var_name : input_vars_for_mf_plot) {
            if (fls.getFuzzySets().count(var_name)) {
                for (double val = var_plot_ranges[var_name].first; val <= var_plot_ranges[var_name].second; val += mf_plot_step) {
                    const auto& sets_for_var = fls.getFuzzySets().at(var_name);
                    for (const auto& set_entry : sets_for_var) {
                        const std::string& set_name = set_entry.first;
                        const FOU& fou = set_entry.second;
                        agent_control_pkg::GT2FuzzyLogicSystem::MembershipInterval mi = fls.getTriangularMembership(val,
                            fou.lmf_left_base, fou.lmf_peak, fou.lmf_right_base,
                            fou.umf_left_base, fou.umf_peak, fou.umf_right_base
                        );
                         mf_data_file << var_name << "," << set_name << "," << val << ","
                                     << mi.first << "," << mi.second << "\n";
                    }
                }
            }
        }
        mf_data_file.close();
        std::cout << "Membership function data written to fls_mf_data.csv" << std::endl;
    }

    // --- 2. Generate Data for Control Surface Plot (e.g., error vs wind -> correction, dError fixed) ---
    // ... (This part of your code remains the same) ...
    std::ofstream surface_data_file("fls_surface_data.csv");
    if (!surface_data_file.is_open()) {
        std::cerr << "Error: Could not open fls_surface_data.csv" << std::endl;
    } else {
        surface_data_file << "Error,Wind,dError_fixed,Correction\n";
        double fixed_dError = 0.0; // Fix one input
        double error_min = -10.0, error_max = 10.0, error_step = 0.5;
        double wind_min = -10.0, wind_max = 10.0, wind_step = 0.5;

        std::cout << "\n--- Generating Control Surface Data for CSV ---" << std::endl;
        for (double e_val = error_min; e_val <= error_max; e_val += error_step) {
            for (double w_val = wind_min; w_val <= wind_max; w_val += wind_step) {
                double correction_output = fls.calculateOutput(e_val, fixed_dError, w_val);
                surface_data_file << e_val << "," << w_val << "," << fixed_dError << "," << correction_output << "\n";
            }
        }
        surface_data_file.close();
        std::cout << "Control surface data written to fls_surface_data.csv" << std::endl;
    }

    // --- Original Specific Test Scenarios (for console verification) ---
    // ... (This part of your code remains the same) ...
    std::cout << "\n--- Running Original Specific Test Scenarios ---" << std::endl;
    struct Scenario { double e, de, w; const char* name; };
    std::vector<Scenario> tests = {
        {0.1,   0.0,   0.05, "Test1: Error ZE, dError DZ, Wind NWN (Rule ZE,DZ,NWN -> NC)"}, // Updated expected rule
        {0.1,   0.0,   7.0,  "Test2: Error ZE, dError DZ, Wind SPW (Rule ZE,DZ,SPW -> LNC)"}, // Updated expected rule
        {7.0,   0.0,   7.0,  "Test3: Error PB, dError DZ, Wind SPW (Rule PB,DZ,SPW -> XLPC)"},
        {-7.0,  0.0,  -7.0,  "Test4: Error NB, dError DZ, Wind SNW (Rule NB,DZ,SNW -> XLNC)"},
        {2.5,   0.0,   0.0,  "Test5: Error PS, dError DZ, Wind NWN (Rule PS,DZ,NWN -> SPC)"}
    };
     for (const auto& s : tests) {
        std::cout << "\n--- Testing Scenario: " << s.name << " ---" << std::endl;
        std::cout << "Crisp Inputs: error=" << s.e << ", dError=" << s.de << ", wind=" << s.w << std::endl;
        double u = fls.calculateOutput(s.e, s.de, s.w);
        std::cout << "  FLS Crisp Output = " << u << '\n';
    }

    std::cout << "\n--- FLS Data Generation End ---\n";
    return 0;
}