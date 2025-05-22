/**
 * fuzzy_test_main.cpp - Standalone test for GT2FuzzyLogicSystem
 * Focus: Defining Fuzzy Sets and testing the Fuzzification stage.
 */
#include "agent_control_pkg/gt2_fuzzy_logic_system.hpp" // Your FLS header
#include <iostream>
#include <iomanip> // For std::fixed, std::setprecision
#include <cmath>   // For std::sqrt (used in your example, though not directly in FLS MFs here)
#include <vector>  // For storing scenarios
#include <map>     // For std::map
#include <string>  // For std::string

// Helper function to print the results of fuzzification for a given input variable
void print_fuzzified_variable_results(
    const std::string& input_variable_name,
    double crisp_value, // Also print the crisp value for context
    const std::map<std::string, agent_control_pkg::GT2FuzzyLogicSystem::MembershipInterval>& fuzzified_sets_for_variable) 
{
    std::cout << "  Fuzzification for Input Variable: '" << input_variable_name << "' (Crisp Value: " << crisp_value << ")" << std::endl;
    if (fuzzified_sets_for_variable.empty()) {
        std::cout << "    No fuzzified sets found for this variable." << std::endl;
        return;
    }
    for (const auto& set_pair : fuzzified_sets_for_variable) { // set_pair is (string set_name, MembershipInterval interval)
        std::cout << "    Set '" << set_pair.first << "': mu_interval = ["
                  << set_pair.second.first << ", "
                  << set_pair.second.second << "]" << std::endl;
    }
}

// Example function for theta (if you want to keep it for future use)
// This function is NOT used by the FLS directly unless you modify FLS to use it.
double example_updateThetaByRef(double ref_m) {
    if (ref_m < 0.85)      return -0.8;
    else if (ref_m > 1.25) return  1.0;
    else                   return  0.1;
}

int main() {
    std::cout << std::fixed << std::setprecision(3)
              << "--- GT2-FLS Fuzzification Test ---" << std::endl;

    using FOU = agent_control_pkg::GT2FuzzyLogicSystem::IT2TriangularFS_FOU;
    agent_control_pkg::GT2FuzzyLogicSystem fls;

    // --- Global Parameters (placeholders for now, as FLS core doesn't use them yet) ---
    // const int    ALPHA_PLANES = 4; 
    // double       theta        = 0.1; 
    // fls.setAlphaPlaneCount(ALPHA_PLANES); // Assuming these methods exist in your FLS class
    // fls.setShapeParameter(theta);       // but their effect is not yet implemented

    // --- Define Input/Output Variables ---
    fls.addInputVariable ("error");
    fls.addInputVariable ("dError"); 
    fls.addInputVariable ("wind");
    fls.addOutputVariable("correction"); 

    // --- Define Fuzzy Sets for "error" (Universe: e.g., -10 to 10) ---
    fls.addFuzzySetToVariable("error","NB",FOU{-10.0, -8.0, -6.0,  -11.0, -8.5, -5.5}); // Negative Big
    fls.addFuzzySetToVariable("error","NS",FOU{ -7.0, -4.0, -1.0,   -8.0, -4.5,  0.0}); // Negative Small
    fls.addFuzzySetToVariable("error","ZE",FOU{ -1.0,  0.0,  1.0,   -2.0,  0.0,  2.0}); // Zero
    fls.addFuzzySetToVariable("error","PS",FOU{  1.0,  4.0,  7.0,    0.0,  4.5,  8.0}); // Positive Small
    fls.addFuzzySetToVariable("error","PB",FOU{  6.0,  8.0, 10.0,    5.5,  8.5, 11.0}); // Positive Big

    // --- Define Fuzzy Sets for "dError" (Universe: e.g., -5 to 5) ---
    fls.addFuzzySetToVariable("dError","DN",FOU{-5.0, -3.0,  0.0,   -6.0, -3.5,  1.0}); // Decreasing Negative
    fls.addFuzzySetToVariable("dError","DZ",FOU{-1.0,  0.0,  1.0,   -1.5,  0.0,  1.5}); // Zero Change
    fls.addFuzzySetToVariable("dError","DP",FOU{ 0.0,  3.0,  5.0,   -1.0,  3.0,  6.0}); // Increasing Positive

    // --- Define Fuzzy Sets for "wind" (Universe: e.g., -10 to 10) ---
    fls.addFuzzySetToVariable("wind","SNW",FOU{-10.0, -8.0, -5.0,  -11.0, -8.5, -4.5}); // Strong Negative Wind
    fls.addFuzzySetToVariable("wind","WNW",FOU{ -7.0, -4.0, -1.0,   -8.0, -4.5,  0.0}); // Weak Negative Wind
    fls.addFuzzySetToVariable("wind","NWN",FOU{ -1.0,  0.0,  1.0,   -1.5,  0.0,  1.5}); // No Wind
    fls.addFuzzySetToVariable("wind","WPW",FOU{  1.0,  4.0,  7.0,    0.0,  4.5,  8.0}); // Weak Positive Wind
    fls.addFuzzySetToVariable("wind","SPW",FOU{  6.0,  8.0, 10.0,    5.5,  8.5, 11.0}); // Strong Positive Wind

    // --- Define Fuzzy Sets for "correction" (Output - Universe: e.g., -5 to 5) ---
    fls.addFuzzySetToVariable("correction","LNC",FOU{-5.0, -4.0, -2.5,  -5.5, -4.5, -2.0}); // Large Negative
    fls.addFuzzySetToVariable("correction","SNC",FOU{-3.0, -1.5,  0.0,  -3.5, -1.5,  0.5}); // Small Negative
    fls.addFuzzySetToVariable("correction","NC", FOU{-0.5,  0.0,  0.5,  -1.0,  0.0,  1.0}); // No Correction
    fls.addFuzzySetToVariable("correction","SPC",FOU{ 0.0,  1.5,  3.0,  -0.5,  1.5,  3.5}); // Small Positive
    fls.addFuzzySetToVariable("correction","LPC",FOU{ 2.5,  4.0,  5.0,   2.0,  4.5,  5.5}); // Large Positive

    // --- Define Rules (Placeholders for now, as fuzzification is the focus) ---
    // Rule 1: IF error is ZE AND dError is DZ AND wind is NWN THEN correction is NC
    fls.addRule({{{"error","ZE"},{"dError","DZ"},{"wind","NWN"}}, {"correction","NC"}});
    // Rule 2: IF error is ZE AND dError is DZ AND wind is SPW THEN correction is LNC 
    fls.addRule({{{"error","ZE"},{"dError","DZ"},{"wind","SPW"}}, {"correction","LNC"}});


    // --- Test Scenarios for Fuzzification ---
    struct Scenario { double e, de, w; const char* name; };
    std::vector<Scenario> tests = {
        {0.1,   0.0,   0.05, "Error near ZE, dError DZ, Wind near NWN"},    
        {-7.0, -2.0,  -8.0, "Error NB/NS, dError DN, Wind SNW"}, 
        {2.5,   0.5,   6.5,  "Error PS, dError DZ/DP, Wind SPW"},
        {10.0,  5.0,  10.0,  "Inputs at max of some sets (Error PB, dError DP, Wind SPW)"},
        {-1.0,  0.0,   0.0, "Error at ZE/NS boundary"} // Test boundary condition
    };

    for (const auto& s : tests) {
        std::cout << "\n--- Testing Scenario: " << s.name << " ---" << std::endl;
        std::cout << "Crisp Inputs: error=" << s.e << ", dError=" << s.de << ", wind=" << s.w << std::endl;

        // Call fuzzifyInputs directly for inspection 
        // IMPORTANT: Assumes fuzzifyInputs is temporarily public in GT2FuzzyLogicSystem.hpp
        auto all_fuzzified_inputs = fls.fuzzifyInputs(s.e, s.de, s.w); 
                                        
        std::cout << "--- Fuzzified Values for Test Case ---" << std::endl;
        
        if (all_fuzzified_inputs.count("error")) {
            print_fuzzified_variable_results("error", s.e, all_fuzzified_inputs.at("error"));
        } else {
            std::cout << "  No fuzzified results for 'error'." << std::endl;
        }

        if (all_fuzzified_inputs.count("dError")) {
            print_fuzzified_variable_results("dError", s.de, all_fuzzified_inputs.at("dError"));
        } else {
            std::cout << "  No fuzzified results for 'dError'." << std::endl;
        }

        if (all_fuzzified_inputs.count("wind")) {
            print_fuzzified_variable_results("wind", s.w, all_fuzzified_inputs.at("wind"));
        } else {
            std::cout << "  No fuzzified results for 'wind'." << std::endl;
        }
        std::cout << "------------------------------------" << std::endl;


        // Call the full calculateOutput to see the placeholder output
        // This will internally call the (now more complete) fuzzifyInputs again
        std::cout << "\nCalling full calculateOutput (will re-fuzzify and use placeholder rule logic):" << std::endl;
        double u = fls.calculateOutput(s.e, s.de, s.w); 
        std::cout << "  FLS Crisp Output (from placeholder rule logic) = " << u << '\n';
    }
    std::cout << "\n--- Fuzzification Test End ---\n";
    return 0;
}