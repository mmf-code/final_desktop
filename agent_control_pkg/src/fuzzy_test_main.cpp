#include "agent_control_pkg/gt2_fuzzy_logic_system.hpp"
#include <iostream>
#include <vector>

int main() {
    std::cout << "--- Fuzzy Logic System Standalone Test ---" << std::endl;

    agent_control_pkg::GT2FuzzyLogicSystem fls;

    // --- Define Input/Output Variables (placeholder for loading from config) ---
    fls.addInputVariable("error");
    fls.addInputVariable("dError");
    fls.addInputVariable("wind");
    fls.addOutputVariable("correction");

    // --- Define Fuzzy Sets (Example for "error" variable, set "Zero") ---
    // FOU for "Zero Error": LMF: (-1, 0, 1), UMF: (-1.5, 0, 1.5)
    agent_control_pkg::GT2FuzzyLogicSystem::IT2TriangularFS_FOU zero_error_fou = 
        {-1.0, 0.0, 1.0,  // Lower MF: left_base, peak, right_base
         -1.5, 0.0, 1.5}; // Upper MF: left_base, peak, right_base
    fls.addFuzzySetToVariable("error", "Zero", zero_error_fou);

    // ... You would define more sets for "error", "dError", "wind", and "correction" ...
    // Example for "PositiveSmall Error"
    agent_control_pkg::GT2FuzzyLogicSystem::IT2TriangularFS_FOU ps_error_fou = 
        {0.5, 1.5, 2.5,
         0.0, 1.5, 3.0};
    fls.addFuzzySetToVariable("error", "PositiveSmall", ps_error_fou);

    // Example for "NoWind"
    agent_control_pkg::GT2FuzzyLogicSystem::IT2TriangularFS_FOU no_wind_fou = 
        {-0.5, 0.0, 0.5,
         -1.0, 0.0, 1.0};
    fls.addFuzzySetToVariable("wind", "NoWind", no_wind_fou);
    
    // Example for output set "NoCorrection"
    agent_control_pkg::GT2FuzzyLogicSystem::IT2TriangularFS_FOU nc_correction_fou =
        {-0.2, 0.0, 0.2,
         -0.3, 0.0, 0.3};
    fls.addFuzzySetToVariable("correction", "NoCorrection", nc_correction_fou);


    // --- Define Rules (Example simplified rule) ---
    agent_control_pkg::GT2FuzzyLogicSystem::FuzzyRule rule1;
    rule1.antecedents = {{"error", "Zero"}, {"wind", "NoWind"}}; // IF error is Zero AND wind is NoWind
    rule1.consequent = {"correction", "NoCorrection"};          // THEN correction is NoCorrection
    fls.addRule(rule1);
    // ... You would add more comprehensive rules ...


    // --- Test with some sample inputs ---
    std::cout << "\nTest Case 1:" << std::endl;
    double output1 = fls.calculateOutput(0.1, 0.0, 0.05); // error=0.1, dError=0, wind=0.05
    std::cout << "Test Case 1 Final Output: " << output1 << std::endl;

    std::cout << "\nTest Case 2:" << std::endl;
    double output2 = fls.calculateOutput(1.0, 0.1, 0.0); // error=1.0 (should hit "PositiveSmall" more)
    std::cout << "Test Case 2 Final Output: " << output2 << std::endl;
    
    std::cout << "\n--- FLS Test End ---" << std::endl;
    return 0;
}