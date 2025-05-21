#include "agent_control_pkg/gt2_fuzzy_logic_system.hpp"
#include <iostream>
#include <iomanip> // For std::fixed, std::setprecision

void print_fuzzified_values(const std::string& var_name, 
                            const std::map<std::string, agent_control_pkg::GT2FuzzyLogicSystem::MembershipInterval>& memberships) {
    std::cout << "  Fuzzified values for '" << var_name << "':" << std::endl;
    if (memberships.empty()) {
        std::cout << "    No fuzzy sets defined or memberships calculated for this variable." << std::endl;
        return;
    }
    for (const auto& pair : memberships) {
        std::cout << "    Set '" << pair.first << "': UpperMU=" << pair.second.second 
                  << ", LowerMU=" << pair.second.first << std::endl;
    }
}


int main() {
    std::cout << std::fixed << std::setprecision(3);
    std::cout << "--- Fuzzy Logic System Standalone Test (Comprehensive Sets) ---" << std::endl;

    agent_control_pkg::GT2FuzzyLogicSystem fls;

    // --- Define Input/Output Variables ---
    fls.addInputVariable("error");    // e.g., universe -10 to 10
    fls.addInputVariable("dError");   // e.g., universe -5 to 5
    fls.addInputVariable("wind");     // e.g., universe -10 (strong tailwind) to 10 (strong headwind)
    fls.addOutputVariable("correction"); // e.g., universe -5 to 5 (correction to PID output)

    // --- Define Fuzzy Sets for "error" (Example: 5 sets) ---
    // NB (Negative Big)
    fls.addFuzzySetToVariable("error", "NB", {-10.0, -10.0, -5.0,  -11.0, -10.0, -4.0});
    // NS (Negative Small)
    fls.addFuzzySetToVariable("error", "NS", {-6.0, -3.0, 0.0,    -7.0, -3.0, 1.0});
    // ZE (Zero)
    fls.addFuzzySetToVariable("error", "ZE", {-1.0, 0.0, 1.0,    -2.0, 0.0, 2.0});
    // PS (Positive Small)
    fls.addFuzzySetToVariable("error", "PS", {0.0, 3.0, 6.0,     -1.0, 3.0, 7.0});
    // PB (Positive Big)
    fls.addFuzzySetToVariable("error", "PB", {5.0, 10.0, 10.0,   4.0, 10.0, 11.0});

    // --- Define Fuzzy Sets for "dError" (Example: 3 sets) ---
    // NE (Negative dError - error increasing negatively or decreasing positively)
    fls.addFuzzySetToVariable("dError", "NE", {-5.0, -5.0, 0.0,    -6.0, -5.0, 1.0});
    // ZE (Zero dError - error is stable)
    fls.addFuzzySetToVariable("dError", "ZE", {-1.0, 0.0, 1.0,    -1.5, 0.0, 1.5});
    // PO (Positive dError - error increasing positively or decreasing negatively)
    fls.addFuzzySetToVariable("dError", "PO", {0.0, 5.0, 5.0,     -1.0, 5.0, 6.0});

    // --- Define Fuzzy Sets for "wind" (Example: 5 sets) ---
    // SN (Strong Negative - strong tailwind pushing drone forward)
    fls.addFuzzySetToVariable("wind", "SN", {-10.0, -10.0, -5.0,  -11.0, -10.0, -4.0});
    // WN (Weak Negative - weak tailwind)
    fls.addFuzzySetToVariable("wind", "WN", {-6.0, -3.0, 0.0,    -7.0, -3.0, 1.0});
    // NW (No Wind)
    fls.addFuzzySetToVariable("wind", "NW", {-1.0, 0.0, 1.0,    -1.5, 0.0, 1.5});
    // WP (Weak Positive - weak headwind resisting drone)
    fls.addFuzzySetToVariable("wind", "WP", {0.0, 3.0, 6.0,     -1.0, 3.0, 7.0});
    // SP (Strong Positive - strong headwind)
    fls.addFuzzySetToVariable("wind", "SP", {5.0, 10.0, 10.0,   4.0, 10.0, 11.0});

    // --- Define Fuzzy Sets for "correction" (Output - Example: 5 sets) ---
    // LNC (Large Negative Correction - reduce PID output significantly)
    fls.addFuzzySetToVariable("correction", "LNC", {-5.0, -5.0, -2.5,   -5.5, -5.0, -2.0});
    // SNC (Small Negative Correction)
    fls.addFuzzySetToVariable("correction", "SNC", {-3.0, -1.5, 0.0,    -3.5, -1.5, 0.5});
    // NC (No Correction)
    fls.addFuzzySetToVariable("correction", "NC",  {-0.5, 0.0, 0.5,    -1.0, 0.0, 1.0});
    // SPC (Small Positive Correction)
    fls.addFuzzySetToVariable("correction", "SPC", {0.0, 1.5, 3.0,     -0.5, 1.5, 3.5});
    // LPC (Large Positive Correction - increase PID output significantly)
    fls.addFuzzySetToVariable("correction", "LPC", {2.5, 5.0, 5.0,      2.0, 5.0, 5.5});


    // --- Define Some Example Rules ---
    // Rule 1: IF error is ZE AND dError is ZE AND wind is NW THEN correction is NC
    fls.addRule({ {{"error", "ZE"}, {"dError", "ZE"}, {"wind", "NW"}}, {"correction", "NC"} });
    
    // Rule 2: IF error is ZE AND dError is ZE AND wind is SP THEN correction is LNC 
    // (If on target, but strong headwind, provide negative correction to PID to add more "thrust" - assumes PID output is acceleration command)
    // (Correction might be counter-intuitive: if PID out is +ve for fwd, and wind is +ve (headwind), FLS correction should be +ve to add more effort)
    // Let's rephrase: Positive wind means a force *against* the drone if drone wants to move positive.
    // So if drone is on target and strong positive wind, it needs more positive thrust. So FLS correction should be Positive.
    fls.addRule({ {{"error", "ZE"}, {"dError", "ZE"}, {"wind", "SP"}}, {"correction", "LPC"} });

    // Rule 3: IF error is PS AND dError is ZE AND wind is NW THEN correction is SNC
    fls.addRule({ {{"error", "PS"}, {"dError", "ZE"}, {"wind", "NW"}}, {"correction", "SNC"} });
    
    // Rule 4: IF error is PB AND wind is SP THEN correction is LPC // Needs strong positive correction
    fls.addRule({ {{"error", "PB"}, {"wind", "SP"}}, {"correction", "LPC"} });


    // --- Test with some sample inputs ---
    // We can manually call fuzzifyInputs here to see its output if we want,
    // or just let calculateOutput call it internally.

    std::cout << "\nTest Case 1: (Error near zero, no dError, no wind)" << std::endl;
    double output1 = fls.calculateOutput(0.1, 0.0, 0.05); 
    // Since evaluateRulesAndAggregate is a placeholder, this will still be dummy.
    // But fuzzifyInputs will have run with these values.
    std::cout << "Test Case 1 Final Output (placeholder logic): " << output1 << std::endl;

    std::cout << "\nTest Case 2: (Error near zero, no dError, strong positive wind)" << std::endl;
    double output2 = fls.calculateOutput(0.1, 0.0, 7.0); // 7.0 should fall into "SP" wind
    std::cout << "Test Case 2 Final Output (placeholder logic): " << output2 << std::endl;

    std::cout << "\nTest Case 3: (Positive Small error, no dError, no wind)" << std::endl;
    double output3 = fls.calculateOutput(2.0, 0.0, 0.0); // 2.0 should fall into "PS" error
    std::cout << "Test Case 3 Final Output (placeholder logic): " << output3 << std::endl;
    
    std::cout << "\n--- FLS Test End ---" << std::endl;
    return 0;
}