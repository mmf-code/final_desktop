#include "agent_control_pkg/gt2_fuzzy_logic_system.hpp"
#include <iostream> // For placeholder outputs
#include <algorithm> // For std::min, std::max
#include <cmath>     // For std::abs, etc.

namespace agent_control_pkg
{

GT2FuzzyLogicSystem::GT2FuzzyLogicSystem() {
    // In a real system, load MFs and rules here, or provide methods to add them.
    // For this placeholder, we might add some default test MFs/Rules later if needed for fuzzy_test_main
    std::cout << "GT2FuzzyLogicSystem: Placeholder constructor." << std::endl;
}

GT2FuzzyLogicSystem::~GT2FuzzyLogicSystem() {}

// --- Configuration Method Placeholders ---
void GT2FuzzyLogicSystem::addInputVariable(const std::string& var_name) {
    // Placeholder: Initialize an entry for this variable if not present
    if (fuzzy_sets_.find(var_name) == fuzzy_sets_.end()) {
        fuzzy_sets_[var_name] = {};
        std::cout << "GT2FLS: Added input variable: " << var_name << std::endl;
    }
}
void GT2FuzzyLogicSystem::addOutputVariable(const std::string& var_name) {
    // Placeholder: Similar to addInputVariable
     if (fuzzy_sets_.find(var_name) == fuzzy_sets_.end()) {
        fuzzy_sets_[var_name] = {};
        std::cout << "GT2FLS: Added output variable: " << var_name << std::endl;
    }
}

void GT2FuzzyLogicSystem::addFuzzySetToVariable(const std::string& var_name, const std::string& set_name, const IT2TriangularFS_FOU& fou) {
    // Placeholder: Store the FOU for the given variable and set name
    if (fuzzy_sets_.count(var_name)) {
        fuzzy_sets_[var_name][set_name] = fou;
        std::cout << "GT2FLS: Added set '" << set_name << "' to variable '" << var_name << "'" << std::endl;
    } else {
        std::cerr << "GT2FLS: Error - Variable '" << var_name << "' not defined before adding set." << std::endl;
    }
}
void GT2FuzzyLogicSystem::addRule(const FuzzyRule& rule) {
    // Placeholder: Add the rule to the rule base
    rules_.push_back(rule);
    std::cout << "GT2FLS: Added a rule. Total rules: " << rules_.size() << std::endl;
}


// Helper for calculating membership for a triangular FOU
// Returns [lower_mu, upper_mu]
GT2FuzzyLogicSystem::MembershipInterval GT2FuzzyLogicSystem::getTriangularMembership(
    double crisp_input, 
    double l_left, double l_peak, double l_right,
    double u_left, double u_peak, double u_right) 
{
    double lower_mu = 0.0;
    double upper_mu = 0.0;

    // Lower Membership Function (Triangle)
    if (crisp_input >= l_left && crisp_input <= l_peak) {
        if (l_peak - l_left > 1e-6) // Avoid division by zero
            lower_mu = (crisp_input - l_left) / (l_peak - l_left);
        else if (std::abs(crisp_input - l_peak) < 1e-6) // If it's a spike
             lower_mu = 1.0;
    } else if (crisp_input > l_peak && crisp_input <= l_right) {
        if (l_right - l_peak > 1e-6) // Avoid division by zero
            lower_mu = (l_right - crisp_input) / (l_right - l_peak);
         else if (std::abs(crisp_input - l_peak) < 1e-6) // If it's a spike
             lower_mu = 1.0;
    }

    // Upper Membership Function (Triangle)
    if (crisp_input >= u_left && crisp_input <= u_peak) {
         if (u_peak - u_left > 1e-6)
            upper_mu = (crisp_input - u_left) / (u_peak - u_left);
        else if (std::abs(crisp_input - u_peak) < 1e-6)
            upper_mu = 1.0;
    } else if (crisp_input > u_peak && crisp_input <= u_right) {
        if (u_right - u_peak > 1e-6)
            upper_mu = (u_right - crisp_input) / (u_right - u_peak);
        else if (std::abs(crisp_input - u_peak) < 1e-6)
            upper_mu = 1.0;
    }
    
    // Ensure lower_mu <= upper_mu (can happen with overlapping UMF/LMF if not careful with FOU definition)
    // For a valid FOU, UMF should always be >= LMF
    if (lower_mu > upper_mu) {
        // This indicates an issue with FOU definition or calculation.
        // For robustness, can swap or log error. Here, let's just clamp.
        // std::cerr << "Warning: LMF > UMF in getTriangularMembership. Clamping." << std::endl;
        lower_mu = std::min(lower_mu, upper_mu); // Or just set lower_mu = upper_mu
    }

    return {lower_mu, upper_mu};
}


// --- Placeholder FLS Stage Implementations ---
std::map<std::string, std::map<std::string, GT2FuzzyLogicSystem::MembershipInterval>>
GT2FuzzyLogicSystem::fuzzifyInputs(double error, double dError, double wind) {
    std::map<std::string, std::map<std::string, MembershipInterval>> fuzzified_values;
    std::cout << "GT2FLS: Fuzzify (Placeholder) - Error=" << error << ", dError=" << dError << ", Wind=" << wind << std::endl;

    // Example: For each input variable ("error", "dError", "wind")
    // For each fuzzy set defined for that variable (e.g., "error" -> "PositiveSmall")
    // Calculate membership interval using getTriangularMembership or similar
    // fuzzified_values["error"]["PositiveSmall"] = getTriangularMembership(error, ...params for PS error set...);
    // fuzzified_values["dError"]["Zero"] = getTriangularMembership(dError, ...params for Z dError set...);
    // fuzzified_values["wind"]["NoWind"] = getTriangularMembership(wind, ...params for NW wind set...);

    // For now, just return empty or a dummy value
    // Let's return a dummy interval for one set of one input to test structure
    if (fuzzy_sets_.count("error") && fuzzy_sets_["error"].count("Zero")) {
         const auto& fou_error_zero = fuzzy_sets_["error"]["Zero"];
         fuzzified_values["error"]["Zero"] = getTriangularMembership(error,
            fou_error_zero.lmf_left_base, fou_error_zero.lmf_peak, fou_error_zero.lmf_right_base,
            fou_error_zero.umf_left_base, fou_error_zero.umf_peak, fou_error_zero.umf_right_base
         );
    } else {
         fuzzified_values["error"]["Zero"] = {0.0, 0.0}; // Default if not defined
    }
    // ... Do similarly for dError and wind, and for all their sets ...

    return fuzzified_values;
}

GT2FuzzyLogicSystem::MembershipInterval
GT2FuzzyLogicSystem::evaluateRulesAndAggregate(const std::map<std::string, std::map<std::string, MembershipInterval>>& fuzzified_inputs) {
    std::cout << "GT2FLS: Evaluate Rules & Aggregate (Placeholder)" << std::endl;
    // This is complex. For each rule:
    // 1. Get firing interval of antecedents (using t-norm, e.g., min for lower, min for upper bounds)
    // 2. Apply firing interval to rule's consequent fuzzy set (implication, e.g., min product)
    // 3. Aggregate all fired rule consequents (using t-conorm, e.g., max for lower, max for upper bounds)
    // For now, return a dummy aggregated output interval
    return {0.2, 0.8}; // Example dummy aggregated output FOU (as a simple interval)
}

std::pair<double, double>
GT2FuzzyLogicSystem::typeReduce(const MembershipInterval& aggregated_output_interval) {
    std::cout << "GT2FLS: Type Reduce (Placeholder)" << std::endl;
    // Implement Karnik-Mendel or similar iterative algorithm.
    // This is the most mathematically intensive part for IT2FLS.
    // For now, let's just return the input interval (which is not correct type reduction)
    // or its average as a point, then make it a small interval.
    double center = (aggregated_output_interval.first + aggregated_output_interval.second) / 2.0;
    return {center - 0.05, center + 0.05}; // Example dummy type-reduced interval
}

double GT2FuzzyLogicSystem::defuzzify(const std::pair<double, double>& crisp_interval) {
    std::cout << "GT2FLS: Defuzzify (Placeholder)" << std::endl;
    // Often the average of the type-reduced interval's endpoints.
    return (crisp_interval.first + crisp_interval.second) / 2.0;
}

// --- Main Calculation Method ---
double GT2FuzzyLogicSystem::calculateOutput(double crisp_error, double crisp_dError, double crisp_wind) {
    std::cout << "\n--- GT2FLS Calculation Start ---" << std::endl;
    // 1. Fuzzify inputs
    auto fuzzified_inputs = fuzzifyInputs(crisp_error, crisp_dError, crisp_wind);

    // 2. Evaluate rules and aggregate outputs
    MembershipInterval aggregated_output_fou = evaluateRulesAndAggregate(fuzzified_inputs);

    // 3. Type Reduction
    std::pair<double, double> type_reduced_interval = typeReduce(aggregated_output_fou);

    // 4. Defuzzification
    double crisp_output = defuzzify(type_reduced_interval);
    
    std::cout << "GT2FLS Crisp Output: " << crisp_output << std::endl;
    std::cout << "--- GT2FLS Calculation End ---\n" << std::endl;
    return crisp_output;
}

} // namespace agent_control_pkg