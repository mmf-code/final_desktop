#include "agent_control_pkg/gt2_fuzzy_logic_system.hpp"
#include <iostream> // For placeholder outputs
#include <algorithm> // For std::min, std::max
#include <cmath>     // For std::abs, etc.

namespace agent_control_pkg
{

// Constructor (remains the same)
GT2FuzzyLogicSystem::GT2FuzzyLogicSystem() {
    std::cout << "GT2FuzzyLogicSystem: Placeholder constructor." << std::endl;
}

// Destructor (remains the same)
GT2FuzzyLogicSystem::~GT2FuzzyLogicSystem() {}

// --- Configuration Method Placeholders (remain the same) ---
void GT2FuzzyLogicSystem::addInputVariable(const std::string& var_name) {
    if (fuzzy_sets_.find(var_name) == fuzzy_sets_.end()) {
        fuzzy_sets_[var_name] = {};
        std::cout << "GT2FLS: Added input variable: " << var_name << std::endl;
    }
}
void GT2FuzzyLogicSystem::addOutputVariable(const std::string& var_name) {
     if (fuzzy_sets_.find(var_name) == fuzzy_sets_.end()) {
        fuzzy_sets_[var_name] = {};
        std::cout << "GT2FLS: Added output variable: " << var_name << std::endl;
    }
}
void GT2FuzzyLogicSystem::addFuzzySetToVariable(const std::string& var_name, const std::string& set_name, const IT2TriangularFS_FOU& fou) {
    if (fuzzy_sets_.count(var_name)) {
        fuzzy_sets_[var_name][set_name] = fou;
        std::cout << "GT2FLS: Added set '" << set_name << "' to variable '" << var_name << "'" << std::endl;
    } else {
        std::cerr << "GT2FLS: Error - Variable '" << var_name << "' not defined before adding set." << std::endl;
    }
}
void GT2FuzzyLogicSystem::addRule(const FuzzyRule& rule) {
    rules_.push_back(rule);
    std::cout << "GT2FLS: Added a rule. Total rules: " << rules_.size() << std::endl;
}

// Helper for getTriangularMembership (remains the same)
GT2FuzzyLogicSystem::MembershipInterval GT2FuzzyLogicSystem::getTriangularMembership(
    double crisp_input, 
    double l_left, double l_peak, double l_right,
    double u_left, double u_peak, double u_right) 
{
    double lower_mu = 0.0;
    double upper_mu = 0.0;
    // ... (implementation as you have it) ...
    if (crisp_input >= l_left && crisp_input <= l_peak) {
        if (l_peak - l_left > 1e-6) lower_mu = (crisp_input - l_left) / (l_peak - l_left);
        else if (std::abs(crisp_input - l_peak) < 1e-6) lower_mu = 1.0;
    } else if (crisp_input > l_peak && crisp_input <= l_right) {
        if (l_right - l_peak > 1e-6) lower_mu = (l_right - crisp_input) / (l_right - l_peak);
         else if (std::abs(crisp_input - l_peak) < 1e-6) lower_mu = 1.0;
    }
    if (crisp_input >= u_left && crisp_input <= u_peak) {
         if (u_peak - u_left > 1e-6) upper_mu = (crisp_input - u_left) / (u_peak - u_left);
        else if (std::abs(crisp_input - u_peak) < 1e-6) upper_mu = 1.0;
    } else if (crisp_input > u_peak && crisp_input <= u_right) {
        if (u_right - u_peak > 1e-6) upper_mu = (u_right - crisp_input) / (u_right - u_peak);
        else if (std::abs(crisp_input - u_peak) < 1e-6) upper_mu = 1.0;
    }
    if (lower_mu > upper_mu) {
        lower_mu = std::min(lower_mu, upper_mu); 
    }
    return {lower_mu, upper_mu};
}


// *************************************************************************** //
// *** THIS IS THE METHOD YOU NEED TO REPLACE / UPDATE ***
std::map<std::string, std::map<std::string, GT2FuzzyLogicSystem::MembershipInterval>>
GT2FuzzyLogicSystem::fuzzifyInputs(double crisp_error, double crisp_dError, double crisp_wind) {

    std::cout << "GT2FLS: Fuzzifying Inputs - Error=" << crisp_error 
          << ", dError=" << crisp_dError << ", Wind=" << crisp_wind << std::endl;
          
    std::map<std::string, MembershipInterval> error_memberships;
    std::map<std::string, MembershipInterval> dError_memberships;
    std::map<std::string, MembershipInterval> wind_memberships;



    // Fuzzify "error"
    if (fuzzy_sets_.count("error")) {
        std::cout << "  Fuzzifying error input: " << crisp_error << std::endl; // Optional debug
        for (const auto& pair : fuzzy_sets_["error"]) {
            const std::string& set_name = pair.first;
            const IT2TriangularFS_FOU& fou = pair.second;
            error_memberships[set_name] = getTriangularMembership(
                crisp_error,
                fou.lmf_left_base, fou.lmf_peak, fou.lmf_right_base,
                fou.umf_left_base, fou.umf_peak, fou.umf_right_base
            );
            // Optional: Print individual membership for debugging
            std::cout << "    Error Set '" << set_name << "': [" << error_memberships[set_name].first 
                       << ", " << error_memberships[set_name].second << "]" << std::endl;
        }
    }

    // Fuzzify "dError"
    if (fuzzy_sets_.count("dError")) {
        std::cout << "  Fuzzifying dError input: " << crisp_dError << std::endl; // Optional debug
        for (const auto& pair : fuzzy_sets_["dError"]) {
            const std::string& set_name = pair.first;
            const IT2TriangularFS_FOU& fou = pair.second;
            dError_memberships[set_name] = getTriangularMembership(
                crisp_dError,
                fou.lmf_left_base, fou.lmf_peak, fou.lmf_right_base,
                fou.umf_left_base, fou.umf_peak, fou.umf_right_base
            );
            // Optional: Print individual membership
            std::cout << "    dError Set '" << set_name << "': [" << dError_memberships[set_name].first 
                       << ", " << dError_memberships[set_name].second << "]" << std::endl;
        }
    }

    // Fuzzify "wind"
    if (fuzzy_sets_.count("wind")) {
        std::cout << "  Fuzzifying wind input: " << crisp_wind << std::endl; // Optional debug
        for (const auto& pair : fuzzy_sets_["wind"]) {
            const std::string& set_name = pair.first;
            const IT2TriangularFS_FOU& fou = pair.second;
            wind_memberships[set_name] = getTriangularMembership(
                crisp_wind,
                fou.lmf_left_base, fou.lmf_peak, fou.lmf_right_base,
                fou.umf_left_base, fou.umf_peak, fou.umf_right_base
            );
            // Optional: Print individual membership
            std::cout << "    Wind Set '" << set_name << "': [" << wind_memberships[set_name].first 
                       << ", " << wind_memberships[set_name].second << "]" << std::endl;
        }
    }

    std::map<std::string, std::map<std::string, MembershipInterval>> all_fuzzified_inputs;
    if (!error_memberships.empty()) all_fuzzified_inputs["error"] = error_memberships;
    if (!dError_memberships.empty()) all_fuzzified_inputs["dError"] = dError_memberships;
    if (!wind_memberships.empty()) all_fuzzified_inputs["wind"] = wind_memberships;
    
    return all_fuzzified_inputs;
}
// *************************************************************************** //


// In gt2_fuzzy_logic_system.cpp

// ... (constructor, addInputVariable, addOutputVariable, addFuzzySetToVariable, addRule, 
//      getTriangularMembership, fuzzifyInputs remain the same) ...

GT2FuzzyLogicSystem::MembershipInterval // We'll still return an interval, but it'll be based on weighted average
GT2FuzzyLogicSystem::evaluateRulesAndAggregate(
    const std::map<std::string, std::map<std::string, MembershipInterval>>& fuzzified_inputs)
{
    std::cout << "GT2FLS: Evaluating " << rules_.size() << " Rules & Aggregating..." << std::endl;

    if (rules_.empty()) {
        std::cerr << "Warning: No rules defined in FLS!" << std::endl;
        return {0.0, 0.0}; // Return a zero interval if no rules
    }

    double overall_max_lower_firing_strength = 0.0;
    double overall_max_upper_firing_strength = 0.0;
    bool any_rule_fired = false;

    for (size_t rule_idx = 0; rule_idx < rules_.size(); ++rule_idx) {
        const FuzzyRule& rule = rules_[rule_idx];
        
        double current_rule_lower_firing_strength = 1.0; // Start with max for min operation
        double current_rule_upper_firing_strength = 1.0; // Start with max for min operation
        bool can_evaluate_rule = true;

        // 1. Antecedent Evaluation (t-norm: min)
        // std::cout << "  Evaluating Rule " << rule_idx + 1 << ":" << std::endl;
        for (const auto& antecedent_pair : rule.antecedents) {
            // antecedent_pair is like {"error", "ZE"}
            const std::string& input_variable_name = antecedent_pair.first;
            const std::string& fuzzy_set_name = antecedent_pair.second;

            // Check if fuzzified input for this variable exists
            if (fuzzified_inputs.count(input_variable_name)) {
                const auto& var_fuzz_sets = fuzzified_inputs.at(input_variable_name);
                // Check if the specific fuzzy set for this antecedent exists
                if (var_fuzz_sets.count(fuzzy_set_name)) {
                    const MembershipInterval& mi = var_fuzz_sets.at(fuzzy_set_name);
                    current_rule_lower_firing_strength = std::min(current_rule_lower_firing_strength, mi.first);
                    current_rule_upper_firing_strength = std::min(current_rule_upper_firing_strength, mi.second);
                    // std::cout << "    Antecedent: " << input_variable_name << " is " << fuzzy_set_name
                    //           << " -> mu_interval = [" << mi.first << ", " << mi.second << "]" << std::endl;
                } else {
                    std::cerr << "Warning: Rule " << rule_idx + 1 << ": Fuzzy set '" << fuzzy_set_name 
                              << "' not found for input '" << input_variable_name << "'. Skipping rule part." << std::endl;
                    can_evaluate_rule = false; // Cannot fully evaluate this rule's antecedents
                    break; 
                }
            } else {
                std::cerr << "Warning: Rule " << rule_idx + 1 << ": Input variable '" << input_variable_name 
                          << "' not found in fuzzified inputs. Skipping rule part." << std::endl;
                can_evaluate_rule = false;
                break;
            }
        }

        if (!can_evaluate_rule) {
            current_rule_lower_firing_strength = 0.0; // Rule effectively doesn't fire
            current_rule_upper_firing_strength = 0.0;
        }
        
        // std::cout << "    Rule " << rule_idx + 1 << " Firing Strength Interval: [" 
        //           << current_rule_lower_firing_strength << ", " 
        //           << current_rule_upper_firing_strength << "]" << std::endl;

        // For this simplified aggregation, we take the maximum firing strength found so far
        // This is a very crude form of aggregation. True aggregation combines implied consequents.
        if (current_rule_lower_firing_strength > 0 || current_rule_upper_firing_strength > 0) {
            any_rule_fired = true;
        }
        overall_max_lower_firing_strength = std::max(overall_max_lower_firing_strength, current_rule_lower_firing_strength);
        overall_max_upper_firing_strength = std::max(overall_max_upper_firing_strength, current_rule_upper_firing_strength);
    } // End of loop through rules

    if (!any_rule_fired) {
        std::cout << "  No rules fired significantly." << std::endl;
    }

    // This returned interval is what our (currently placeholder) type-reducer will get.
    // It's a very simplified representation of the aggregated output.
    MembershipInterval simplified_aggregated_output_interval = {overall_max_lower_firing_strength, overall_max_upper_firing_strength};
    std::cout << "GT2FLS: Simplified Aggregated Output Interval: [" 
              << simplified_aggregated_output_interval.first << ", " 
              << simplified_aggregated_output_interval.second << "]" << std::endl;
              
    return simplified_aggregated_output_interval;
}
// --- typeReduce and defuzzify methods (remain the same simple placeholders for now) ---
std::pair<double, double>
GT2FuzzyLogicSystem::typeReduce(const MembershipInterval& aggregated_output_interval) {
    std::cout << "GT2FLS: Type Reduce (Using input interval as is - Placeholder)" << std::endl;
    // For this simplified approach, typeReduce might just pass through the interval from evaluateRulesAndAggregate
    return aggregated_output_interval; 
}

double GT2FuzzyLogicSystem::defuzzify(const std::pair<double, double>& type_reduced_interval) {
    std::cout << "GT2FLS: Defuzzify (Averaging interval)" << std::endl;
    // Defuzzify by taking the average of the interval from our simplified aggregation
    return (type_reduced_interval.first + type_reduced_interval.second) / 2.0;
}


// calculateOutput method remains the same, calling these updated/placeholder stages
// ... (calculateOutput method as before) ...

// --- Main Calculation Method (remains the same, calls the above methods) ---
double GT2FuzzyLogicSystem::calculateOutput(double crisp_error, double crisp_dError, double crisp_wind) {
    std::cout << "\n--- GT2FLS Calculation Start ---" << std::endl;
    auto fuzzified_inputs = fuzzifyInputs(crisp_error, crisp_dError, crisp_wind);
    MembershipInterval aggregated_output_fou = evaluateRulesAndAggregate(fuzzified_inputs);
    std::pair<double, double> type_reduced_interval = typeReduce(aggregated_output_fou);
    double crisp_output = defuzzify(type_reduced_interval);
    std::cout << "GT2FLS Crisp Output: " << crisp_output << std::endl;
    std::cout << "--- GT2FLS Calculation End ---\n" << std::endl;
    return crisp_output;
}

} // namespace agent_control_pkg