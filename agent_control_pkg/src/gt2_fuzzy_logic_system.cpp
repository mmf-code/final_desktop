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

    double weighted_sum_output_lower = 0.0;
    double weighted_sum_output_upper = 0.0;
    double sum_of_firing_strengths_lower = 0.0;
    double sum_of_firing_strengths_upper = 0.0;

    bool any_rule_fired = false;

    for (const auto& rule : rules_) {
        // --- 1. Antecedent Evaluation (Calculate Firing Strength Interval [w_low, w_up] for the rule) ---
        MembershipInterval rule_firing_strength = {1.0, 1.0}; // Start with full strength

        for (const auto& antecedent_pair : rule.antecedents) {
            const std::string& input_var_name = antecedent_pair.first;
            const std::string& fuzzy_set_name = antecedent_pair.second;

            if (fuzzified_inputs.count(input_var_name) && 
                fuzzified_inputs.at(input_var_name).count(fuzzy_set_name)) {
                
                const MembershipInterval& term_membership = fuzzified_inputs.at(input_var_name).at(fuzzy_set_name);
                
                // Apply t-norm (min for Interval Type-2)
                rule_firing_strength.first  = std::min(rule_firing_strength.first,  term_membership.first);  // Lower bound
                rule_firing_strength.second = std::min(rule_firing_strength.second, term_membership.second); // Upper bound
            } else {
                // If any antecedent term is not found or has no membership, this rule effectively doesn't fire
                rule_firing_strength = {0.0, 0.0}; 
                // std::cerr << "Warning: Antecedent term " << input_var_name << "." << fuzzy_set_name 
                //           << " not found in fuzzified inputs for a rule." << std::endl;
                break; // Stop processing antecedents for this rule
            }
        }

        // std::cout << "  Rule (Antecedents -> " << rule.consequent.first << "." << rule.consequent.second 
        //           << ") Firing Strength: [" << rule_firing_strength.first << ", " 
        //           << rule_firing_strength.second << "]" << std::endl;

        if (rule_firing_strength.second > 1e-6) { // If the upper bound of firing strength is greater than (almost) zero
            any_rule_fired = true;
            // --- 2. Get Representative Crisp Value for Consequent ---
            // This is a MAJOR SIMPLIFICATION. Ideally, this involves IT2 set operations.
            // For now, map consequent set names to crisp values.
            double consequent_crisp_value = 0.0;
            const std::string& output_var_name = rule.consequent.first; // Should be "correction"
            const std::string& output_set_name = rule.consequent.second;

            // You'll need to define these mappings based on your output fuzzy sets
            // This is a placeholder - you should adjust these values based on your FOU peaks for output sets
            if (output_set_name == "LNC") consequent_crisp_value = -4.0; // Center of a "Large Negative Correction"
            else if (output_set_name == "SNC") consequent_crisp_value = -1.5;
            else if (output_set_name == "NC")  consequent_crisp_value = 0.0;
            else if (output_set_name == "SPC") consequent_crisp_value = 1.5;
            else if (output_set_name == "LPC") consequent_crisp_value = 4.0;
            else {
                // std::cerr << "Warning: Unknown consequent set name '" << output_set_name << "' in rule." << std::endl;
            }
            
            // std::cout << "    Consequent '" << output_set_name << "' maps to crisp value: " << consequent_crisp_value << std::endl;

            // --- 3. Weighted Sum (Simplified Aggregation/Defuzzification Step) ---
            // We use the firing strength interval to contribute to two sums
            weighted_sum_output_lower += rule_firing_strength.first * consequent_crisp_value;
            weighted_sum_output_upper += rule_firing_strength.second * consequent_crisp_value;
            
            sum_of_firing_strengths_lower += rule_firing_strength.first;
            sum_of_firing_strengths_upper += rule_firing_strength.second;
        }
    }

    // --- Calculate Final Output Interval (Simplified) ---
    MembershipInterval final_aggregated_interval = {0.0, 0.0}; // Default if no rules fired

    if (any_rule_fired) {
        if (sum_of_firing_strengths_lower > 1e-6) {
            final_aggregated_interval.first = weighted_sum_output_lower / sum_of_firing_strengths_lower;
        } else {
            final_aggregated_interval.first = 0.0; // Avoid division by zero
        }
        if (sum_of_firing_strengths_upper > 1e-6) {
            final_aggregated_interval.second = weighted_sum_output_upper / sum_of_firing_strengths_upper;
        } else {
             final_aggregated_interval.second = 0.0; // Avoid division by zero
        }
        
        // Ensure lower <= upper
        if (final_aggregated_interval.first > final_aggregated_interval.second) {
            // This can happen if negative consequents are fired with stronger lower bounds than positive ones.
            // For this simplified method, it might be okay or indicate rule/MF tuning is needed.
            // Let's just swap them for now to maintain a valid interval.
            std::swap(final_aggregated_interval.first, final_aggregated_interval.second);
        }

    } else {
        std::cout << "  No rules fired significantly." << std::endl;
    }
    
    std::cout << "GT2FLS: Simplified Aggregated Output Interval: [" << final_aggregated_interval.first 
              << ", " << final_aggregated_interval.second << "]" << std::endl;

    return final_aggregated_interval; // This interval will then go to typeReduce
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