#include "agent_control_pkg/gt2_fuzzy_logic_system.hpp"
#include <iostream>  // For placeholder outputs
#include <algorithm> // For std::min, std::max
#include <cmath>     // For std::abs, etc.

namespace agent_control_pkg
{

// Constructor
GT2FuzzyLogicSystem::GT2FuzzyLogicSystem() {
    // Assuming you might have alpha_planes_ and theta_ as private members
    // Initialize them if they are part of your class from gt2_fuzzy_logic_system.hpp
    // alpha_planes_ = 1; // Default or example
    // theta_ = 0.0;    // Default or example
    std::cout << "GT2FuzzyLogicSystem: Placeholder constructor." << std::endl;
}

// Destructor
GT2FuzzyLogicSystem::~GT2FuzzyLogicSystem() {}

// --- Configuration Methods ---
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
    // std::cout << "GT2FLS: Added a rule. Total rules: " << rules_.size() << std::endl; // Already printed in your main
}

// --- Alpha Planes and Shape Parameter Setters (ensure these are declared in .hpp if called from main) ---
// void GT2FuzzyLogicSystem::setAlphaPlaneCount(int count) { 
//     alpha_planes_ = count; 
//     std::cout << "GT2FLS: Alpha planes set to " << count << std::endl;
// }
// void GT2FuzzyLogicSystem::setShapeParameter(double theta_val) { 
//     theta_ = theta_val; 
//     std::cout << "GT2FLS: Shape parameter theta set to " << theta_val << std::endl;
// }


// Helper for getTriangularMembership
GT2FuzzyLogicSystem::MembershipInterval GT2FuzzyLogicSystem::getTriangularMembership(
    double crisp_input, 
    double l_left, double l_peak, double l_right,
    double u_left, double u_peak, double u_right) 
{
    double lower_mu = 0.0;
    double upper_mu = 0.0;

    // Lower Membership Function (Triangle)
    if (l_peak - l_left > 1e-9) { 
        if (crisp_input >= l_left && crisp_input <= l_peak) {
            lower_mu = (crisp_input - l_left) / (l_peak - l_left);
        }
    } else if (std::abs(crisp_input - l_peak) < 1e-9) { 
         lower_mu = 1.0;
    }
    if (l_right - l_peak > 1e-9) { 
         if (crisp_input > l_peak && crisp_input <= l_right) {
            lower_mu = (l_right - crisp_input) / (l_right - l_peak);
        }
    } else if (std::abs(crisp_input - l_peak) < 1e-9 && !(l_peak - l_left > 1e-9) ) { 
        if (lower_mu < 1.0) lower_mu = 1.0; // Ensure peak is 1 if it's a spike
    }

    // Upper Membership Function (Triangle)
     if (u_peak - u_left > 1e-9) {
        if (crisp_input >= u_left && crisp_input <= u_peak) {
            upper_mu = (crisp_input - u_left) / (u_peak - u_left);
        }
    } else if (std::abs(crisp_input - u_peak) < 1e-9) {
         upper_mu = 1.0;
    }
    if (u_right - u_peak > 1e-9) {
        if (crisp_input > u_peak && crisp_input <= u_right) {
            upper_mu = (u_right - crisp_input) / (u_right - u_peak);
        }
    } else if (std::abs(crisp_input - u_peak) < 1e-9 && !(u_peak - u_left > 1e-9) ) {
         if (upper_mu < 1.0) upper_mu = 1.0;
    }
    
    if (lower_mu > upper_mu) {
        // This case should ideally be prevented by correct FOU definition (UMF >= LMF)
        // For robustness, ensure valid interval.
        // std::cerr << "Warning: LMF > UMF in getTriangularMembership. Clamping. Input: " << crisp_input << std::endl;
        lower_mu = upper_mu; 
    }
    // Clamp mu values between 0 and 1
    lower_mu = std::max(0.0, std::min(lower_mu, 1.0));
    upper_mu = std::max(0.0, std::min(upper_mu, 1.0));

    return {lower_mu, upper_mu};
}

// Fuzzify Inputs
std::map<std::string, std::map<std::string, GT2FuzzyLogicSystem::MembershipInterval>>
GT2FuzzyLogicSystem::fuzzifyInputs(double crisp_error, double crisp_dError, double crisp_wind) {
    // ... (Your existing, correct fuzzifyInputs implementation with std::cout statements) ...
    // (The one you provided that iterates through fuzzy_sets_["error"], etc.)
    std::cout << "GT2FLS: Fuzzifying Inputs - Error=" << crisp_error 
          << ", dError=" << crisp_dError << ", Wind=" << crisp_wind << std::endl;
          
    std::map<std::string, MembershipInterval> error_memberships;
    std::map<std::string, MembershipInterval> dError_memberships;
    std::map<std::string, MembershipInterval> wind_memberships;

    if (fuzzy_sets_.count("error")) {
        std::cout << "  Fuzzifying error input: " << crisp_error << std::endl;
        for (const auto& pair : fuzzy_sets_.at("error")) { // Use .at() for safety if sure key exists
            const std::string& set_name = pair.first;
            const IT2TriangularFS_FOU& fou = pair.second;
            error_memberships[set_name] = getTriangularMembership(
                crisp_error,
                fou.lmf_left_base, fou.lmf_peak, fou.lmf_right_base,
                fou.umf_left_base, fou.umf_peak, fou.umf_right_base
            );
            std::cout << "    Error Set '" << set_name << "': [" << error_memberships[set_name].first 
                       << ", " << error_memberships[set_name].second << "]" << std::endl;
        }
    }
    if (fuzzy_sets_.count("dError")) {
        std::cout << "  Fuzzifying dError input: " << crisp_dError << std::endl;
        for (const auto& pair : fuzzy_sets_.at("dError")) {
            const std::string& set_name = pair.first;
            const IT2TriangularFS_FOU& fou = pair.second;
            dError_memberships[set_name] = getTriangularMembership(
                crisp_dError,
                fou.lmf_left_base, fou.lmf_peak, fou.lmf_right_base,
                fou.umf_left_base, fou.umf_peak, fou.umf_right_base
            );
            std::cout << "    dError Set '" << set_name << "': [" << dError_memberships[set_name].first 
                       << ", " << dError_memberships[set_name].second << "]" << std::endl;
        }
    }
    if (fuzzy_sets_.count("wind")) {
        std::cout << "  Fuzzifying wind input: " << crisp_wind << std::endl;
        for (const auto& pair : fuzzy_sets_.at("wind")) {
            const std::string& set_name = pair.first;
            const IT2TriangularFS_FOU& fou = pair.second;
            wind_memberships[set_name] = getTriangularMembership(
                crisp_wind,
                fou.lmf_left_base, fou.lmf_peak, fou.lmf_right_base,
                fou.umf_left_base, fou.umf_peak, fou.umf_right_base
            );
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

// Rule Evaluation and Aggregation (Simplified Weighted Average)
GT2FuzzyLogicSystem::MembershipInterval 
GT2FuzzyLogicSystem::evaluateRulesAndAggregate(
    const std::map<std::string, std::map<std::string, MembershipInterval>>& fuzzified_inputs)
{
    // ... (Your existing, correct evaluateRulesAndAggregate implementation that uses weighted sums) ...
    // (The one that calculates weighted_sum_output_lower/upper and sum_of_firing_strengths_lower/upper)
    std::cout << "GT2FLS: Evaluating " << rules_.size() << " Rules & Aggregating (Weighted Average Method)..." << std::endl;

    if (rules_.empty()) {
        std::cerr << "Warning: No rules defined in FLS!" << std::endl;
        return {0.0, 0.0};
    }

    double weighted_sum_output_lower = 0.0;
    double weighted_sum_output_upper = 0.0;
    double sum_of_firing_strengths_lower = 0.0;
    double sum_of_firing_strengths_upper = 0.0;
    bool any_rule_fired_meaningfully = false;

    for (size_t rule_idx = 0; rule_idx < rules_.size(); ++rule_idx) {
        const FuzzyRule& rule = rules_[rule_idx];
        
        MembershipInterval rule_firing_strength = {1.0, 1.0}; 
        bool can_evaluate_this_rule = true;

        for (const auto& antecedent_pair : rule.antecedents) {
            const std::string& input_var_name = antecedent_pair.first;
            const std::string& fuzzy_set_name = antecedent_pair.second;

            auto it_var = fuzzified_inputs.find(input_var_name);
            if (it_var != fuzzified_inputs.end()) {
                const auto& var_fuzz_sets = it_var->second;
                auto it_set = var_fuzz_sets.find(fuzzy_set_name);
                if (it_set != var_fuzz_sets.end()) {
                    const MembershipInterval& term_membership = it_set->second;
                    rule_firing_strength.first  = std::min(rule_firing_strength.first,  term_membership.first);
                    rule_firing_strength.second = std::min(rule_firing_strength.second, term_membership.second);
                } else {
                    rule_firing_strength = {0.0, 0.0}; 
                    can_evaluate_this_rule = false; 
                    break; 
                }
            } else {
                rule_firing_strength = {0.0, 0.0}; 
                can_evaluate_this_rule = false; 
                break;
            }
        }
        
        if (can_evaluate_this_rule && rule_firing_strength.second > 1e-9) { 
            any_rule_fired_meaningfully = true;
            double consequent_crisp_value = 0.0;
            const std::string& output_set_name = rule.consequent.second;

            if (output_set_name == "XLNC") consequent_crisp_value = -6.0; // Example mapping
            else if (output_set_name == "LNC") consequent_crisp_value = -4.0; 
            else if (output_set_name == "SNC") consequent_crisp_value = -1.5;
            else if (output_set_name == "NC")  consequent_crisp_value = 0.0;
            else if (output_set_name == "SPC") consequent_crisp_value = 1.5;
            else if (output_set_name == "LPC") consequent_crisp_value = 4.0;
            else if (output_set_name == "XLPC") consequent_crisp_value = 6.0; // Example mapping
            else { /* warning */ }
            
            weighted_sum_output_lower += rule_firing_strength.first * consequent_crisp_value;
            weighted_sum_output_upper += rule_firing_strength.second * consequent_crisp_value;
            sum_of_firing_strengths_lower += rule_firing_strength.first;
            sum_of_firing_strengths_upper += rule_firing_strength.second;
        }
    }

    MembershipInterval final_aggregated_interval = {0.0, 0.0}; 
    if (any_rule_fired_meaningfully) {
        if (sum_of_firing_strengths_lower > 1e-9) {
            final_aggregated_interval.first = weighted_sum_output_lower / sum_of_firing_strengths_lower;
        } else if (sum_of_firing_strengths_upper > 1e-9) { 
            final_aggregated_interval.first = weighted_sum_output_upper / sum_of_firing_strengths_upper;
        }
        if (sum_of_firing_strengths_upper > 1e-9) {
            final_aggregated_interval.second = weighted_sum_output_upper / sum_of_firing_strengths_upper;
        } else if (sum_of_firing_strengths_lower > 1e-9) {
            final_aggregated_interval.second = weighted_sum_output_lower / sum_of_firing_strengths_lower;
        }
        if (final_aggregated_interval.first > final_aggregated_interval.second) {
            std::swap(final_aggregated_interval.first, final_aggregated_interval.second);
        }
    } else {
        std::cout << "  No rules fired significantly enough for weighted average." << std::endl;
    }
    
    std::cout << "GT2FLS: Weighted Average Aggregated Output Interval: [" << final_aggregated_interval.first 
              << ", " << final_aggregated_interval.second << "]" << std::endl;
    return final_aggregated_interval;
}

// --- ADDED/MODIFIED DEFINITIONS FOR typeReduce and defuzzify ---
std::pair<double, double> // Return type matches declaration
GT2FuzzyLogicSystem::typeReduce(const MembershipInterval& aggregated_output_interval) { // Parameter type matches
    std::cout << "GT2FLS: Type Reduce (Placeholder - Passing through aggregated interval)" << std::endl;
    // For this simplified FLS, the "aggregated_output_interval" from our simplified evaluateRulesAndAggregate
    // already represents a crisp interval [left_centroid_approx, right_centroid_approx].
    // So, typeReduce just passes it through.
    // In a full IT2FLS where evaluateRulesAndAggregate returns an IT2FS,
    // this step would involve Karnik-Mendel or similar algorithms.
    return aggregated_output_interval; 
}

double 
GT2FuzzyLogicSystem::defuzzify(const std::pair<double, double>& type_reduced_interval) { // Parameter type matches
    std::cout << "GT2FLS: Defuzzify (Placeholder - Averaging interval)" << std::endl;
    // Defuzzify by taking the average of the interval from our typeReduce placeholder
    return (type_reduced_interval.first + type_reduced_interval.second) / 2.0;
}
// --- END ADDED/MODIFIED DEFINITIONS ---


// Main Calculation Method
double GT2FuzzyLogicSystem::calculateOutput(double crisp_error, double crisp_dError, double crisp_wind) {
    std::cout << "\n--- GT2FLS Calculation Start ---" << std::endl;
    // 1. Fuzzify inputs
    auto fuzzified_inputs = fuzzifyInputs(crisp_error, crisp_dError, crisp_wind);

    // 2. Evaluate rules and aggregate outputs (Simplified weighted average interval)
    MembershipInterval aggregated_output_interval = evaluateRulesAndAggregate(fuzzified_inputs);

    // 3. Type Reduction (Placeholder - passes interval through)
    std::pair<double, double> type_reduced_interval = typeReduce(aggregated_output_interval);

    // 4. Defuzzification (Placeholder - averages the interval)
    double crisp_output = defuzzify(type_reduced_interval);
    
    std::cout << "GT2FLS Crisp Output: " << crisp_output << std::endl;
    std::cout << "--- GT2FLS Calculation End ---\n" << std::endl;
    return crisp_output;
}

// Add if you added these to your header for fuzzy_test_main.cpp:
// void GT2FuzzyLogicSystem::setAlphaPlaneCount(int count) { 
//     alpha_planes_ = count; 
// }
// void GT2FuzzyLogicSystem::setShapeParameter(double theta_val) { 
//     theta_ = theta_val; 
// }


} // namespace agent_control_pkg