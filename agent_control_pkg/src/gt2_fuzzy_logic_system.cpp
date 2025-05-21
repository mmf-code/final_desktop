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


// --- Placeholder methods for evaluateRulesAndAggregate, typeReduce, defuzzify (remain the same for now) ---
GT2FuzzyLogicSystem::MembershipInterval
GT2FuzzyLogicSystem::evaluateRulesAndAggregate(const std::map<std::string, std::map<std::string, MembershipInterval>>& fuzzified_inputs) {
    std::cout << "GT2FLS: Evaluate Rules & Aggregate (Placeholder)" << std::endl;
    // For now, just to see if fuzzified_inputs has content from the previous step:
    if (fuzzified_inputs.count("error")) {
        for(const auto& err_set_pair : fuzzified_inputs.at("error")) {
            // std::cout << "  Aggregating based on Error set '" << err_set_pair.first << "' with interval [" 
            //           << err_set_pair.second.first << ", " << err_set_pair.second.second << "]" << std::endl;
        }
    }
    return {0.2, 0.8}; // Dummy aggregated output
}

std::pair<double, double>
GT2FuzzyLogicSystem::typeReduce(const MembershipInterval& aggregated_output_interval) {
    std::cout << "GT2FLS: Type Reduce (Placeholder)" << std::endl;
    double center = (aggregated_output_interval.first + aggregated_output_interval.second) / 2.0;
    return {center - 0.05, center + 0.05}; 
}

double GT2FuzzyLogicSystem::defuzzify(const std::pair<double, double>& crisp_interval) {
    std::cout << "GT2FLS: Defuzzify (Placeholder)" << std::endl;
    return (crisp_interval.first + crisp_interval.second) / 2.0;
}

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