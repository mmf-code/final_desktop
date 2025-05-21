#ifndef AGENT_CONTROL_PKG__GT2_FUZZY_LOGIC_SYSTEM_HPP_
#define AGENT_CONTROL_PKG__GT2_FUZZY_LOGIC_SYSTEM_HPP_

#include <vector>
#include <string>
#include <map>    // For storing named fuzzy sets or rules
#include <utility> // For std::pair

namespace agent_control_pkg
{

class GT2FuzzyLogicSystem
{
public:
    // For an Interval Type-2 Fuzzy Set, a membership value is an interval [lower, upper]
    using MembershipInterval = std::pair<double, double>;

    // A simple representation of an Interval Type-2 Triangular Fuzzy Set's Footprint of Uncertainty (FOU)
    // LMF: Lower Membership Function (triangle: left_base, peak, right_base)
    // UMF: Upper Membership Function (triangle: left_base, peak, right_base)
    struct IT2TriangularFS_FOU {
        double lmf_left_base, lmf_peak, lmf_right_base;
        double umf_left_base, umf_peak, umf_right_base;
    };

    // A simplified rule structure
    // For now, let's assume specific input names for simplicity: "error", "dError", "wind"
    // And one output: "correction"
    // Antecedents: map<input_name, fuzzy_set_name>
    // Consequent: map<output_name, fuzzy_set_name>
    struct FuzzyRule {
        std::map<std::string, std::string> antecedents; // e.g., {"error": "PositiveSmall", "wind": "StrongPositive"}
        std::pair<std::string, std::string> consequent;   // e.g., {"correction": "NegativeMedium"}
    };

public:
    GT2FuzzyLogicSystem();
    ~GT2FuzzyLogicSystem();

    // --- Configuration Methods (placeholders for now) ---
    // In a real system, these would load from a file (e.g., fuzzy_params.yaml)
    // or be added programmatically.
    void addInputVariable(const std::string& var_name);
    void addOutputVariable(const std::string& var_name);
    void addFuzzySetToVariable(const std::string& var_name, const std::string& set_name, const IT2TriangularFS_FOU& fou);
    void addRule(const FuzzyRule& rule);

    // --- Core FLS Calculation ---
    // Takes crisp inputs, returns a crisp output
    // For now, let's make it specific to our expected inputs for simplicity in this stage
    double calculateOutput(double crisp_error, double crisp_dError, double crisp_wind);

private:
    // --- Internal FLS Components (placeholders/simplified) ---
    std::map<std::string, std::map<std::string, IT2TriangularFS_FOU>> fuzzy_sets_; // var_name -> set_name -> FOU
    std::vector<FuzzyRule> rules_;

    // Helper for triangular MF. Returns [lower_mu, upper_mu]
    MembershipInterval getTriangularMembership(double crisp_input, double l_left, double l_peak, double l_right,
                                                                     double u_left, double u_peak, double u_right);
    
    // Placeholder for Fuzzification
    std::map<std::string, std::map<std::string, MembershipInterval>> fuzzifyInputs(double error, double dError, double wind);

    // Placeholder for Rule Evaluation & Aggregation
    // This would return an aggregated IT2FS for the output variable. For simplicity, let's have it return an interval.
    MembershipInterval evaluateRulesAndAggregate(const std::map<std::string, std::map<std::string, MembershipInterval>>& fuzzified_inputs);
    
    // Placeholder for Type Reduction (e.g., Karnik-Mendel) - returns a crisp interval [left, right]
    std::pair<double, double> typeReduce(const MembershipInterval& aggregated_output_interval);

    // Placeholder for Defuzzification (e.g., average of the type-reduced interval)
    double defuzzify(const std::pair<double, double>& crisp_interval);
};

} // namespace agent_control_pkg

#endif // AGENT_CONTROL_PKG__GT2_FUZZY_LOGIC_SYSTEM_HPP_