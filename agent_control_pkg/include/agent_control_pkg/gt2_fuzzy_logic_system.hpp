#ifndef AGENT_CONTROL_PKG__GT2_FUZZY_LOGIC_SYSTEM_HPP_
#define AGENT_CONTROL_PKG__GT2_FUZZY_LOGIC_SYSTEM_HPP_

#include <vector>
#include <string>
#include <map>
#include <utility> // For std::pair

namespace agent_control_pkg
{

class GT2FuzzyLogicSystem
{
public:
    using MembershipInterval = std::pair<double, double>;

    struct IT2TriangularFS_FOU {
        double lmf_left_base, lmf_peak, lmf_right_base;
        double umf_left_base, umf_peak, umf_right_base;
    };

    struct FuzzyRule {
        std::map<std::string, std::string> antecedents;
        std::pair<std::string, std::string> consequent;
    };

    // --- Constructor & Destructor ---
    GT2FuzzyLogicSystem();
    ~GT2FuzzyLogicSystem();

    // --- Configuration Methods ---
    void addInputVariable(const std::string& var_name);
    void addOutputVariable(const std::string& var_name);
    void addFuzzySetToVariable(const std::string& var_name, const std::string& set_name, const IT2TriangularFS_FOU& fou);
    void addRule(const FuzzyRule& rule);

    // --- Methods for Testing & Data Generation (made public for convenience) ---
    // Getter for all defined fuzzy sets (needed by fuzzy_test_main.cpp for MF plotting)
    const std::map<std::string, std::map<std::string, IT2TriangularFS_FOU>>& getFuzzySets() const {
        return fuzzy_sets_;
    }

    // Helper for triangular MF. Public for direct use in MF plotting if needed.
    MembershipInterval getTriangularMembership(double crisp_input, 
                                               double l_left, double l_peak, double l_right,
                                               double u_left, double u_peak, double u_right);

    // Public for direct testing of fuzzification stage from fuzzy_test_main.cpp
    std::map<std::string, std::map<std::string, MembershipInterval>> fuzzifyInputs(
        double crisp_error, double crisp_dError, double crisp_wind);

    // Getter for rule count (already public, good)
    size_t getRuleCount() const { return rules_.size(); }

    // --- (Optional) Stubs for advanced parameters if you plan to use them ---
    // void setAlphaPlaneCount(int count) { alpha_planes_ = count; /* Placeholder */ }
    // void setShapeParameter(double theta_val) { theta_ = theta_val; /* Placeholder */ }


    // --- Core FLS Calculation ---
    double calculateOutput(double crisp_error, double crisp_dError, double crisp_wind);

private:
    // --- Internal FLS Components ---
    std::map<std::string, std::map<std::string, IT2TriangularFS_FOU>> fuzzy_sets_;
    std::vector<FuzzyRule> rules_;

    // --- (Optional) Private members for advanced parameters ---
    // int alpha_planes_ = 1; 
    // double theta_ = 0.0;   

    // --- Private Helper Methods for FLS Stages ---
    MembershipInterval evaluateRulesAndAggregate(
        const std::map<std::string, std::map<std::string, MembershipInterval>>& fuzzified_inputs);
    
    std::pair<double, double> typeReduce(
        const MembershipInterval& aggregated_output_interval);

    double defuzzify(
        const std::pair<double, double>& type_reduced_interval);
};

} // namespace agent_control_pkg

#endif // AGENT_CONTROL_PKG__GT2_FUZZY_LOGIC_SYSTEM_HPP_