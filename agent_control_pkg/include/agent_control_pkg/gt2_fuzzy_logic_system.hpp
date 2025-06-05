#ifndef AGENT_CONTROL_PKG__GT2_FUZZY_LOGIC_SYSTEM_HPP_
#define AGENT_CONTROL_PKG__GT2_FUZZY_LOGIC_SYSTEM_HPP_

#include <vector>
#include <string>
#include <map>
#include <utility>
#include <algorithm>
#include <numeric>

// #define DEBUG_FLS // uncomment for debug prints

#ifdef DEBUG_FLS
#include <iostream>
#endif

namespace agent_control_pkg
{

class GT2FuzzyLogicSystem
{
public:
    using MembershipInterval = std::pair<double, double>;

    struct IT2TriangularFS_FOU {
        double lmf_left_base, lmf_peak, lmf_right_base;
        double umf_left_base, umf_peak, umf_right_base;

        // Peak assumed as centroid (can change if triangle is not symmetric)
        double getLMFCentroid() const { return lmf_peak; }
    };

    struct FuzzyRule {
        std::map<std::string, std::string> antecedents;
        std::pair<std::string, std::string> consequent;
    };

    struct RuleFiringInfo {
        MembershipInterval firing_strength;
        double consequent_centroid;
        size_t rule_index; // optional for debugging
    };

    GT2FuzzyLogicSystem();
    ~GT2FuzzyLogicSystem();

    // Setups
    void addInputVariable(const std::string& var_name);
    void addOutputVariable(const std::string& var_name);
    void addFuzzySetToVariable(const std::string& var_name, const std::string& set_name, const IT2TriangularFS_FOU& fou);
    void addRule(const FuzzyRule& rule);

    const std::map<std::string, std::map<std::string, IT2TriangularFS_FOU>>& getFuzzySets() const {
        return fuzzy_sets_;
    }

    MembershipInterval getTriangularMembership(double crisp_input, 
                                               double l_left, double l_peak, double l_right,
                                               double u_left, double u_peak, double u_right);

    size_t getRuleCount() const { return rules_.size(); }

    double calculateOutput(double crisp_error, double crisp_dError, double crisp_wind);

private:
    std::map<std::string, std::map<std::string, IT2TriangularFS_FOU>> fuzzy_sets_;
    std::vector<FuzzyRule> rules_;
    std::string output_variable_name_;

    // Pipeline
    std::map<std::string, std::map<std::string, MembershipInterval>> fuzzifyInputs(
        double crisp_error, double crisp_dError, double crisp_wind);

    std::vector<RuleFiringInfo> calculateRuleFirings(
        const std::map<std::string, std::map<std::string, MembershipInterval>>& fuzzified_inputs);
    
    std::pair<double, double> typeReduce_KarnikMendel(std::vector<RuleFiringInfo>& rule_firings);

    double defuzzify(const std::pair<double, double>& type_reduced_interval);
};

} // namespace agent_control_pkg

#endif // AGENT_CONTROL_PKG__GT2_FUZZY_LOGIC_SYSTEM_HPP_
