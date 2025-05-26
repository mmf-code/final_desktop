#ifndef AGENT_CONTROL_PKG__GT2_FUZZY_LOGIC_SYSTEM_HPP_
#define AGENT_CONTROL_PKG__GT2_FUZZY_LOGIC_SYSTEM_HPP_

#include <vector>
#include <string>
#include <map>
#include <utility> // For std::pair
#include <algorithm> // For std::sort
#include <numeric>   // For std::accumulate

// Define this to enable verbose FLS logging
// #define DEBUG_FLS 

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

        // Helper to get the centroid of the LMF (assuming it's a Type-1 triangle)
        // For a symmetric triangle, peak is centroid. For asymmetric (a,p,c), centroid is (a+p+c)/3.
        // Let's assume lmf_peak is the representative point for simplicity,
        // or that your triangles are defined such that lmf_peak is the centroid.
        double getLMFCentroid() const { return lmf_peak; }
    };

    struct FuzzyRule {
        std::map<std::string, std::string> antecedents;
        std::pair<std::string, std::string> consequent; // {variable_name, set_name}
    };

    // Information needed for Karnik-Mendel
    struct RuleFiringInfo {
        MembershipInterval firing_strength; // [w_bar, w_tilde]
        double consequent_centroid;         // y_i
        // Storing original rule index can be useful for debugging KM
        size_t rule_index; 
    };


    // --- Constructor & Destructor ---
    GT2FuzzyLogicSystem();
    ~GT2FuzzyLogicSystem();

    // --- Configuration Methods ---
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
    std::string output_variable_name_; // Store the name of the output variable

    // --- Private Helper Methods for FLS Stages ---
    std::map<std::string, std::map<std::string, MembershipInterval>> fuzzifyInputs(
        double crisp_error, double crisp_dError, double crisp_wind);

    // Renamed and refactored:
    // This will now collect firing strengths and consequent centroids for rules that fire.
    std::vector<RuleFiringInfo> calculateRuleFirings(
        const std::map<std::string, std::map<std::string, MembershipInterval>>& fuzzified_inputs);
    
    // This will implement Karnik-Mendel
    std::pair<double, double> typeReduce_KarnikMendel(
        std::vector<RuleFiringInfo>& rule_firings); // Pass by value if sorting, or sort copy

    double defuzzify(
        const std::pair<double, double>& type_reduced_interval);
};

} // namespace agent_control_pkg

#endif // AGENT_CONTROL_PKG__GT2_FUZZY_LOGIC_SYSTEM_HPP_