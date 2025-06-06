#ifndef AGENT_CONTROL_PKG__FUZZY_PARAMS_LOADER_HPP_
#define AGENT_CONTROL_PKG__FUZZY_PARAMS_LOADER_HPP_

#include <array>
#include <map>
#include <string>
#include <vector>

namespace agent_control_pkg {

struct FuzzySetFOU {
    double l1, l2, l3, u1, u2, u3;
};

struct FuzzyParams {
    std::map<std::string, std::map<std::string, FuzzySetFOU>> sets;
    std::vector<std::array<std::string,4>> rules;
};

bool loadFuzzyParams(const std::string& file, FuzzyParams& fp);

} // namespace agent_control_pkg

#endif // AGENT_CONTROL_PKG__FUZZY_PARAMS_LOADER_HPP_
