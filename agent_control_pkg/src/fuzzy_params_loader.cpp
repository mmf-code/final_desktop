#include "agent_control_pkg/fuzzy_params_loader.hpp"
#include <fstream>
#include <sstream>
#include <cctype>

namespace agent_control_pkg {

static std::string trim(const std::string& s) {
    size_t start = s.find_first_not_of(" \t\r\n");
    size_t end = s.find_last_not_of(" \t\r\n");
    if(start==std::string::npos) return "";
    return s.substr(start, end-start+1);
}

static std::vector<double> parseNumberList(const std::string& in) {
    std::vector<double> values;
    std::stringstream ss(in);
    std::string tok;
    while(std::getline(ss, tok, ',')) {
        tok = trim(tok);
        if(!tok.empty()) values.push_back(std::stod(tok));
    }
    return values;
}

static std::vector<std::string> parseStringList(const std::string& in) {
    std::vector<std::string> values;
    std::stringstream ss(in);
    std::string tok;
    while(std::getline(ss, tok, ',')) {
        values.push_back(trim(tok));
    }
    return values;
}

static std::string findConfigFile(const std::string& filename) {
    std::vector<std::string> candidates = {
        "../config/" + filename,
        "../../config/" + filename,
        "config/" + filename,
    };
    for(const auto& p : candidates) {
        std::ifstream f(p);
        if(f.good()) return p;
    }
    return filename;
}

bool loadFuzzyParams(const std::string& file, FuzzyParams& fp) {
    std::ifstream in(findConfigFile(file));
    if(!in.is_open()) {
        return false;
    }
    std::string line;
    std::string section;
    std::string current_var;
    while(std::getline(in,line)) {
        line = trim(line);
        if(line.empty() || line[0]=='#') continue;
        if(line == "membership_functions:") { section = "mf"; continue; }
        if(line == "rules:") { section = "rules"; continue; }
        if(section == "mf") {
            if(line.back()==':') {
                current_var = trim(line.substr(0,line.size()-1));
                continue;
            }
            auto pos = line.find(':');
            if(pos==std::string::npos) continue;
            std::string setname = trim(line.substr(0,pos));
            std::string rest = line.substr(pos+1);
            auto lb = rest.find('[');
            auto rb = rest.find(']');
            if(lb==std::string::npos || rb==std::string::npos) continue;
            std::string nums = rest.substr(lb+1, rb-lb-1);
            auto values = parseNumberList(nums);
            if(values.size()==6) {
                fp.sets[current_var][setname] = {values[0],values[1],values[2],values[3],values[4],values[5]};
            }
        } else if(section == "rules") {
            if(line[0]=='-') {
                auto lb=line.find('[');
                auto rb=line.find(']');
                if(lb==std::string::npos || rb==std::string::npos) continue;
                auto tokens = parseStringList(line.substr(lb+1, rb-lb-1));
                if(tokens.size()==4) {
                    fp.rules.push_back({tokens[0],tokens[1],tokens[2],tokens[3]});
                }
            }
        }
    }
    return true;
}

} // namespace agent_control_pkg
