#include "agent_control_pkg/gt2_fuzzy_logic_system.hpp"
#include <gtest/gtest.h>
#include <cmath>

using agent_control_pkg::GT2FuzzyLogicSystem;
using FOU = GT2FuzzyLogicSystem::IT2TriangularFS_FOU;

// Helper function to construct a very small FLS with three symmetric sets
static GT2FuzzyLogicSystem makeSimpleFLS() {
    GT2FuzzyLogicSystem fls;
    fls.addInputVariable("error");
    fls.addInputVariable("dError");
    fls.addOutputVariable("correction");

    // Triangular sets with identical lower/upper membership (Type-1 like)
    fls.addFuzzySetToVariable("error","N",FOU{-10.0,-5.0,0.0,-10.0,-5.0,0.0});
    fls.addFuzzySetToVariable("error","Z",FOU{-1.0,0.0,1.0,-1.0,0.0,1.0});
    fls.addFuzzySetToVariable("error","P",FOU{0.0,5.0,10.0,0.0,5.0,10.0});

    fls.addFuzzySetToVariable("dError","N",FOU{-10.0,-5.0,0.0,-10.0,-5.0,0.0});
    fls.addFuzzySetToVariable("dError","Z",FOU{-1.0,0.0,1.0,-1.0,0.0,1.0});
    fls.addFuzzySetToVariable("dError","P",FOU{0.0,5.0,10.0,0.0,5.0,10.0});

    fls.addFuzzySetToVariable("correction","N",FOU{-10.0,-5.0,0.0,-10.0,-5.0,0.0});
    fls.addFuzzySetToVariable("correction","Z",FOU{-1.0,0.0,1.0,-1.0,0.0,1.0});
    fls.addFuzzySetToVariable("correction","P",FOU{0.0,5.0,10.0,0.0,5.0,10.0});

    // Three simple rules
    GT2FuzzyLogicSystem::FuzzyRule r1{{{"error","N"},{"dError","N"}}, {"correction","N"}};
    GT2FuzzyLogicSystem::FuzzyRule r2{{{"error","Z"},{"dError","Z"}}, {"correction","Z"}};
    GT2FuzzyLogicSystem::FuzzyRule r3{{{"error","P"},{"dError","P"}}, {"correction","P"}};
    fls.addRule(r1);
    fls.addRule(r2);
    fls.addRule(r3);
    return fls;
}

TEST(FuzzyLogicSystemTest, FuzzificationMembership) {
    GT2FuzzyLogicSystem fls = makeSimpleFLS();
    // Membership of error=-5 in set N should be 1
    auto mi = fls.getTriangularMembership(-5.0,-10.0,-5.0,0.0,-10.0,-5.0,0.0);
    EXPECT_NEAR(mi.first,1.0,1e-6);
    EXPECT_NEAR(mi.second,1.0,1e-6);
    // Membership of error=0 in set Z should be 1
    auto mi2 = fls.getTriangularMembership(0.0,-1.0,0.0,1.0,-1.0,0.0,1.0);
    EXPECT_NEAR(mi2.first,1.0,1e-6);
    EXPECT_NEAR(mi2.second,1.0,1e-6);
}

TEST(FuzzyLogicSystemTest, RuleFiringAndOutput) {
    GT2FuzzyLogicSystem fls = makeSimpleFLS();
    // Case where rule 1 should fire fully
    double out1 = fls.calculateOutput(-5.0,-5.0,0.0);
    EXPECT_NEAR(out1,-5.0,1e-6);

    // Case where rule 2 should fire fully
    double out2 = fls.calculateOutput(0.0,0.0,0.0);
    EXPECT_NEAR(out2,0.0,1e-6);

    // Case where rule 3 should fire fully
    double out3 = fls.calculateOutput(5.0,5.0,0.0);
    EXPECT_NEAR(out3,5.0,1e-6);
}

TEST(FuzzyLogicSystemTest, NoMatchingRuleGivesZero) {
    GT2FuzzyLogicSystem fls = makeSimpleFLS();
    double out = fls.calculateOutput(-5.0,5.0,0.0);
    EXPECT_NEAR(out,0.0,1e-6);
}

