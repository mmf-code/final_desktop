#include <gtest/gtest.h>
#include "agent_control_pkg/pid_controller.hpp"
#include <cmath>

class PIDControllerTest : public ::testing::Test {
protected:
    // This will be called before each test
    void SetUp() override {
        // Create a PID controller with default test values
        // kp=1.0, ki=0.1, kd=0.01, output_min=-100, output_max=100, setpoint=0
        pid = new agent_control_pkg::PIDController(1.0, 0.1, 0.01, -100.0, 100.0);
    }

    // This will be called after each test
    void TearDown() override {
        delete pid;
    }

    agent_control_pkg::PIDController* pid;
    
    // Helper function to check if two doubles are approximately equal
    bool approxEqual(double a, double b, double tolerance = 1e-10) {
        return std::abs(a - b) < tolerance;
    }
};

// Test constructor and initial values
TEST_F(PIDControllerTest, InitialValues) {
    EXPECT_DOUBLE_EQ(pid->getKp(), 1.0);
    EXPECT_DOUBLE_EQ(pid->getKi(), 0.1);
    EXPECT_DOUBLE_EQ(pid->getKd(), 0.01);
    EXPECT_DOUBLE_EQ(pid->getSetpoint(), 0.0);
}

// Test setters
TEST_F(PIDControllerTest, Setters) {
    pid->setTunings(2.0, 0.2, 0.02);
    EXPECT_DOUBLE_EQ(pid->getKp(), 2.0);
    EXPECT_DOUBLE_EQ(pid->getKi(), 0.2);
    EXPECT_DOUBLE_EQ(pid->getKd(), 0.02);

    pid->setSetpoint(50.0);
    EXPECT_DOUBLE_EQ(pid->getSetpoint(), 50.0);
}

// Test proportional control
TEST_F(PIDControllerTest, ProportionalControl) {
    // Set Ki and Kd to 0 to test only P term
    pid->setTunings(1.0, 0.0, 0.0);
    pid->setSetpoint(10.0);
    
    // With current_value = 5.0, error = 5.0
    // Expected output = Kp * error = 1.0 * 5.0 = 5.0
    double output = pid->calculate(5.0, 0.1);
    EXPECT_DOUBLE_EQ(output, 5.0);

    // Get individual terms to verify
    auto terms = pid->getLastTerms();
    EXPECT_DOUBLE_EQ(terms.p, 5.0);
    EXPECT_DOUBLE_EQ(terms.i, 0.0);
    EXPECT_DOUBLE_EQ(terms.d, 0.0);
}

// Test integral control
TEST_F(PIDControllerTest, IntegralControl) {
    // Set Kp and Kd to 0 to test only I term
    pid->setTunings(0.0, 1.0, 0.0);
    pid->setSetpoint(10.0);
    
    // With constant error = 5.0 and dt = 0.1
    // First iteration: I term = Ki * error * dt = 1.0 * 5.0 * 0.1 = 0.5
    double output = pid->calculate(5.0, 0.1);
    EXPECT_DOUBLE_EQ(output, 0.5);
    
    // Second iteration: I term should accumulate
    // New I term = 0.5 + (1.0 * 5.0 * 0.1) = 1.0
    output = pid->calculate(5.0, 0.1);
    EXPECT_DOUBLE_EQ(output, 1.0);
}

// Test derivative control
TEST_F(PIDControllerTest, DerivativeControl) {
    // Set Kp and Ki to 0 to test only D term
    pid->setTunings(0.0, 0.0, 1.0);
    pid->setSetpoint(10.0);
    
    // First call to establish previous error
    pid->calculate(5.0, 0.1);
    
    // Second call with new value
    // Change in error = 0 (5.0 - 5.0)
    // D term = Kd * (error - prev_error) / dt = 1.0 * (0) / 0.1 = 0
    double output = pid->calculate(5.0, 0.1);
    EXPECT_DOUBLE_EQ(output, 0.0);
    
    // Third call with changed value
    // Change in error = -2.0 (3.0 - 5.0)
    // D term = 1.0 * (-2.0) / 0.1 = -20.0
    output = pid->calculate(7.0, 0.1);
    EXPECT_DOUBLE_EQ(output, -20.0);
}

// Test reset functionality
TEST_F(PIDControllerTest, Reset) {
    pid->setSetpoint(10.0);
    
    // Run a calculation to accumulate some integral term
    pid->calculate(5.0, 0.1);
    
    // Reset the controller
    pid->reset();
    
    // Get terms after reset
    auto terms = pid->getLastTerms();
    EXPECT_DOUBLE_EQ(terms.p, 0.0);
    EXPECT_DOUBLE_EQ(terms.i, 0.0);
    EXPECT_DOUBLE_EQ(terms.d, 0.0);
}

// Test output limits
TEST_F(PIDControllerTest, OutputLimits) {
    // Set aggressive gains to force output limits
    pid->setTunings(10.0, 0.0, 0.0);
    pid->setSetpoint(100.0);
    
    // This should hit the upper limit (100)
    double output = pid->calculate(0.0, 0.1);
    EXPECT_DOUBLE_EQ(output, 100.0);
    
    // This should hit the lower limit (-100)
    pid->setSetpoint(-100.0);
    output = pid->calculate(0.0, 0.1);
    EXPECT_DOUBLE_EQ(output, -100.0);
}

// Test zero dt handling
TEST_F(PIDControllerTest, ZeroDtHandling) {
    pid->setTunings(1.0, 1.0, 1.0);
    pid->setSetpoint(10.0);
    
    // With dt = 0, all terms should be 0 as a safety measure
    double output = pid->calculate(5.0, 0.0);
    auto terms = pid->getLastTerms();
    
    // All terms should be 0 when dt <= 0
    EXPECT_DOUBLE_EQ(terms.p, 0.0);
    EXPECT_DOUBLE_EQ(terms.i, 0.0);
    EXPECT_DOUBLE_EQ(terms.d, 0.0);
    EXPECT_DOUBLE_EQ(output, 0.0);
    
    // Test negative dt as well
    output = pid->calculate(5.0, -0.1);
    terms = pid->getLastTerms();
    
    EXPECT_DOUBLE_EQ(terms.p, 0.0);
    EXPECT_DOUBLE_EQ(terms.i, 0.0);
    EXPECT_DOUBLE_EQ(terms.d, 0.0);
    EXPECT_DOUBLE_EQ(output, 0.0);
}

// Test combined PID response
TEST_F(PIDControllerTest, CombinedPIDResponse) {
    pid->setTunings(1.0, 0.1, 0.01);
    pid->setSetpoint(10.0);
    
    // First calculation
    double output1 = pid->calculate(5.0, 0.1);
    auto terms1 = pid->getLastTerms();
    
    // Expected:
    // P = 5.0 (error = 5.0, Kp = 1.0)
    // I = 0.05 (error = 5.0, Ki = 0.1, dt = 0.1)
    // D = 0 (first calculation)
    EXPECT_DOUBLE_EQ(terms1.p, 5.0);
    EXPECT_DOUBLE_EQ(terms1.i, 0.05);
    EXPECT_DOUBLE_EQ(terms1.d, 0.0);
    EXPECT_DOUBLE_EQ(output1, 5.05);
    
    // Second calculation with changed error
    double output2 = pid->calculate(7.0, 0.1);
    auto terms2 = pid->getLastTerms();
    
    // Expected:
    // P = 3.0 (error = 3.0)
    // I = 0.08 (previous I + new error * Ki * dt)
    // D = -0.2 (change in error = -2.0, Kd = 0.01, dt = 0.1)
    EXPECT_DOUBLE_EQ(terms2.p, 3.0);
    EXPECT_NEAR(terms2.i, 0.08, 1e-10);
    EXPECT_DOUBLE_EQ(terms2.d, -0.2);
    EXPECT_NEAR(output2, 2.88, 1e-10);
} 