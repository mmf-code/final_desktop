#ifndef AGENT_CONTROL_PKG__PID_CONTROLLER_HPP_
#define AGENT_CONTROL_PKG__PID_CONTROLLER_HPP_

namespace agent_control_pkg
{

class PIDController
{
public:
    // Holds the breakdown of P, I, D and total output
    struct PIDTerms {
        double p;
        double i;
        double d;
        double total_output;
    };

    // Constructor
    PIDController(double kp, double ki, double kd, double output_min, double output_max, double setpoint = 0.0);

    // Update gains, output limits, or target
    void setTunings(double kp, double ki, double kd);
    void setOutputLimits(double min, double max);
    void setSetpoint(double setpoint);

    // Access current PID settings
    double getKp() const;
    double getKi() const;
    double getKd() const;
    double getSetpoint() const;

    // Basic PID computation (returns control output)
    double calculate(double current_value, double dt);
    
    // Version that also returns the breakdown of P/I/D
    PIDTerms calculate_with_terms(double current_value, double dt);

    // Retrieve last computed PID components
    PIDTerms getLastTerms() const;

    // Reset integrator and internal state
    void reset();
    
    // Enhanced diagnostic information
    double getIntegralValue() const;
    bool isOutputSaturated() const;

private:
    // PID gains
    double kp_;
    double ki_;
    double kd_;

    // Output range
    double output_min_;
    double output_max_;

    // Target value
    double setpoint_;

    // Internal memory
    double previous_error_;
    double integral_;
    bool first_calculation_;

    double previous_measurement_;  // optional, used if needed

    // Last PID terms (for logging/debug/analysis)
    PIDTerms last_terms_;
    
    // Enhanced anti-windup tracking
    bool output_saturated_;
    double last_output_before_clamp_;
};

}  // namespace agent_control_pkg

#endif  // AGENT_CONTROL_PKG__PID_CONTROLLER_HPP_
