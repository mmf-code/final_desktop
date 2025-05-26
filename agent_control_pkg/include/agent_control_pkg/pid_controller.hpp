#ifndef AGENT_CONTROL_PKG__PID_CONTROLLER_HPP_
#define AGENT_CONTROL_PKG__PID_CONTROLLER_HPP_

namespace agent_control_pkg
{

class PIDController
{
public:
    // Structure to store individual PID terms
    struct PIDTerms {
        double p;
        double i;
        double d;
        double total_output;
    };

    // Constructor
    PIDController(double kp, double ki, double kd, double output_min, double output_max, double setpoint = 0.0);

    // Setters
    void setTunings(double kp, double ki, double kd);
    void setOutputLimits(double min, double max);
    void setSetpoint(double setpoint);

    // Getters
    double getKp() const;
    double getKi() const;
    double getKd() const;
    double getSetpoint() const;

    // Core PID calculation
    double calculate(double current_value, double dt);
    PIDTerms calculate_with_terms(double current_value, double dt); // Extended version

    // Access last calculated terms
    PIDTerms getLastTerms() const;

    // Reset internal states
    void reset();

private:
    // Gains
    double kp_;
    double ki_;
    double kd_;

    // Output limits
    double output_min_;
    double output_max_;

    // Target value
    double setpoint_;

    // Internal state
    double previous_error_;
    double integral_;
    bool first_calculation_; // Should be bool

    double previous_measurement_;

    // Last PID components
    PIDTerms last_terms_;
};

}  // namespace agent_control_pkg

#endif  // AGENT_CONTROL_PKG__PID_CONTROLLER_HPP_
