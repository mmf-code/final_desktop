// agent_control_pkg/src/pid_controller.cpp
#include "agent_control_pkg/pid_controller.hpp"
#include <algorithm> // For std::clamp
#include <iostream>  // For potential debugging

namespace agent_control_pkg
{

PIDController::PIDController(double kp, double ki, double kd, double output_min, double output_max, double setpoint)
: kp_(kp),
  ki_(ki),
  kd_(kd),
  output_min_(output_min),
  output_max_(output_max),
  setpoint_(setpoint),
  integral_(0.0),
  previous_error_(0.0), // Still useful if other parts of your system might want raw error
  first_calculation_(true),
  last_terms_{0.0, 0.0, 0.0, 0.0},
  previous_measurement_(0.0), // Initialize previous_measurement_
  output_saturated_(false),
  last_output_before_clamp_(0.0)
{
  if (output_min_ >= output_max_) {
    // Consider throwing an exception or logging an error for invalid limits
    std::cerr << "PIDController Warning: Output min (" << output_min_ 
              << ") is not less than output max (" << output_max_ << ")." << std::endl;
    // Default to some safe limits or ensure this state is handled.
  }
  // It might be better to initialize previous_measurement_ with the first 'current_value'
  // but for simplicity, 0.0 is often okay if the system starts near there or first_calculation_ handles it.
  // Or, you can pass an initial current_value to the constructor if known.
}

void PIDController::setTunings(double kp, double ki, double kd)
{
  kp_ = kp;
  ki_ = ki;
  kd_ = kd;
}

void PIDController::setOutputLimits(double min_val, double max_val)
{
  if (min_val < max_val) {
    output_min_ = min_val;
    output_max_ = max_val;
  } else {
     std::cerr << "PIDController Warning: Invalid output limits. Min (" << min_val 
               << ") must be less than Max (" << max_val << ")." << std::endl;
  }
}

void PIDController::setSetpoint(double setpoint)
{
  setpoint_ = setpoint;
  // When setpoint changes, it does not affect the D term directly with derivative on measurement
  // No need to reset integral usually, unless it's a very large step change and you want faster response
  // for some specific strategies (e.g., reset integral on large setpoint changes if it causes windup issues
  // despite clamping, but clamping the PID output usually handles integral windup sufficiently).
}

double PIDController::getKp() const { return kp_; }
double PIDController::getKi() const { return ki_; }
double PIDController::getKd() const { return kd_; }
double PIDController::getSetpoint() const { return setpoint_; }

PIDController::PIDTerms PIDController::calculate_with_terms(double current_value, double dt)
{
  if (dt <= 0.0) {
    // Or return last_terms_ if you prefer no change on invalid dt
    // std::cerr << "PIDController Warning: dt is zero or negative. Returning zero output." << std::endl;
    last_terms_ = {0.0, 0.0, 0.0, 0.0};
    return last_terms_;
  }

  double error = setpoint_ - current_value;

  // Proportional Term
  double p_term = kp_ * error;

  // Integral Term with proper anti-windup protection
  integral_ += error * dt;
  double i_term = ki_ * integral_;
  
  // Calculate preliminary output to check for saturation
  double preliminary_output = p_term + i_term + (-kd_ * (first_calculation_ ? 0.0 : (current_value - previous_measurement_) / dt));
  
  // If output would be saturated and error would increase saturation, don't accumulate integral
  if ((preliminary_output > output_max_ && error > 0.0) || 
      (preliminary_output < output_min_ && error < 0.0)) {
    integral_ -= error * dt;  // Remove the integral accumulation
    i_term = ki_ * integral_;   // Recalculate integral term
  }
  
  // Also clamp integral term directly to prevent excessive buildup
  if (std::abs(ki_) > 1e-9) {
    double max_integral_contribution = std::max(std::abs(output_max_), std::abs(output_min_)) * 0.8;
    double max_integral_value = max_integral_contribution / std::abs(ki_);
    integral_ = std::clamp(integral_, -max_integral_value, max_integral_value);
    i_term = ki_ * integral_;
  }


  // Derivative Term (on Measurement)
  double derivative_input_change = 0.0;
  if (first_calculation_) {
    // On the first call, there's no previous_measurement_ to calculate a derivative.
    // So, derivative is zero. Also, initialize previous_measurement_.
    derivative_input_change = 0.0;
    first_calculation_ = false;
  } else {
    derivative_input_change = current_value - previous_measurement_;
  }
  previous_measurement_ = current_value; // Store current value for next iteration

  // Derivative term is calculated as -Kd * (change_in_measurement / dt)
  // or Kd * (change_in_error_if_setpoint_is_constant / dt)
  // Since dError/dt = d(Setpoint - Measurement)/dt. If Setpoint is const, dError/dt = -dMeasurement/dt.
  // So, Kd * dError/dt = Kd * (-dMeasurement/dt)
  // The `derivative_input_change / dt` is dMeasurement/dt
  double d_term = -kd_ * (derivative_input_change / dt);


  // Store previous_error_ - still useful if your FLS or other logic needs the raw error derivative.
  // If FLS dError is also meant to be based on measurement, that's a separate consideration.
  // For now, this PID's D-term is on measurement.
  previous_error_ = error;


  // Raw PID Output
  double output = p_term + i_term + d_term;
  last_output_before_clamp_ = output;

  // Clamp Output to defined limits with enhanced anti-windup
  double clamped_output = std::clamp(output, output_min_, output_max_);
  output_saturated_ = (output != clamped_output);

  // Enhanced anti-windup: If output was clamped, back-calculate integral to prevent further windup
  if (output_saturated_ && std::abs(ki_) > 1e-9) {
    double excess_output = output - clamped_output;
    double integral_correction = excess_output / ki_;
    integral_ -= integral_correction * 0.5;  // Partial correction to prevent oscillation
  }


  // Store terms (using potentially unclamped P, I, D for analysis, but clamped total_output)
  last_terms_ = {p_term, i_term, d_term, clamped_output};
  return last_terms_;
}

double PIDController::calculate(double current_value, double dt)
{
  return calculate_with_terms(current_value, dt).total_output;
}

PIDController::PIDTerms PIDController::getLastTerms() const
{
  return last_terms_;
}

void PIDController::reset()
{
  integral_ = 0.0;
  previous_error_ = 0.0;
  // previous_measurement_ should ideally be reset to the current system value if known,
  // or rely on first_calculation_ to handle it.
  // Setting to 0.0 might cause a small d-term blip on the first calculation after reset if system isn't at 0.
  // A common approach is to not reset previous_measurement_ here, or set first_calculation_ to true.
  first_calculation_ = true;
  // previous_measurement_ = 0.0; // Or get current value if possible at reset time
  last_terms_ = {0.0, 0.0, 0.0, 0.0};
  output_saturated_ = false;
  last_output_before_clamp_ = 0.0;
}

double PIDController::getIntegralValue() const
{
  return integral_;
}

bool PIDController::isOutputSaturated() const
{
  return output_saturated_;
}

} // namespace agent_control_pkg