#include "agent_control_pkg/pid_controller.hpp"
#include <algorithm> // For std::clamp

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
  previous_error_(0.0),
  first_calculation_(true),
  last_terms_{0.0, 0.0, 0.0, 0.0} // initialize terms
{
  if (output_min_ >= output_max_) {
    // handle invalid limits
  }
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
  }
}

void PIDController::setSetpoint(double setpoint)
{
  setpoint_ = setpoint;
}

double PIDController::getKp() const { return kp_; }
double PIDController::getKi() const { return ki_; }
double PIDController::getKd() const { return kd_; }
double PIDController::getSetpoint() const { return setpoint_; }

PIDController::PIDTerms PIDController::calculate_with_terms(double current_value, double dt)
{
  if (dt <= 0.0) {
    last_terms_ = {0.0, 0.0, 0.0, 0.0};
    return last_terms_;
  }

  double error = setpoint_ - current_value;

  // Proportional
  double p_term = kp_ * error;

  // Integral
  integral_ += error * dt;
  double i_term = ki_ * integral_;

  // Derivative
  double derivative = 0.0;
  if (first_calculation_) {
    derivative = 0.0;
    first_calculation_ = false;
  } else {
    derivative = (error - previous_error_) / dt;
  }
  double d_term = kd_ * derivative;

  previous_error_ = error;

  // Raw output
  double output = p_term + i_term + d_term;

  // Clamp output
  output = std::clamp(output, output_min_, output_max_);

  // Store terms
  last_terms_ = {p_term, i_term, d_term, output};
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
  first_calculation_ = true;
  last_terms_ = {0.0, 0.0, 0.0, 0.0};
}

} // namespace agent_control_pkg
