#include "core_pid_controller/pid_controller.hpp"

#include <algorithm>
#include <cmath>

namespace core_pid_controller
{

PIDController::PIDController(const std::string & name)
: name_(name),
  calculate_error_func_(&calculate_error_minus),
  active_(false),
  P_(0.0),
  I_(0.0),
  D_(0.0),
  FF_(0.0),
  use_negative_gains_(false),
  neg_P_(0.0),
  neg_I_(0.0),
  neg_D_(0.0),
  neg_FF_(0.0),
  integral_(0.0),
  derivative_(0.0),
  derivative_filtered_(0.0),
  error_prev_(0.0),
  integral_threshold_(4.0),
  minimum_(std::numeric_limits<double>::lowest()),
  maximum_(std::numeric_limits<double>::max()),
  constant_(0.0),
  target_(0.0),
  time_prev_()
{
}

void PIDController::set_P(double p) { P_ = p; }
void PIDController::set_I(double i) { I_ = i; }
void PIDController::set_D(double d) { D_ = d; }
void PIDController::set_FF(double ff) { FF_ = ff; }

void PIDController::set_use_negative_gains(bool use_negative)
{
  use_negative_gains_ = use_negative;
  if (!use_negative) {
    neg_P_ = P_;
    neg_I_ = I_;
    neg_D_ = D_;
    neg_FF_ = FF_;
  }
}

void PIDController::set_negative_gains(double neg_p, double neg_i, double neg_d, double neg_ff)
{
  neg_P_ = neg_p;
  neg_I_ = neg_i;
  neg_D_ = neg_d;
  neg_FF_ = neg_ff;
}

void PIDController::set_integral_threshold(double integral_threshold)
{
  integral_threshold_ = integral_threshold;
}

void PIDController::set_minimum(double minimum_value)
{
  minimum_ = minimum_value;
}

void PIDController::set_maximum(double maximum_value)
{
  maximum_ = maximum_value;
}

void PIDController::set_constant(double constant_value)
{
  constant_ = constant_value;
}

void PIDController::set_calculate_error_func(double (*func)(double, double))
{
  calculate_error_func_ = func;
}

void PIDController::set_target(double target_value)
{
  target_ = target_value;
}

double PIDController::get_control(double actual, double ff_quantity)
{
  (void)ff_quantity;  // currently unused, kept for API compatibility

  const auto time_now = std::chrono::steady_clock::now();

  // First call: just initialize the timestamp and return 0.
  if (time_prev_.time_since_epoch().count() == 0) {
    time_prev_ = time_now;
    return 0.0;
  }

  const double dt =
    std::chrono::duration_cast<std::chrono::duration<double>>(time_now - time_prev_).count();

  if (dt <= 0.0) {
    return last_control_;
  }

  const double error = (calculate_error_func_)(target_, actual);
  integral_ += error * dt;
  if (integral_ > integral_threshold_) {
    integral_ = integral_threshold_;
  } else if (integral_ < -integral_threshold_) {
    integral_ = -integral_threshold_;
  }

  if (active_) {
    derivative_ = (error - error_prev_) / dt;
  } else {
    derivative_ = 0.0;
    active_ = true;
  }

  // P/I and derivative-filter mixing FF may switch to neg_* when error < 0 (ROS1 parity).
  double p_gain = P_;
  double i_gain = I_;
  double ff_gain = FF_;
  if (use_negative_gains_ && error < 0.0) {
    p_gain = neg_P_;
    i_gain = neg_I_;
    ff_gain = neg_FF_;
  }

  // Derivative low-pass: (1 - FF) acts on derivative step (ROS1 names this param "FF").
  double derivative_diff = derivative_ - derivative_filtered_;
  const double half_max = 0.5 * maximum_;
  const double half_min = 0.5 * minimum_;
  if (derivative_diff > half_max) {
    derivative_diff = half_max;
  } else if (derivative_diff < half_min) {
    derivative_diff = half_min;
  }
  derivative_filtered_ = derivative_filtered_ + (1.0 - ff_gain) * derivative_diff;

  const double p_component = p_gain * error;
  const double i_component = i_gain * integral_;
  // ROS1: unfiltered D*derivative is not added to control; only D*derivative_filtered is (as "ff_component").
  const double ff_component = D_ * derivative_filtered_;

  double control = p_component + i_component + ff_component + constant_;

  control = std::max(std::min(control, maximum_), minimum_);

  time_prev_ = time_now;
  error_prev_ = error;
  last_control_ = control;

  return control;
}

void PIDController::reset_integral()
{
  integral_ = 0.0;
}

double calculate_error_minus(double target, double actual)
{
  return target - actual;
}

double calculate_error_angle(double target, double actual)
{
  return std::atan2(
    std::sin(target - actual),
    std::cos(target - actual));
}

}  // namespace core_pid_controller

