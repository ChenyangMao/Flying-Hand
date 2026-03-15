#ifndef ROS2_CONTROL_STACK_CORE_PID_CONTROLLER_HPP_
#define ROS2_CONTROL_STACK_CORE_PID_CONTROLLER_HPP_

#include <chrono>
#include <cstdint>
#include <limits>
#include <string>

namespace core_pid_controller
{

class PIDController
{
public:
  explicit PIDController(const std::string & name = "");

  // Gain setters
  void set_P(double p);
  void set_I(double i);
  void set_D(double d);
  void set_FF(double ff);

  // Limits and constants
  void set_integral_threshold(double integral_threshold);
  void set_minimum(double minimum_value);
  void set_maximum(double maximum_value);
  void set_constant(double constant_value);

  // Error calculation function
  void set_calculate_error_func(double (*func)(double, double));

  // Target and control
  void set_target(double target_value);
  double get_control(double actual, double ff_quantity = 0.0);

  // State management
  void reset_integral();

private:
  std::string name_;

  double (*calculate_error_func_)(double, double);

  bool active_;

  double P_;
  double I_;
  double D_;
  double FF_;

  bool use_negative_gains_;
  double neg_P_;
  double neg_I_;
  double neg_D_;
  double neg_FF_;

  double integral_;
  double derivative_;
  double derivative_filtered_;
  double error_prev_;

  double integral_threshold_;
  double minimum_;
  double maximum_;
  double constant_;

  double target_;

  std::chrono::steady_clock::time_point time_prev_;
};

// Helper error functions (same signatures as ROS1 version)
double calculate_error_minus(double target, double actual);
double calculate_error_angle(double target, double actual);

}  // namespace core_pid_controller

#endif  // ROS2_CONTROL_STACK_CORE_PID_CONTROLLER_HPP_

