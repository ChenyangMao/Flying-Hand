#ifndef ROS2_CONTROL_STACK_WRENCH_CONTROLLER_HPP_
#define ROS2_CONTROL_STACK_WRENCH_CONTROLLER_HPP_

#include <string>
#include <deque>

#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Quaternion.h>

#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "core_pid_controller/pid_controller.hpp"

namespace tf2_ros
{
class Buffer;
}

namespace wrench_controller
{

class WrenchController
{
public:
  WrenchController(
    const std::string & sensor_frame,
    const std::string & robot_frame,
    const std::string & world_frame,
    const std::string & camera_frame,
    const std::string & thrust_output_frame,
    const std::string & torque_output_frame,
    double hover_thrust,
    bool filter_ft_data,
    int median_buffer_size,
    int mean_buffer_size);

  void update_target(
    const geometry_msgs::msg::WrenchStamped & target,
    const tf2_ros::Buffer & tf_buffer);

  geometry_msgs::msg::WrenchStamped update_state(
    const geometry_msgs::msg::WrenchStamped & ft_data,
    const tf2_ros::Buffer & tf_buffer);

  void update_odom_state(
    const nav_msgs::msg::Odometry & odom,
    const tf2_ros::Buffer & tf_buffer);

  bool calculate_thrust_torque(
    tf2::Vector3 & thrust_des,
    tf2::Vector3 & torque_des,
    const tf2_ros::Buffer & tf_buffer,
    double force_ff_coefficient,
    double force_ff_coefficient_bias,
    const tf2::Vector3 & wrench_controller_ff_force,
    double velx_damping_coefficient);

  bool check_in_contact(double thresh_force_z) const;

  void reset();

  // Configure PID gains for each force axis from external parameters.
  void configure_fx(
    double p,
    double i,
    double d,
    double integral_threshold,
    double minimum,
    double maximum);
  void configure_fy(double p, double d, double minimum, double maximum);
  void configure_fz(
    double p,
    double i,
    double d,
    double integral_threshold,
    double minimum,
    double maximum);

  double meas_force_x() const { return meas_force_sensor_frame_.x(); }
  double target_force_x() const { return target_force_sensor_frame_.x(); }

private:
  // Controllers
  core_pid_controller::PIDController fx_controller_;
  core_pid_controller::PIDController fy_controller_;
  core_pid_controller::PIDController fz_controller_;
  core_pid_controller::PIDController tx_controller_;
  core_pid_controller::PIDController ty_controller_;
  core_pid_controller::PIDController tz_controller_;

  // Frames and configuration
  std::string sensor_frame_;
  std::string robot_frame_;
  std::string world_frame_;
  std::string camera_frame_;
  std::string thrust_output_frame_;
  std::string torque_output_frame_;

  double last_thrust_{0.0};
  double hover_thrust_{0.0};
  bool got_target_wrench_{false};
  bool got_ft_data_{false};
  bool filter_ft_data_{false};
  int median_filter_max_buffer_size_{0};
  int mean_filter_max_buffer_size_{0};

  // Simple buffers for median/mean filtering of force measurements
  std::deque<tf2::Vector3> force_samples_;
  tf2::Vector3 force_median_filtered_{0.0, 0.0, 0.0};
  tf2::Vector3 force_mean_filtered_{0.0, 0.0, 0.0};

  // Latest target wrench and measured force/torque data
  geometry_msgs::msg::WrenchStamped target_wrench_;
  geometry_msgs::msg::WrenchStamped current_wrench_;

  tf2::Vector3 target_force_sensor_frame_;
  tf2::Vector3 target_torque_sensor_frame_;
  tf2::Vector3 meas_force_sensor_frame_;
  tf2::Vector3 meas_torque_sensor_frame_;
  tf2::Vector3 odometry_vel_world_frame_;

  double thrust_vel_damping_{0.0};

  // TODO: port median/mean filtering helpers when needed.
};

}  // namespace wrench_controller

#endif  // ROS2_CONTROL_STACK_WRENCH_CONTROLLER_HPP_

