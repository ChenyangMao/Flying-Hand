#ifndef CORE_PX4_INTERFACE__PX4_INTERFACE_HPP_
#define CORE_PX4_INTERFACE__PX4_INTERFACE_HPP_

#include <core_drone_interface/drone_interface.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/msg/attitude_target.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <memory>

namespace core_px4_interface
{

class PX4Interface : public core_drone_interface::DroneInterface
{
public:
  void initialize(rclcpp::Node * node) override;

  bool request_control() override;
  bool arm() override;
  bool disarm() override;
  bool is_armed() override;
  bool has_control() override;

  void command_attitude_thrust(const mav_msgs::msg::AttitudeThrust & msg) override;
  void command_velocity(geometry_msgs::msg::TwistStamped msg) override;

private:
  void state_callback(const mavros_msgs::msg::State::SharedPtr msg);
  void setpoint_timer_callback();

  mavros_msgs::msg::State current_state_;
  mavros_msgs::msg::AttitudeTarget last_attitude_target_;
  bool has_last_command_{false};

  rclcpp::Publisher<mavros_msgs::msg::AttitudeTarget>::SharedPtr attitude_target_pub_;
  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
  rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_client_;
  rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client_;
  rclcpp::TimerBase::SharedPtr setpoint_timer_;
  double setpoint_rate_hz_{50.0};
  bool log_mavros_setpoints_{false};
};

}  // namespace core_px4_interface

#endif  // CORE_PX4_INTERFACE__PX4_INTERFACE_HPP_
