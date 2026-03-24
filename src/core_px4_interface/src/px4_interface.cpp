#include "core_px4_interface/px4_interface.hpp"
#include <pluginlib/class_list_macros.hpp>
#include <cmath>
#include <algorithm>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/msg/attitude_target.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/set_mode.hpp>

namespace core_px4_interface
{

void PX4Interface::initialize(rclcpp::Node * node)
{
  core_drone_interface::DroneInterface::initialize(node);
  if (!node_) return;

  attitude_target_pub_ = node_->create_publisher<mavros_msgs::msg::AttitudeTarget>(
    "mavros/setpoint_raw/attitude", 10);
  state_sub_ = node_->create_subscription<mavros_msgs::msg::State>(
    "mavros/state", 10, std::bind(&PX4Interface::state_callback, this, std::placeholders::_1));
  arming_client_ = node_->create_client<mavros_msgs::srv::CommandBool>("mavros/cmd/arming");
  set_mode_client_ = node_->create_client<mavros_msgs::srv::SetMode>("mavros/set_mode");

  setpoint_rate_hz_ = node_->declare_parameter<double>("setpoint_rate", 50.0);
  setpoint_timer_ = node_->create_wall_timer(
    std::chrono::duration<double>(1.0 / setpoint_rate_hz_),
    std::bind(&PX4Interface::setpoint_timer_callback, this));

  RCLCPP_INFO(
    node_->get_logger(),
    "PX4Interface: subscribe (via DroneInterfaceNode) to attitude_thrust_command; "
    "publish mavros/setpoint_raw/attitude at %.1f Hz (timer resends last command).",
    setpoint_rate_hz_);
}

void PX4Interface::state_callback(const mavros_msgs::msg::State::SharedPtr msg)
{
  current_state_ = *msg;
}

bool PX4Interface::request_control()
{
  if (!node_ || !set_mode_client_->service_is_ready()) return false;
  RCLCPP_INFO(node_->get_logger(), "Requesting OFFBOARD mode.");
  auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
  request->custom_mode = "OFFBOARD";
  set_mode_client_->async_send_request(request);
  return true;  // optimistic; check has_control() after a short delay
}

bool PX4Interface::arm()
{
  if (!node_ || !arming_client_->service_is_ready()) return false;
  if (current_state_.armed) return true;
  RCLCPP_INFO(node_->get_logger(), "Arming.");
  auto request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
  request->value = true;
  arming_client_->async_send_request(request);
  return true;  // optimistic; check is_armed() after a short delay
}

bool PX4Interface::disarm()
{
  if (!node_ || !arming_client_->service_is_ready()) return false;
  if (!current_state_.armed) return true;
  RCLCPP_INFO(node_->get_logger(), "Disarming.");
  auto request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
  request->value = false;
  arming_client_->async_send_request(request);
  return true;  // optimistic; check is_armed() after a short delay
}

bool PX4Interface::is_armed()
{
  return current_state_.armed;
}

bool PX4Interface::has_control()
{
  return current_state_.mode == "OFFBOARD";
}

void PX4Interface::command_attitude_thrust(const mav_msgs::msg::AttitudeThrust & msg)
{
  mavros_msgs::msg::AttitudeTarget att;
  att.header.stamp = node_ ? node_->now() : rclcpp::Time(0);
  att.header.frame_id = "world";
  att.type_mask = mavros_msgs::msg::AttitudeTarget::IGNORE_ROLL_RATE |
                  mavros_msgs::msg::AttitudeTarget::IGNORE_PITCH_RATE |
                  mavros_msgs::msg::AttitudeTarget::IGNORE_YAW_RATE;
  att.orientation = msg.attitude;
  att.body_rate.x = 0.0f;
  att.body_rate.y = 0.0f;
  att.body_rate.z = 0.0f;
  // Use thrust magnitude (same as ROS1) instead of thrust.z only
  const double thrust_mag = std::sqrt(
    msg.thrust.x * msg.thrust.x +
    msg.thrust.y * msg.thrust.y +
    msg.thrust.z * msg.thrust.z);
  att.thrust = static_cast<float>(std::clamp(thrust_mag, 0.0, 1.0));

  last_attitude_target_ = att;
  has_last_command_ = true;
  attitude_target_pub_->publish(att);
}

void PX4Interface::command_velocity(geometry_msgs::msg::TwistStamped msg)
{
  (void)msg;
  if (node_) {
    RCLCPP_WARN_ONCE(node_->get_logger(),
      "PX4Interface::command_velocity() is not implemented; use attitude commands instead.");
  }
}

void PX4Interface::setpoint_timer_callback()
{
  if (!has_last_command_ || !attitude_target_pub_) return;
  last_attitude_target_.header.stamp = node_ ? node_->now() : rclcpp::Time(0);
  attitude_target_pub_->publish(last_attitude_target_);
}

}  // namespace core_px4_interface

PLUGINLIB_EXPORT_CLASS(core_px4_interface::PX4Interface, core_drone_interface::DroneInterface)
