#include "core_drone_interface/drone_interface.hpp"
#include <rclcpp/logging.hpp>

namespace core_drone_interface
{

void DroneInterface::initialize(rclcpp::Node * node)
{
  node_ = node;
}

void DroneInterface::command_attitude_thrust(const mav_msgs::msg::AttitudeThrust &)
{
  if (node_) {
    RCLCPP_WARN(node_->get_logger(), "command_attitude_thrust not implemented by this plugin.");
  }
}

void DroneInterface::command_rate_thrust(const mav_msgs::msg::RateThrust &)
{
  if (node_) {
    RCLCPP_WARN(node_->get_logger(), "command_rate_thrust not implemented by this plugin.");
  }
}

void DroneInterface::command_roll_pitch_yawrate_thrust(const mav_msgs::msg::RollPitchYawrateThrust &)
{
  if (node_) {
    RCLCPP_WARN(node_->get_logger(), "command_roll_pitch_yawrate_thrust not implemented by this plugin.");
  }
}

void DroneInterface::command_torque_thrust(const mav_msgs::msg::TorqueThrust &)
{
  if (node_) {
    RCLCPP_WARN(node_->get_logger(), "command_torque_thrust not implemented by this plugin.");
  }
}

void DroneInterface::command_pose(const geometry_msgs::msg::PoseStamped &)
{
  if (node_) {
    RCLCPP_WARN(node_->get_logger(), "command_pose not implemented by this plugin.");
  }
}

}  // namespace core_drone_interface
