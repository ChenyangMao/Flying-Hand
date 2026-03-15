#ifndef CORE_DRONE_INTERFACE__DRONE_INTERFACE_NODE_HPP_
#define CORE_DRONE_INTERFACE__DRONE_INTERFACE_NODE_HPP_

#include <memory>
#include <string>
#include <base/BaseNode.hpp>
#include <core_drone_interface/drone_interface.hpp>
#include <core_drone_interface/srv/drone_command.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <mav_msgs/msg/attitude_thrust.hpp>
#include <mav_msgs/msg/rate_thrust.hpp>
#include <mav_msgs/msg/roll_pitch_yawrate_thrust.hpp>
#include <mav_msgs/msg/torque_thrust.hpp>

namespace core_drone_interface
{

class DroneInterfaceNode : public base::BaseNode
{
public:
  explicit DroneInterfaceNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  bool initialize() override;
  bool execute() override;

private:
  void drone_command_callback(
    const std::shared_ptr<rmw_request_id_t>,
    const std::shared_ptr<core_drone_interface::srv::DroneCommand::Request> request,
    std::shared_ptr<core_drone_interface::srv::DroneCommand::Response> response);

  void attitude_thrust_callback(const mav_msgs::msg::AttitudeThrust::SharedPtr msg);
  void rate_thrust_callback(const mav_msgs::msg::RateThrust::SharedPtr msg);
  void roll_pitch_yawrate_thrust_callback(const mav_msgs::msg::RollPitchYawrateThrust::SharedPtr msg);
  void torque_thrust_callback(const mav_msgs::msg::TorqueThrust::SharedPtr msg);
  void velocity_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg);
  void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

  std::string drone_interface_name_;
  std::shared_ptr<DroneInterface> drone_interface_;
  rclcpp::Service<core_drone_interface::srv::DroneCommand>::SharedPtr drone_command_srv_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr is_armed_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr has_control_pub_;
};

}  // namespace core_drone_interface

#endif  // CORE_DRONE_INTERFACE__DRONE_INTERFACE_NODE_HPP_
