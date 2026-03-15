#ifndef CORE_DRONE_INTERFACE__DRONE_INTERFACE_HPP_
#define CORE_DRONE_INTERFACE__DRONE_INTERFACE_HPP_

#include <rclcpp/node.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <mav_msgs/msg/attitude_thrust.hpp>
#include <mav_msgs/msg/rate_thrust.hpp>
#include <mav_msgs/msg/roll_pitch_yawrate_thrust.hpp>
#include <mav_msgs/msg/torque_thrust.hpp>

namespace core_drone_interface
{

class DroneInterface
{
public:
  virtual ~DroneInterface() = default;

  /// Called by DroneInterfaceNode after loading the plugin.
  virtual void initialize(rclcpp::Node * node);

  virtual bool request_control() = 0;
  virtual bool arm() = 0;
  virtual bool disarm() = 0;
  virtual bool is_armed() = 0;
  virtual bool has_control() = 0;

  virtual void command_attitude_thrust(const mav_msgs::msg::AttitudeThrust & msg);
  virtual void command_rate_thrust(const mav_msgs::msg::RateThrust & msg);
  virtual void command_roll_pitch_yawrate_thrust(const mav_msgs::msg::RollPitchYawrateThrust & msg);
  virtual void command_torque_thrust(const mav_msgs::msg::TorqueThrust & msg);
  virtual void command_velocity(geometry_msgs::msg::TwistStamped msg) = 0;
  virtual void command_pose(const geometry_msgs::msg::PoseStamped & msg);

protected:
  DroneInterface() = default;
  rclcpp::Node * node_{nullptr};
};

}  // namespace core_drone_interface

#endif  // CORE_DRONE_INTERFACE__DRONE_INTERFACE_HPP_
