#ifndef ROS2_CONTROL_STACK_WRENCH_CONTROLLER_NODE_HPP_
#define ROS2_CONTROL_STACK_WRENCH_CONTROLLER_NODE_HPP_

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "mav_msgs/msg/attitude_thrust.hpp"

#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2/LinearMath/Vector3.h"
#include "tf2/LinearMath/Quaternion.h"

#include "wrench_controller/wrench_controller.hpp"
#include "pose_controller/pose_controller.hpp"

#include "base/BaseNode.hpp"

namespace wrench_controller
{

// Simplified tactile parameter placeholder, keeping the same intent as the ROS1 version.
struct TactileParameter
{
  TactileParameter() : enabled(false) {}
  explicit TactileParameter(bool e) : enabled(e) {}
  bool enabled;
};

class WrenchControlNode : public base::BaseNode
{
public:
  explicit WrenchControlNode(const std::string & node_name);
  ~WrenchControlNode() override = default;

  bool initialize() override;
  bool execute() override;

private:
  // Placeholders for ROS2 subscribers/publishers/services converted from ROS1.

  // Subscribers
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr tracking_point_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr tracking_target_sub_;  // 占位类型
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr ft_data_sub_;
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr ft_setpoint_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr switch_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr arm_sub_;
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_controller_ff_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr vs_active_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr vs_velocity_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr vs_circle_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr euler_angles_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr vins_odom_sub_;

  // Publishers (types may be refined later)
  rclcpp::Publisher<mav_msgs::msg::AttitudeThrust>::SharedPtr command_pub_;
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr filtered_ft_data_pub_;
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr virtual_ft_data_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr thrust_debug_pub_;
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr ft_setpoint_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr tracking_point_pub_;

  // Service
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr publish_control_srv_;

  // Core wrench controller logic and TF buffer
  std::unique_ptr<wrench_controller::WrenchController> wrench_controller_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // ROS1-equivalent state variables
  bool should_publish_{true};
  bool mode_switch_{false};
  bool publish_filtered_ft_data_{true};
  bool visual_servo_active_{false};

  tf2::Vector3 wrench_controller_ff_force_{0.0, 0.0, 0.0};

  // Pose controller for motion control
  std::unique_ptr<pose_controller::PoseController> pose_controller_;

  // Callbacks (ROS2 equivalents of the ROS1 versions)
  void ft_data_callback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg);
  void ft_setpoint_callback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg);
  void tracking_point_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void switch_callback(const std_msgs::msg::Bool::SharedPtr msg);

  // Helper that mixes motion and force thrusts under contact constraints.
  bool combine_motion_and_force(
    const tf2::Vector3 & thrust_force,
    const tf2::Vector3 & thrust_motion,
    const tf2::Vector3 & contact_normal,
    const tf2::Matrix3x3 & vel_mat,
    const tf2::Vector3 & force_constraint_vec,
    const std::string & thrust_frame,
    tf2::Vector3 & out_thrust_des);
};

}  // namespace wrench_controller

#endif  // ROS2_CONTROL_STACK_WRENCH_CONTROLLER_NODE_HPP_

