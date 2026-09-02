#ifndef ROS2_CONTROL_STACK_WRENCH_CONTROLLER_NODE_HPP_
#define ROS2_CONTROL_STACK_WRENCH_CONTROLLER_NODE_HPP_

#include <memory>
#include <string>

#include <Eigen/Dense>

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
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr vs_enable_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr vs_active_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr vs_velocity_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr vs_circle_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr vs_depth_sub_;
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
  bool visual_servo_enabled_{false};
  bool visual_servo_active_{false};

  tf2::Vector3 wrench_controller_ff_force_{0.0, 0.0, 0.0};
  tf2::Vector3 current_body_velocity_{0.0, 0.0, 0.0};
  tf2::Vector3 latest_vs_cmd_camera_{0.0, 0.0, 0.0};
  tf2::Vector3 filtered_vs_body_velocity_{0.0, 0.0, 0.0};
  Eigen::Vector3d vs_integral_error_{0.0, 0.0, 0.0};
  Eigen::Vector3d vs_kp_{0.60, 0.40, 0.45};
  Eigen::Vector3d vs_ki_{0.03, 0.03, 0.03};
  Eigen::Vector3d vs_max_body_velocity_{0.12, 0.06, 0.08};
  Eigen::Vector3d vs_max_thrust_delta_{0.06, 0.04, 0.05};
  double thrust_z_damping_coeff_{0.01};
  double vs_depth_min_{0.55};
  double vs_depth_max_{0.75};
  double vs_timeout_sec_{0.5};
  double vs_cmd_alpha_{0.25};
  double vs_enable_ramp_sec_{1.0};
  double latest_vs_depth_{0.0};
  double last_vs_active_stamp_sec_{0.0};
  double last_vs_cmd_stamp_sec_{0.0};
  double last_vs_depth_stamp_sec_{0.0};
  double last_vs_enable_change_sec_{0.0};
  std::string target_frame_{"map"};
  std::string robot_frame_{"base_link"};
  std::string camera_frame_{"camera"};
  std::string world_frame_{"map"};
  std::string sensor_frame_{"ft_sensor"};
  double force_ff_coefficient_{0.01};
  double force_ff_coefficient_bias_{0.05};
  double velx_damping_coefficient_{0.0};

  // Sensor-dropout watchdog: disable wrench control if odom or FT data
  // goes stale for longer than these thresholds (seconds).
  double odom_dropout_timeout_sec_{0.5};
  double ft_dropout_timeout_sec_{0.5};
  double last_odom_stamp_sec_{0.0};
  double last_ft_stamp_sec_{0.0};

  // Pose controller for motion control
  std::unique_ptr<pose_controller::PoseController> pose_controller_;

  // Contact frame name (from parameter, used by combine_motion_and_force)
  std::string contact_frame_{"contact"};

  // Diagonal of vel_mat in combine_motion_and_force: force_mat = I - diag(mix_vel_*).
  // For axis a, wrench thrust contributes (1 - mix_vel_a) * F_a (e.g. 0.3 when mix_vel_x=0.7).
  double mix_vel_x_{0.7};
  double mix_vel_y_{1.0};
  double mix_vel_z_{1.0};

  // Callbacks (ROS2 equivalents of the ROS1 versions)
  void ft_data_callback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg);
  void ft_setpoint_callback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg);
  void tracking_point_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void switch_callback(const std_msgs::msg::Bool::SharedPtr msg);
  void vs_enable_callback(const std_msgs::msg::Bool::SharedPtr msg);
  void vs_active_callback(const std_msgs::msg::Bool::SharedPtr msg);
  void vs_velocity_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg);
  void vs_depth_callback(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg);
  void reset_visual_servo_pi();
  bool get_visual_servo_body_velocity(tf2::Vector3 & desired_body_velocity);
  bool rotate_vector_between_frames(
    const tf2::Vector3 & input,
    const std::string & target_frame,
    const std::string & source_frame,
    tf2::Vector3 & output);
  double compute_vs_force_lambda(double depth) const;
  double compute_vs_enable_ramp(double now_sec) const;

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
