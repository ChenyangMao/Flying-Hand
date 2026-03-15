#ifndef ROS2_CONTROL_STACK_POSE_CONTROLLER_NODE_HPP_
#define ROS2_CONTROL_STACK_POSE_CONTROLLER_NODE_HPP_

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

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/LinearMath/Vector3.h"
#include "tf2/LinearMath/Quaternion.h"

#include "base/BaseNode.hpp"
#include "pose_controller/pose_controller.hpp"

namespace pose_controller
{

class PoseControlNode : public base::BaseNode
{
public:
  explicit PoseControlNode(const std::string & node_name);
  ~PoseControlNode() override = default;

  bool initialize() override;
  bool execute() override;

private:
  // Subscribers
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr tracking_point_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr arm_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr tracking_target_sub_;  // placeholder, real type is PoseCtrlTarget

  // Publishers
  rclcpp::Publisher<mav_msgs::msg::AttitudeThrust>::SharedPtr command_pub_;

  // Services
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr publish_control_srv_;

  // Variables
  bool should_publish_{true};

  // Core pose controller and TF
  std::unique_ptr<pose_controller::PoseController> pose_controller_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Callbacks
  void tracking_point_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void arm_callback(const std_msgs::msg::Bool::SharedPtr msg);
  void publish_control_callback(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response);
};

}  // namespace pose_controller

#endif  // ROS2_CONTROL_STACK_POSE_CONTROLLER_NODE_HPP_

