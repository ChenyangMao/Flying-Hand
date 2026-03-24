#include "core_drone_interface/drone_interface_node.hpp"
#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>
#include <functional>

using namespace std::placeholders;

namespace core_drone_interface
{

DroneInterfaceNode::DroneInterfaceNode(const rclcpp::NodeOptions & options)
: base::BaseNode("drone_interface_node", options)
{
  if (!this->_initialize()) {
    RCLCPP_FATAL(this->get_logger(), "DroneInterfaceNode initialization failed.");
  } else if (!this->start_execute_timer()) {
    RCLCPP_FATAL(this->get_logger(), "Failed to start execute timer (check 'execute_target' parameter).");
  }
}

bool DroneInterfaceNode::initialize()
{
  drone_interface_name_ = this->declare_parameter<std::string>("drone_interface", "GazeboInterface");

  pluginlib::ClassLoader<DroneInterface> loader(
    "core_drone_interface", "core_drone_interface::DroneInterface");
  try {
    drone_interface_ = loader.createSharedInstance(drone_interface_name_);
    drone_interface_->initialize(this);
  } catch (const pluginlib::PluginlibException & ex) {
    RCLCPP_ERROR(this->get_logger(), "Failed to load DroneInterface plugin '%s': %s",
      drone_interface_name_.c_str(), ex.what());
    return false;
  }

  drone_command_srv_ = this->create_service<core_drone_interface::srv::DroneCommand>(
    "drone_command",
    std::bind(&DroneInterfaceNode::drone_command_callback, this, _1, _2, _3));

  attitude_thrust_sub_ = this->create_subscription<mav_msgs::msg::AttitudeThrust>(
    "attitude_thrust_command", 10, std::bind(&DroneInterfaceNode::attitude_thrust_callback, this, _1));
  rate_thrust_sub_ = this->create_subscription<mav_msgs::msg::RateThrust>(
    "rate_thrust_command", 10, std::bind(&DroneInterfaceNode::rate_thrust_callback, this, _1));
  roll_pitch_yawrate_thrust_sub_ = this->create_subscription<mav_msgs::msg::RollPitchYawrateThrust>(
    "roll_pitch_yawrate_thrust_command", 10, std::bind(&DroneInterfaceNode::roll_pitch_yawrate_thrust_callback, this, _1));
  torque_thrust_sub_ = this->create_subscription<mav_msgs::msg::TorqueThrust>(
    "torque_thrust_command", 10, std::bind(&DroneInterfaceNode::torque_thrust_callback, this, _1));
  velocity_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
    "velocity_command", 10, std::bind(&DroneInterfaceNode::velocity_callback, this, _1));
  pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "pose_command", 10, std::bind(&DroneInterfaceNode::pose_callback, this, _1));

  is_armed_pub_ = this->create_publisher<std_msgs::msg::Bool>("is_armed", 1);
  has_control_pub_ = this->create_publisher<std_msgs::msg::Bool>("has_control", 1);

  RCLCPP_INFO(this->get_logger(), "DroneInterfaceNode initialized with plugin '%s'", drone_interface_name_.c_str());
  return true;
}

bool DroneInterfaceNode::execute()
{
  std_msgs::msg::Bool msg;
  msg.data = drone_interface_->is_armed();
  is_armed_pub_->publish(msg);
  msg.data = drone_interface_->has_control();
  has_control_pub_->publish(msg);
  return true;
}

void DroneInterfaceNode::drone_command_callback(
  const std::shared_ptr<rmw_request_id_t>,
  const std::shared_ptr<core_drone_interface::srv::DroneCommand::Request> request,
  std::shared_ptr<core_drone_interface::srv::DroneCommand::Response> response)
{
  switch (request->command) {
    case core_drone_interface::srv::DroneCommand::Request::REQUEST_CONTROL:
      response->success = drone_interface_->request_control();
      break;
    case core_drone_interface::srv::DroneCommand::Request::ARM:
      response->success = drone_interface_->arm();
      break;
    case core_drone_interface::srv::DroneCommand::Request::DISARM:
      response->success = drone_interface_->disarm();
      break;
    default:
      response->success = false;
      RCLCPP_WARN(this->get_logger(), "Unknown drone_command: %u", request->command);
  }
}

void DroneInterfaceNode::attitude_thrust_callback(const mav_msgs::msg::AttitudeThrust::SharedPtr msg)
{
  drone_interface_->command_attitude_thrust(*msg);
}

void DroneInterfaceNode::rate_thrust_callback(const mav_msgs::msg::RateThrust::SharedPtr msg)
{
  drone_interface_->command_rate_thrust(*msg);
}

void DroneInterfaceNode::roll_pitch_yawrate_thrust_callback(const mav_msgs::msg::RollPitchYawrateThrust::SharedPtr msg)
{
  drone_interface_->command_roll_pitch_yawrate_thrust(*msg);
}

void DroneInterfaceNode::torque_thrust_callback(const mav_msgs::msg::TorqueThrust::SharedPtr msg)
{
  drone_interface_->command_torque_thrust(*msg);
}

void DroneInterfaceNode::velocity_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
  drone_interface_->command_velocity(*msg);
}

void DroneInterfaceNode::pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  drone_interface_->command_pose(*msg);
}

}  // namespace core_drone_interface

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto node = std::make_shared<core_drone_interface::DroneInterfaceNode>(options);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
