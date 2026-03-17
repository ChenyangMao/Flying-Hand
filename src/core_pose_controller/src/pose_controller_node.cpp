#include "pose_controller/pose_controller_node.hpp"

namespace pose_controller
{

PoseControlNode::PoseControlNode(const std::string & node_name)
: base::BaseNode(node_name)
{
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Mirror ROS1 behavior: call Base::_initialize() and start the execute timer.
  if (!this->_initialize()) {
    RCLCPP_FATAL(this->get_logger(), "PoseControlNode initialization (initialize()) failed.");
  } else if (!this->start_execute_timer()) {
    RCLCPP_FATAL(this->get_logger(), "Failed to start execute timer (check 'execute_target' parameter).");
  }
}

bool PoseControlNode::initialize()
{
  // Geofence parameters
  bool geofence_enable = this->declare_parameter<bool>("geofence.enable", false);
  double geofence_x_min = this->declare_parameter<double>("geofence.x_min", 0.0);
  double geofence_x_max = this->declare_parameter<double>("geofence.x_max", 0.0);
  double geofence_y_min = this->declare_parameter<double>("geofence.y_min", 0.0);
  double geofence_y_max = this->declare_parameter<double>("geofence.y_max", 0.0);
  double geofence_z_min = this->declare_parameter<double>("geofence.z_min", 0.0);
  double geofence_z_max = this->declare_parameter<double>("geofence.z_max", 0.0);

  GeoFence pos_fence(
    geofence_enable,
    geofence_x_min, geofence_x_max,
    geofence_y_min, geofence_y_max,
    geofence_z_min, geofence_z_max);

  // Other controller parameters
  std::string target_frame_str =
    this->declare_parameter<std::string>("target_frame", "map");
  double xy_vel_limit = this->declare_parameter<double>("max_xy_vel", 12.0);
  double hover_thrust = this->declare_parameter<double>("hover_thrust", 0.5);
  double voltage_compensation_gain =
    this->declare_parameter<double>("voltage_compensation_gain", 0.0869);
  double voltage_compensation_offset =
    this->declare_parameter<double>("voltage_compensation_offset", -0.0664);
  double voltage_compensation_update_rate =
    this->declare_parameter<double>("voltage_compensation_update_rate", 0.005);
  double thrust_max = this->declare_parameter<double>("thrust_max", 1.0);
  double thrust_min = this->declare_parameter<double>("thrust_min", 0.15);
  double max_tilt_deg = this->declare_parameter<double>("max_tilt", 45.0);

  // Control loop frequency: already declared by BaseNode; just read it here if needed.

  RCLCPP_INFO(
    this->get_logger(),
    "Pose controller geofence %s, X:[%.2f, %.2f] Y:[%.2f, %.2f] Z:[%.2f, %.2f]",
    geofence_enable ? "ENABLED" : "DISABLED",
    geofence_x_min, geofence_x_max,
    geofence_y_min, geofence_y_max,
    geofence_z_min, geofence_z_max);

  RCLCPP_INFO(
    this->get_logger(),
    "Pose controller params: target_frame=%s, xy_vel_limit=%.2f, hover_thrust=%.3f, "
    "thrust_range=[%.3f, %.3f], max_tilt=%.1f deg, V_comp(gain=%.4f, offset=%.4f, rate=%.4f)",
    target_frame_str.c_str(),
    xy_vel_limit,
    hover_thrust,
    thrust_min, thrust_max,
    max_tilt_deg,
    voltage_compensation_gain,
    voltage_compensation_offset,
    voltage_compensation_update_rate);

  // Create the new pose controller
  pose_controller_ = std::make_unique<pose_controller::PoseController>(
    pos_fence,
    target_frame_str,
    xy_vel_limit,
    thrust_min,
    thrust_max,
    max_tilt_deg,
    hover_thrust);

  // Load P/I/D/FF and limits from parameters (vector<double>, similar to ROS1 behavior)
  auto get_vec3 = [this](const std::string & name, const std::array<double, 3> & defaults)
    -> Eigen::Vector3d
  {
    std::vector<double> vals =
      this->declare_parameter<std::vector<double>>(name, {defaults[0], defaults[1], defaults[2]});
    if (vals.size() < 3) {
      vals.resize(3, 0.0);
    }
    return Eigen::Vector3d(vals[0], vals[1], vals[2]);
  };

  Eigen::Vector3d P = get_vec3("P", {0.0, 0.0, 0.0});
  Eigen::Vector3d I = get_vec3("I", {0.0, 0.0, 0.0});
  Eigen::Vector3d D = get_vec3("D", {0.0, 0.0, 0.0});
  Eigen::Vector3d FF = get_vec3("FF", {0.0, 0.0, 0.0});
  Eigen::Vector3d integral_threshold =
    get_vec3("integral_threshold", {4.0, 4.0, 4.0});

  const double min_default = std::numeric_limits<double>::lowest();
  const double max_default = std::numeric_limits<double>::max();
  Eigen::Vector3d minimum =
    get_vec3("min", {min_default, min_default, min_default});
  Eigen::Vector3d maximum =
    get_vec3("max", {max_default, max_default, max_default});

  pose_controller_->configure_gains(
    P, I, D, FF, integral_threshold, minimum, maximum);

  // Publisher for attitude and thrust commands
  command_pub_ = this->create_publisher<mav_msgs::msg::AttitudeThrust>(
    "attitude_thrust_command", 10);

  // Subscribers
  tracking_point_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "tracking_point",
    10,
    std::bind(&PoseControlNode::tracking_point_callback, this, std::placeholders::_1));

  odometry_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "odometry",
    rclcpp::SensorDataQoS(),
    std::bind(&PoseControlNode::odometry_callback, this, std::placeholders::_1));

  arm_sub_ = this->create_subscription<std_msgs::msg::Bool>(
    "arm_active",
    10,
    std::bind(&PoseControlNode::arm_callback, this, std::placeholders::_1));

  // Service
  publish_control_srv_ = this->create_service<std_srvs::srv::SetBool>(
    "publish_control",
    std::bind(
      &PoseControlNode::publish_control_callback,
      this,
      std::placeholders::_1,
      std::placeholders::_2));

  return true;
}

bool PoseControlNode::execute()
{
  if (!pose_controller_) {
    return true;
  }

  tf2::Vector3 thrust_des;
  if (!pose_controller_->calculate_thrust(thrust_des)) {
    return true;
  }

  tf2::Quaternion att_sp;
  tf2::Vector3 total_thrust;
  std::tie(att_sp, total_thrust) =
    pose_controller_->calculate_attitude_thrust(thrust_des);

  mav_msgs::msg::AttitudeThrust drone_cmd;
  drone_cmd.attitude.x = 0.0;
  drone_cmd.attitude.y = 0.0;
  drone_cmd.attitude.z = 0.0;
  drone_cmd.attitude.w = 1.0;
  drone_cmd.thrust.x = thrust_des.x();
  drone_cmd.thrust.y = thrust_des.y();
  drone_cmd.thrust.z = thrust_des.z();

  if (should_publish_ && command_pub_) {
    command_pub_->publish(drone_cmd);
  }

  return true;
}

void PoseControlNode::tracking_point_callback(
  const nav_msgs::msg::Odometry::SharedPtr msg)
{
  if (!pose_controller_ || !tf_buffer_) {
    return;
  }
  pose_controller_->update_target(*msg, *tf_buffer_);
}

void PoseControlNode::odometry_callback(
  const nav_msgs::msg::Odometry::SharedPtr msg)
{
  if (!pose_controller_ || !tf_buffer_) {
    return;
  }
  pose_controller_->update_state(*msg, *tf_buffer_);
}

void PoseControlNode::arm_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
  if (!pose_controller_) {
    return;
  }

  if (msg->data) {
    RCLCPP_WARN(this->get_logger(), "Resetting PoseController due to arm_active=true.");
    pose_controller_->reset();
  }
}

void PoseControlNode::publish_control_callback(
  const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
  std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
  if (!pose_controller_) {
    response->success = false;
    return;
  }

  // Reset integrators when just starting to publish
  if (!should_publish_ && request->data) {
    pose_controller_->reset();
  }

  should_publish_ = request->data;
  response->success = true;

  RCLCPP_WARN(
    this->get_logger(),
    "Received publish_control request: %s",
    should_publish_ ? "true" : "false");
}

}  // namespace pose_controller

