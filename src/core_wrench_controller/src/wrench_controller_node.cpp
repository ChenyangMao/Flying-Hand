#include "wrench_controller/wrench_controller_node.hpp"

#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace wrench_controller
{

WrenchControlNode::WrenchControlNode(const std::string & node_name)
: base::BaseNode(node_name)
{
  // Call Base::_initialize() and BaseNode::start_execute_timer() in the constructor
  // to mirror the ROS1 behavior of main() + Base::_initialize() + execute timer.

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  if (!this->_initialize()) {
    RCLCPP_FATAL(this->get_logger(), "WrenchControlNode initialization (initialize()) failed.");
  } else if (!this->start_execute_timer()) {
    RCLCPP_FATAL(this->get_logger(), "Failed to start execute timer (check 'execute_target' parameter).");
  }
}

bool WrenchControlNode::initialize()
{
  // First migrate the parameter loading logic from the ROS1 initialize() into ROS2 style.
  // Controller objects, subscriptions, publishers, and services will be added step by step.

  // Geofence and filtering parameters
  bool geofence_enable = this->declare_parameter<bool>("geofence.enable", false);
  double geofence_x_min = this->declare_parameter<double>("geofence.x_min", 0.0);
  double geofence_x_max = this->declare_parameter<double>("geofence.x_max", 0.0);
  double geofence_y_min = this->declare_parameter<double>("geofence.y_min", 0.0);
  double geofence_y_max = this->declare_parameter<double>("geofence.y_max", 0.0);
  double geofence_z_min = this->declare_parameter<double>("geofence.z_min", 0.0);
  double geofence_z_max = this->declare_parameter<double>("geofence.z_max", 0.0);

  int mean_filter_max_buffer_size =
    this->declare_parameter<int>("mean_filter_max_buffer_size", 15);
  int median_filter_max_buffer_size =
    this->declare_parameter<int>("median_filter_max_buffer_size", 15);

  // Tactile and sensor fusion parameters
  bool tactile_enable = this->declare_parameter<bool>("tactile.enable", false);
  bool sensor_fusion = this->declare_parameter<bool>("sensor_fusion", false);
  (void)tactile_enable;
  (void)sensor_fusion;

  // Frames and other controller parameters
  std::string target_frame =
    this->declare_parameter<std::string>("target_frame", "map");
  std::string sensor_frame =
    this->declare_parameter<std::string>("ft_sensor_frame", "ft_sensor");
  std::string robot_frame =
    this->declare_parameter<std::string>("robot_frame", "base_link");
  std::string world_frame =
    this->declare_parameter<std::string>("world_frame", "map");
  std::string contact_frame =
    this->declare_parameter<std::string>("contact_frame", "contact");
  std::string camera_frame =
    this->declare_parameter<std::string>("camera_frame", "camera");

  double xy_vel_limit = this->declare_parameter<double>("max_xy_vel", 12.0);
  double hover_thrust = this->declare_parameter<double>("hover_thrust", 0.5);
  double thrust_max = this->declare_parameter<double>("thrust_max", 1.0);
  double thrust_min = this->declare_parameter<double>("thrust_min", 0.15);
  double max_tilt_deg = this->declare_parameter<double>("max_tilt", 45.0);

  // Control loop frequency: already declared by BaseNode; just read it here if needed.

  bool filter_ft_data =
    this->declare_parameter<bool>("ft_data.filter", true);
  bool publish_filtered_ft_data =
    this->declare_parameter<bool>("ft_data.publish", true);
  publish_filtered_ft_data_ = publish_filtered_ft_data;

  double force_ff_coefficient =
    this->declare_parameter<double>("force_ff_coefficient", 0.01);
  double force_ff_coefficient_bias =
    this->declare_parameter<double>("force_ff_coeffcient_bias", 0.05);
  double velx_damping_coefficient =
    this->declare_parameter<double>("velx_damping_coefficient", 0.0);

  RCLCPP_INFO(
    this->get_logger(),
    "Geofence %s, X:[%.2f, %.2f] Y:[%.2f, %.2f] Z:[%.2f, %.2f]",
    geofence_enable ? "ENABLED" : "DISABLED",
    geofence_x_min, geofence_x_max,
    geofence_y_min, geofence_y_max,
    geofence_z_min, geofence_z_max);

  RCLCPP_INFO(
    this->get_logger(),
    "Force data filtering: %s, mean_buffer=%d, median_buffer=%d",
    filter_ft_data ? "ON" : "OFF",
    mean_filter_max_buffer_size,
    median_filter_max_buffer_size);

  RCLCPP_INFO(
    this->get_logger(),
    "Frames: world=%s, robot=%s, sensor=%s, target=%s, camera=%s, contact=%s",
    world_frame.c_str(),
    robot_frame.c_str(),
    sensor_frame.c_str(),
    target_frame.c_str(),
    camera_frame.c_str(),
    contact_frame.c_str());

  RCLCPP_INFO(
    this->get_logger(),
    "Hover thrust=%.3f, thrust range=[%.3f, %.3f], max tilt=%.1f deg, max_xy_vel=%.2f",
    hover_thrust, thrust_min, thrust_max, max_tilt_deg, xy_vel_limit);

  RCLCPP_INFO(
    this->get_logger(),
    "Force FF: coeff=%.4f, bias=%.4f, velx_damping=%.4f",
    force_ff_coefficient, force_ff_coefficient_bias, velx_damping_coefficient);

  // Force-loop PID gains (from wrench_px4_params.yaml style parameters)
  const double fx_p = this->declare_parameter<double>("fx.P", 0.0);
  const double fx_d = this->declare_parameter<double>("fx.D", 0.0);
  const double fx_min = this->declare_parameter<double>("fx.min", -0.3);
  const double fx_max = this->declare_parameter<double>("fx.max", 0.3);

  const double fy_p = this->declare_parameter<double>("fy.P", 0.0);
  const double fy_d = this->declare_parameter<double>("fy.D", 0.0);
  const double fy_min = this->declare_parameter<double>("fy.min", -0.2);
  const double fy_max = this->declare_parameter<double>("fy.max", 0.2);

  const double fz_p = this->declare_parameter<double>("fz.P", 0.0012);
  const double fz_i = this->declare_parameter<double>("fz.I", 0.0);
  const double fz_d = this->declare_parameter<double>("fz.D", 0.0);
  const double fz_integral_threshold =
    this->declare_parameter<double>("fz.integral_threshold", 20.0);
  const double fz_min = this->declare_parameter<double>("fz.min", -0.8);
  const double fz_max = this->declare_parameter<double>("fz.max", 0.8);

  // Create pose controller (reuse pose controller params)
  pose_controller::GeoFence pos_fence(
    geofence_enable,
    geofence_x_min, geofence_x_max,
    geofence_y_min, geofence_y_max,
    geofence_z_min, geofence_z_max);

  pose_controller_ = std::make_unique<pose_controller::PoseController>(
    pos_fence,
    target_frame,
    xy_vel_limit,
    thrust_min,
    thrust_max,
    max_tilt_deg,
    hover_thrust);

  // Create publishers and subscribers that are already known / straightforward to migrate.
  // Command publisher (mav_msgs::msg::AttitudeThrust)
  command_pub_ = this->create_publisher<mav_msgs::msg::AttitudeThrust>(
    "attitude_thrust_command", 10);

  // Filtered FT data publisher (sensor frame)
  filtered_ft_data_pub_ =
    this->create_publisher<geometry_msgs::msg::WrenchStamped>("ft_data_filtered", 10);

  // Wrench setpoint publisher (for debugging/monitoring, optional)
  ft_setpoint_pub_ =
    this->create_publisher<geometry_msgs::msg::WrenchStamped>("ft_setpoint", 10);

  // Core wrench controller instance
  wrench_controller_ = std::make_unique<wrench_controller::WrenchController>(
    sensor_frame,
    robot_frame,
    world_frame,
    camera_frame,
    target_frame,     // thrust output frame
    robot_frame,      // torque output frame
    hover_thrust,
    filter_ft_data,
    median_filter_max_buffer_size,
    mean_filter_max_buffer_size);

  // Configure PID gains inside the wrench controller
  wrench_controller_->configure_fx(fx_p, fx_d, fx_min, fx_max);
  wrench_controller_->configure_fy(fy_p, fy_d, fy_min, fy_max);
  wrench_controller_->configure_fz(
    fz_p, fz_i, fz_d, fz_integral_threshold, fz_min, fz_max);

  // Subscriptions for F/T data, setpoint and odometry
  ft_data_sub_ = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
    "ft_data",
    10,
    std::bind(&WrenchControlNode::ft_data_callback, this, std::placeholders::_1));

  ft_setpoint_sub_ = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
    "ft_setpoint",
    10,
    std::bind(&WrenchControlNode::ft_setpoint_callback, this, std::placeholders::_1));

  odometry_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "odometry",
    10,
    std::bind(&WrenchControlNode::odometry_callback, this, std::placeholders::_1));

  // Mode switch (pose-only vs motion-force control), equivalent to ROS1 "wrench_controller/switch"
  switch_sub_ = this->create_subscription<std_msgs::msg::Bool>(
    "wrench_controller/switch",
    10,
    std::bind(&WrenchControlNode::switch_callback, this, std::placeholders::_1));

  // TODO: In later steps, migrate PoseController/WrenchController to ROS2 and construct them here.
  // TODO: Create ROS2 subscriptions, publishers, and services here, keeping topic/service names compatible with ROS1.

  return true;
}

bool WrenchControlNode::execute()
{
  if (!pose_controller_ || !wrench_controller_) {
    return true;
  }

  // Pose controller: compute desired thrust from motion control
  tf2::Vector3 thrust_pose_des;
  tf2::Quaternion att_des;
  if (!pose_controller_->calculate_thrust(thrust_pose_des, att_des)) {
    return true;
  }

  tf2::Vector3 thrust_des = thrust_pose_des;

  // When in motion-force control mode, mix wrench-based thrust with pose-based thrust.
  if (mode_switch_) {
    tf2::Vector3 thrust_wrench_des;
    tf2::Vector3 torque_wrench_des;

    if (!wrench_controller_->calculate_thrust_torque(
        thrust_wrench_des,
        torque_wrench_des,
        *tf_buffer_,
        this->get_parameter("force_ff_coefficient").as_double(),
        this->get_parameter("force_ff_coeffcient_bias").as_double(),
        wrench_controller_ff_force_,
        this->get_parameter("velx_damping_coefficient").as_double())) {
      return true;
    }

    tf2::Matrix3x3 vel_mat(
      0.7, 0.0, 0.0,
      0.0, 1.0, 0.0,
      0.0, 0.0, 1.0);
    tf2::Vector3 contact_normal(-1.0, 0.0, 0.0);
    tf2::Vector3 force_constraint_vec(0.0, 0.0, 0.0);

    if (!combine_motion_and_force(
        thrust_wrench_des,
        thrust_pose_des,
        contact_normal,
        vel_mat,
        force_constraint_vec,
        this->get_parameter("target_frame").as_string(),
        thrust_des)) {
      return true;
    }
  }

  // Publish debug thrust vector (optional)
  if (thrust_debug_pub_) {
    geometry_msgs::msg::Vector3Stamped thrust_des_msg;
    thrust_des_msg.header.stamp = this->now();
    thrust_des_msg.vector.x = thrust_des.x();
    thrust_des_msg.vector.y = thrust_des.y();
    thrust_des_msg.vector.z = thrust_des.z();
    thrust_debug_pub_->publish(thrust_des_msg);
  }

  // Generate and publish attitude + thrust command
  mav_msgs::msg::AttitudeThrust drone_cmd;

  // For now, mirror PoseControlNode behavior: use roll/pitch/yaw from att_des if valid,
  // but default to identity when zero.
  if (att_des.x() == 0.0 && att_des.y() == 0.0 &&
      att_des.z() == 0.0 && att_des.w() == 0.0) {
    att_des.setValue(0.0, 0.0, 0.0, 1.0);
  }

  drone_cmd.attitude.x = att_des.x();
  drone_cmd.attitude.y = att_des.y();
  drone_cmd.attitude.z = att_des.z();
  drone_cmd.attitude.w = att_des.w();
  drone_cmd.thrust.x = thrust_des.x();
  drone_cmd.thrust.y = thrust_des.y();
  drone_cmd.thrust.z = thrust_des.z();

  if (should_publish_ && command_pub_) {
    command_pub_->publish(drone_cmd);
  }

  return true;
}

bool WrenchControlNode::combine_motion_and_force(
  const tf2::Vector3 & thrust_force,
  const tf2::Vector3 & thrust_motion,
  const tf2::Vector3 & contact_normal,
  const tf2::Matrix3x3 & vel_mat,
  const tf2::Vector3 & force_constraint_vec,
  const std::string & thrust_frame,
  tf2::Vector3 & out_thrust_des)
{
  try {
    // Transform from thrust frame (e.g. world/map) to contact frame
    geometry_msgs::msg::TransformStamped tf_cv_msg =
      tf_buffer_->lookupTransform(
        "contact",  // contact frame name is assumed; can be parameterized if needed
        thrust_frame,
        tf2::TimePointZero);

    tf2::Transform tf_cv;
    tf2::fromMsg(tf_cv_msg.transform, tf_cv);

    // Convert thrusts to contact frame
    tf2::Vector3 thrust_force_c = tf_cv * thrust_force;
    tf2::Vector3 thrust_motion_c = tf_cv * thrust_motion;

    // Force constraint matrix (I - vel_mat diagonal)
    tf2::Matrix3x3 force_mat(
      1.0 - vel_mat[0][0], 0.0, 0.0,
      0.0, 1.0 - vel_mat[1][1], 0.0,
      0.0, 0.0, 1.0 - vel_mat[2][2]);

    auto apply_contact_constraints =
      [](const tf2::Vector3 & vec_c,
         const tf2::Vector3 & contact_n,
         const tf2::Matrix3x3 & free_mat,
         const tf2::Vector3 & constraint_vec) -> tf2::Vector3
      {
        (void)contact_n;
        tf2::Vector3 vec_free_c = free_mat * vec_c;
        for (int i = 0; i < 3; ++i) {
          if (vec_c[i] * constraint_vec[i] < 0.0) {
            vec_free_c[i] = 0.0;
          }
        }
        return vec_free_c;
      };

    tf2::Vector3 thrust_force_constrained_c =
      apply_contact_constraints(thrust_force_c, contact_normal, force_mat, force_constraint_vec);

    tf2::Vector3 thrust_motion_constrained_c =
      apply_contact_constraints(thrust_motion_c, contact_normal, vel_mat, -force_constraint_vec);

    // Combine and transform back
    out_thrust_des = tf_cv.inverse() * (thrust_force_constrained_c + thrust_motion_constrained_c);

    // Constrain horizontal thrust using PoseController helper
    if (pose_controller_) {
      tf2::Vector3 thrust_h_des = pose_controller_->constrain_horizontal_thrust(out_thrust_des);
      out_thrust_des.setX(thrust_h_des.x());
      out_thrust_des.setY(thrust_h_des.y());
    }

    return true;
  } catch (const tf2::TransformException &) {
    return false;
  }
}

void WrenchControlNode::ft_data_callback(
  const geometry_msgs::msg::WrenchStamped::SharedPtr msg)
{
  if (!wrench_controller_) {
    return;
  }

  auto filtered = wrench_controller_->update_state(*msg, *tf_buffer_);

  if (publish_filtered_ft_data_ && filtered_ft_data_pub_) {
    filtered_ft_data_pub_->publish(filtered);
  }
}

void WrenchControlNode::ft_setpoint_callback(
  const geometry_msgs::msg::WrenchStamped::SharedPtr msg)
{
  if (!wrench_controller_) {
    return;
  }

  wrench_controller_->update_target(*msg, *tf_buffer_);

  if (ft_setpoint_pub_) {
    ft_setpoint_pub_->publish(*msg);
  }
}

void WrenchControlNode::odometry_callback(
  const nav_msgs::msg::Odometry::SharedPtr msg)
{
  if (!wrench_controller_) {
    return;
  }

  wrench_controller_->update_odom_state(*msg, *tf_buffer_);
}

void WrenchControlNode::switch_callback(
  const std_msgs::msg::Bool::SharedPtr msg)
{
  mode_switch_ = msg->data;
  RCLCPP_INFO(
    this->get_logger(),
    "Mode switch changed: motion-force control %s",
    mode_switch_ ? "ENABLED" : "DISABLED");
}

}  // namespace wrench_controller

