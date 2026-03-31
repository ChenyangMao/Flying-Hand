#include "wrench_controller/wrench_controller_node.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <vector>

#include <Eigen/Dense>

#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace wrench_controller
{

namespace
{
constexpr double kPi = 3.14159265358979323846;
}  // namespace

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
  target_frame_ = this->declare_parameter<std::string>("target_frame", "map");
  sensor_frame_ = this->declare_parameter<std::string>("ft_sensor_frame", "ft_sensor");
  robot_frame_ = this->declare_parameter<std::string>("robot_frame", "base_link");
  world_frame_ = this->declare_parameter<std::string>("world_frame", "map");
  contact_frame_ = this->declare_parameter<std::string>("contact_frame", "contact");
  mix_vel_x_ = this->declare_parameter<double>("mix_vel_x", 0.7);
  mix_vel_y_ = this->declare_parameter<double>("mix_vel_y", 1.0);
  mix_vel_z_ = this->declare_parameter<double>("mix_vel_z", 1.0);
  camera_frame_ = this->declare_parameter<std::string>("camera_frame", "camera");

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

  force_ff_coefficient_ =
    this->declare_parameter<double>("force_ff_coefficient", 0.01);
  force_ff_coefficient_bias_ =
    this->declare_parameter<double>("force_ff_coeffcient_bias", 0.05);
  velx_damping_coefficient_ =
    this->declare_parameter<double>("velx_damping_coefficient", 0.0);

  RCLCPP_INFO(
    this->get_logger(),
    "mix_vel (pose weight per axis) = (%.2f, %.2f, %.2f) → wrench force gain (1-mix) = (%.2f, %.2f, %.2f)",
    mix_vel_x_, mix_vel_y_, mix_vel_z_,
    1.0 - mix_vel_x_, 1.0 - mix_vel_y_, 1.0 - mix_vel_z_);

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
    world_frame_.c_str(),
    robot_frame_.c_str(),
    sensor_frame_.c_str(),
    target_frame_.c_str(),
    camera_frame_.c_str(),
    contact_frame_.c_str());

  RCLCPP_INFO(
    this->get_logger(),
    "Hover thrust=%.3f, thrust range=[%.3f, %.3f], max tilt=%.1f deg, max_xy_vel=%.2f",
    hover_thrust, thrust_min, thrust_max, max_tilt_deg, xy_vel_limit);

  RCLCPP_INFO(
    this->get_logger(),
    "Force FF: coeff=%.4f, bias=%.4f, velx_damping=%.4f",
    force_ff_coefficient_, force_ff_coefficient_bias_, velx_damping_coefficient_);

  // Force-loop PID gains (from wrench_px4_params.yaml style parameters)
  const double fx_p = this->declare_parameter<double>("fx.P", 0.0);
  const double fx_i = this->declare_parameter<double>("fx.I", 0.0);
  const double fx_d = this->declare_parameter<double>("fx.D", 0.0);
  const double fx_integral_threshold =
    this->declare_parameter<double>("fx.integral_threshold", 20.0);
  const double fx_min = this->declare_parameter<double>("fx.min", -0.3);
  const double fx_max = this->declare_parameter<double>("fx.max", 0.3);
  const double fx_ff = this->declare_parameter<double>("fx.FF", 0.0);
  const double fx_constant = this->declare_parameter<double>("fx.constant", 0.0);
  const bool fx_use_negative_gains =
    this->declare_parameter<bool>("fx.use_negative_gains", false);
  const double fx_neg_p = this->declare_parameter<double>("fx.neg_P", 0.0);
  const double fx_neg_i = this->declare_parameter<double>("fx.neg_I", 0.0);
  const double fx_neg_d = this->declare_parameter<double>("fx.neg_D", 0.0);
  const double fx_neg_ff = this->declare_parameter<double>("fx.neg_FF", 0.0);
  const double x_hold_p = this->declare_parameter<double>("x_hold.P", 0.0);
  const double x_hold_d = this->declare_parameter<double>("x_hold.D", 0.0);

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
    target_frame_,
    xy_vel_limit,
    thrust_min,
    thrust_max,
    max_tilt_deg,
    hover_thrust);

  auto get_vec3 = [this](const std::string & name, const std::array<double, 3> & defaults)
    -> Eigen::Vector3d {
    std::vector<double> vals =
      this->declare_parameter<std::vector<double>>(
      name, {defaults[0], defaults[1], defaults[2]});
    if (vals.size() < 3) {
      vals.resize(3, 0.0);
    }
    return Eigen::Vector3d(vals[0], vals[1], vals[2]);
  };

  // Tuned values: config/wrench_px4_params.yaml (and gazebo overlay) under pose_controller/.
  // Defaults below are only fallbacks if a key is missing from YAML (ROS2 declare_parameter).
  const std::array<double, 3> z3 = {0.0, 0.0, 0.0};
  const std::array<double, 3> vs_kp_defaults = {0.25, 0.18, 0.20};
  const std::array<double, 3> vs_ki_defaults = {0.0, 0.0, 0.0};
  const std::array<double, 3> vs_vel_defaults = {0.12, 0.06, 0.08};
  const std::array<double, 3> vs_thrust_defaults = {0.06, 0.04, 0.05};
  const double lim_lo = std::numeric_limits<double>::lowest();
  const double lim_hi = std::numeric_limits<double>::max();
  const std::array<double, 3> min_fallback = {lim_lo, lim_lo, lim_lo};
  const std::array<double, 3> max_fallback = {lim_hi, lim_hi, lim_hi};

  Eigen::Vector3d pose_P = get_vec3("pose_controller.P", z3);
  Eigen::Vector3d pose_I = get_vec3("pose_controller.I", z3);
  Eigen::Vector3d pose_D = get_vec3("pose_controller.D", z3);
  Eigen::Vector3d pose_FF = get_vec3("pose_controller.FF", z3);
  Eigen::Vector3d pose_integral_threshold =
    get_vec3("pose_controller.integral_threshold", z3);
  Eigen::Vector3d pose_minimum = get_vec3("pose_controller.min", min_fallback);
  Eigen::Vector3d pose_maximum = get_vec3("pose_controller.max", max_fallback);
  vs_kp_ = get_vec3("visual_servo.kp", vs_kp_defaults);
  vs_ki_ = get_vec3("visual_servo.ki", vs_ki_defaults);
  vs_max_body_velocity_ =
    get_vec3("visual_servo.max_body_velocity", vs_vel_defaults);
  vs_max_thrust_delta_ =
    get_vec3("visual_servo.max_thrust_delta", vs_thrust_defaults);
  thrust_z_damping_coeff_ =
    this->declare_parameter<double>("visual_servo.thrust_z_damping_coeff", 0.01);
  vs_depth_min_ = this->declare_parameter<double>("visual_servo.depth_min", 0.55);
  vs_depth_max_ = this->declare_parameter<double>("visual_servo.depth_max", 0.75);
  vs_timeout_sec_ = this->declare_parameter<double>("visual_servo.timeout_sec", 0.5);
  vs_cmd_alpha_ = this->declare_parameter<double>("visual_servo.cmd_alpha", 0.25);
  vs_enable_ramp_sec_ =
    this->declare_parameter<double>("visual_servo.enable_ramp_sec", 1.0);

  pose_controller_->configure_gains(
    pose_P, pose_I, pose_D, pose_FF, pose_integral_threshold, pose_minimum, pose_maximum);

  RCLCPP_INFO(
    this->get_logger(),
    "PoseController gains: P=(%.3f,%.3f,%.3f) D=(%.3f,%.3f,%.3f)",
    pose_P.x(), pose_P.y(), pose_P.z(),
    pose_D.x(), pose_D.y(), pose_D.z());
  RCLCPP_INFO(
    this->get_logger(),
    "Visual servo gains: kp=(%.3f,%.3f,%.3f) ki=(%.3f,%.3f,%.3f), "
    "vmax=(%.3f,%.3f,%.3f) thrust_max=(%.3f,%.3f,%.3f), "
    "alpha=%.2f ramp=%.2fs depth blend=[%.3f, %.3f], timeout=%.2fs",
    vs_kp_.x(), vs_kp_.y(), vs_kp_.z(),
    vs_ki_.x(), vs_ki_.y(), vs_ki_.z(),
    vs_max_body_velocity_.x(), vs_max_body_velocity_.y(), vs_max_body_velocity_.z(),
    vs_max_thrust_delta_.x(), vs_max_thrust_delta_.y(), vs_max_thrust_delta_.z(),
    vs_cmd_alpha_, vs_enable_ramp_sec_,
    vs_depth_min_, vs_depth_max_, vs_timeout_sec_);

  // Create publishers and subscribers that are already known / straightforward to migrate.
  // Command publisher (mav_msgs::msg::AttitudeThrust)
  command_pub_ = this->create_publisher<mav_msgs::msg::AttitudeThrust>(
    "attitude_thrust_command", 10);

  // Filtered FT data publisher (sensor frame)
  filtered_ft_data_pub_ =
    this->create_publisher<geometry_msgs::msg::WrenchStamped>("ft_data_filtered", 10);
  thrust_debug_pub_ =
    this->create_publisher<geometry_msgs::msg::Vector3Stamped>("thrust_debug", 10);

  // Core wrench controller instance
  wrench_controller_ = std::make_unique<wrench_controller::WrenchController>(
    sensor_frame_,
    robot_frame_,
    world_frame_,
    camera_frame_,
    target_frame_,     // thrust output frame
    robot_frame_,      // torque output frame
    hover_thrust,
    filter_ft_data,
    median_filter_max_buffer_size,
    mean_filter_max_buffer_size);

  // Configure PID gains inside the wrench controller
  wrench_controller_->configure_fx(
    fx_p, fx_i, fx_d, fx_integral_threshold, fx_min, fx_max, fx_ff, fx_constant);
  wrench_controller_->configure_fx_negative_gains(
    fx_use_negative_gains, fx_neg_p, fx_neg_i, fx_neg_d, fx_neg_ff);
  wrench_controller_->configure_x_hold_pd(x_hold_p, x_hold_d);
  wrench_controller_->configure_fy(fy_p, fy_d, fy_min, fy_max);
  wrench_controller_->configure_fz(
    fz_p, fz_i, fz_d, fz_integral_threshold, fz_min, fz_max);
  RCLCPP_INFO(
    this->get_logger(),
    "Force PI/PD: fx(P=%.4f I=%.4f D=%.4f), x_hold(P=%.4f D=%.4f), out=[%.3f, %.3f]",
    fx_p, fx_i, fx_d, x_hold_p, x_hold_d, fx_min, fx_max);

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
    rclcpp::SensorDataQoS(),
    std::bind(&WrenchControlNode::odometry_callback, this, std::placeholders::_1));

  tracking_point_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "tracking_point",
    10,
    std::bind(&WrenchControlNode::tracking_point_callback, this, std::placeholders::_1));

  // Mode switch (pose-only vs motion-force control), equivalent to ROS1 "wrench_controller/switch"
  switch_sub_ = this->create_subscription<std_msgs::msg::Bool>(
    "wrench_controller/switch",
    10,
    std::bind(&WrenchControlNode::switch_callback, this, std::placeholders::_1));

  vs_enable_sub_ = this->create_subscription<std_msgs::msg::Bool>(
    "/visual_servo/enable",
    10,
    std::bind(&WrenchControlNode::vs_enable_callback, this, std::placeholders::_1));

  vs_active_sub_ = this->create_subscription<std_msgs::msg::Bool>(
    "/visual_servo/active",
    10,
    std::bind(&WrenchControlNode::vs_active_callback, this, std::placeholders::_1));

  vs_velocity_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
    "/visual_servo/cmd_vel",
    10,
    std::bind(&WrenchControlNode::vs_velocity_callback, this, std::placeholders::_1));

  vs_depth_sub_ = this->create_subscription<geometry_msgs::msg::Vector3Stamped>(
    "/visual_servo/depth",
    10,
    std::bind(&WrenchControlNode::vs_depth_callback, this, std::placeholders::_1));

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
  tf2::Quaternion pose_target_att;
  if (!pose_controller_->calculate_thrust(thrust_pose_des, pose_target_att)) {
    return true;
  }

  RCLCPP_INFO_THROTTLE(
    this->get_logger(),
    *this->get_clock(),
    1000,
    "POS CTRL  pose_thrust_xyz=(%.4f, %.4f, %.4f)",
    thrust_pose_des.x(), thrust_pose_des.y(), thrust_pose_des.z());

  const double now_sec = this->now().seconds();
  const bool vs_active_recent =
    visual_servo_active_ && (now_sec - last_vs_active_stamp_sec_) <= vs_timeout_sec_;
  const bool vs_cmd_recent =
    last_vs_cmd_stamp_sec_ > 0.0 && (now_sec - last_vs_cmd_stamp_sec_) <= vs_timeout_sec_;
  const bool vs_depth_recent =
    last_vs_depth_stamp_sec_ > 0.0 && (now_sec - last_vs_depth_stamp_sec_) <= vs_timeout_sec_;

  tf2::Vector3 thrust_vs_des = thrust_pose_des;
  tf2::Vector3 desired_body_velocity;
  bool vs_valid = false;

  if (visual_servo_enabled_ && vs_active_recent && vs_cmd_recent &&
    get_visual_servo_body_velocity(desired_body_velocity))
  {
    desired_body_velocity.setX(
      std::clamp(
        desired_body_velocity.x(),
        -vs_max_body_velocity_.x(),
        vs_max_body_velocity_.x()));
    desired_body_velocity.setY(
      std::clamp(
        desired_body_velocity.y(),
        -vs_max_body_velocity_.y(),
        vs_max_body_velocity_.y()));
    desired_body_velocity.setZ(
      std::clamp(
        desired_body_velocity.z(),
        -vs_max_body_velocity_.z(),
        vs_max_body_velocity_.z()));

    const double alpha = std::clamp(vs_cmd_alpha_, 0.0, 1.0);
    filtered_vs_body_velocity_ =
      filtered_vs_body_velocity_ * (1.0 - alpha) + desired_body_velocity * alpha;

    const Eigen::Vector3d desired_vel(
      filtered_vs_body_velocity_.x(),
      filtered_vs_body_velocity_.y(),
      filtered_vs_body_velocity_.z());
    const Eigen::Vector3d current_vel(
      current_body_velocity_.x(),
      current_body_velocity_.y(),
      current_body_velocity_.z());
    const Eigen::Vector3d vel_error = desired_vel - current_vel;
    const double execute_target_hz =
      std::max(this->get_parameter("execute_target").as_double(), 1.0);
    const double dt = std::max(1.0 / execute_target_hz, 1e-3);

    vs_integral_error_ += vel_error * dt;
    constexpr double max_integral_norm = 2.0;
    if (vs_integral_error_.norm() > max_integral_norm) {
      vs_integral_error_ = vs_integral_error_.normalized() * max_integral_norm;
    }

    const double thrust_z_damping = -thrust_z_damping_coeff_ * current_vel.z();
    Eigen::Vector3d thrust_vs_delta_body =
      vs_kp_.cwiseProduct(vel_error) +
      vs_ki_.cwiseProduct(vs_integral_error_) +
      Eigen::Vector3d(0.0, 0.0, thrust_z_damping);

    thrust_vs_delta_body = thrust_vs_delta_body.cwiseMin(vs_max_thrust_delta_);
    thrust_vs_delta_body = thrust_vs_delta_body.cwiseMax(-vs_max_thrust_delta_);

    const double ramp = compute_vs_enable_ramp(now_sec);
    thrust_vs_delta_body *= ramp;

    tf2::Vector3 thrust_vs_delta_world;
    if (!rotate_vector_between_frames(
        tf2::Vector3(
          thrust_vs_delta_body.x(),
          thrust_vs_delta_body.y(),
          thrust_vs_delta_body.z()),
        target_frame_,
        robot_frame_,
        thrust_vs_delta_world))
    {
      reset_visual_servo_pi();
      return true;
    }

    thrust_vs_des += thrust_vs_delta_world;
    vs_valid = true;

    RCLCPP_INFO_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      1000,
      "VS CTRL  depth=%.3f  ramp=%.2f  cmd_v_body=(%.3f, %.3f, %.3f)  meas_v_body=(%.3f, %.3f, %.3f)  thrust_delta_body=(%.3f, %.3f, %.3f)  thrust_delta_world=(%.3f, %.3f, %.3f)",
      latest_vs_depth_,
      ramp,
      filtered_vs_body_velocity_.x(),
      filtered_vs_body_velocity_.y(),
      filtered_vs_body_velocity_.z(),
      current_body_velocity_.x(), current_body_velocity_.y(), current_body_velocity_.z(),
      thrust_vs_delta_body.x(), thrust_vs_delta_body.y(), thrust_vs_delta_body.z(),
      thrust_vs_delta_world.x(), thrust_vs_delta_world.y(), thrust_vs_delta_world.z());
  } else {
    reset_visual_servo_pi();
  }

  tf2::Vector3 thrust_des = vs_valid ? thrust_vs_des : thrust_pose_des;

  tf2::Vector3 thrust_wrench_des;
  tf2::Vector3 torque_wrench_des;
  bool wrench_valid = false;
  if (mode_switch_) {
    wrench_valid = wrench_controller_->calculate_thrust_torque(
      thrust_wrench_des,
      torque_wrench_des,
      *tf_buffer_,
      force_ff_coefficient_,
      force_ff_coefficient_bias_,
      wrench_controller_ff_force_,
      velx_damping_coefficient_);
  }

  const double measured_force_mag = std::abs(wrench_controller_->meas_force_x());
  const double target_force_mag = std::abs(wrench_controller_->target_force_x());
  const bool contact_force_observed =
    measured_force_mag >= std::max(0.5, 0.2 * std::max(1.0, target_force_mag));

  if (mode_switch_ && wrench_valid) {
    if (vs_valid && vs_depth_recent) {
      const double lambda = compute_vs_force_lambda(latest_vs_depth_);
      thrust_des.setX(
        (1.0 - lambda) * thrust_vs_des.x() + lambda * thrust_wrench_des.x());
      thrust_des.setY(thrust_vs_des.y());
      thrust_des.setZ(thrust_vs_des.z());

      RCLCPP_INFO_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        1000,
        "VS/WRENCH  depth=%.3f  lambda=%.3f  vs_x=%.4f  wrench_x=%.4f  out=(%.4f, %.4f, %.4f)  meas_fx=%.3f  tgt_fx=%.3f",
        latest_vs_depth_,
        lambda,
        thrust_vs_des.x(),
        thrust_wrench_des.x(),
        thrust_des.x(),
        thrust_des.y(),
        thrust_des.z(),
        wrench_controller_->meas_force_x(),
        wrench_controller_->target_force_x());
    } else if (contact_force_observed) {
      thrust_des = thrust_pose_des;
      thrust_des.setX(thrust_wrench_des.x());

      RCLCPP_INFO_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        1000,
        "WRENCH FALLBACK  wrench_x=%.4f  pose_y=%.4f  pose_z=%.4f  meas_fx=%.3f  tgt_fx=%.3f",
        thrust_wrench_des.x(),
        thrust_des.y(),
        thrust_des.z(),
        wrench_controller_->meas_force_x(),
        wrench_controller_->target_force_x());
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
  drone_cmd.header.stamp = this->now();
  drone_cmd.header.frame_id = target_frame_;

  // Recompute the executable attitude from the final mixed thrust vector.
  // PX4 consumes a scalar thrust plus attitude, so attitude must align with the
  // commanded thrust direction; otherwise x/y thrust components are not realized.
  tf2::Quaternion att_des;
  tf2::Vector3 thrust_exec;
  std::tie(att_des, thrust_exec) = pose_controller_->calculate_attitude_thrust(thrust_des);

  drone_cmd.attitude.x = att_des.x();
  drone_cmd.attitude.y = att_des.y();
  drone_cmd.attitude.z = att_des.z();
  drone_cmd.attitude.w = att_des.w();
  drone_cmd.thrust.x = thrust_exec.x();
  drone_cmd.thrust.y = thrust_exec.y();
  drone_cmd.thrust.z = thrust_exec.z();

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
        contact_frame_,
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
  } catch (const tf2::TransformException & ex) {
    (void)ex;
    return false;
  }
}

void WrenchControlNode::ft_data_callback(
  const geometry_msgs::msg::WrenchStamped::SharedPtr msg)
{
  if (!wrench_controller_) {
    return;
  }

  static bool logged_frame_id = false;
  if (!logged_frame_id) {
    RCLCPP_WARN(this->get_logger(),
      "ft_data frame_id='%s'  raw=(%.3f, %.3f, %.3f)",
      msg->header.frame_id.c_str(),
      msg->wrench.force.x, msg->wrench.force.y, msg->wrench.force.z);
    logged_frame_id = true;
  }

  if (msg->header.frame_id == "sim_ft_sensor") {
    RCLCPP_INFO_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      1000,
      "SIM FT  xyz=(%.4f, %.4f, %.4f)",
      msg->wrench.force.x, msg->wrench.force.y, msg->wrench.force.z);
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
}

void WrenchControlNode::tracking_point_callback(
  const nav_msgs::msg::Odometry::SharedPtr msg)
{
  if (wrench_controller_) {
    wrench_controller_->update_tracking_target(*msg, *tf_buffer_);
  }
  if (pose_controller_) {
    pose_controller_->update_target(*msg, *tf_buffer_);
  }
}

void WrenchControlNode::odometry_callback(
  const nav_msgs::msg::Odometry::SharedPtr msg)
{
  current_body_velocity_.setValue(
    msg->twist.twist.linear.x,
    msg->twist.twist.linear.y,
    msg->twist.twist.linear.z);
  if (wrench_controller_) {
    wrench_controller_->update_odom_state(*msg, *tf_buffer_);
  }
  if (pose_controller_) {
    pose_controller_->update_state(*msg, *tf_buffer_);
  }
}

void WrenchControlNode::switch_callback(
  const std_msgs::msg::Bool::SharedPtr msg)
{
  bool prev = mode_switch_;
  mode_switch_ = msg->data;
  if (!mode_switch_ && prev) {
    RCLCPP_INFO(this->get_logger(), "Switching to pose control mode.");
  } else if (mode_switch_ && !prev) {
    RCLCPP_INFO(
      this->get_logger(),
      "Switching to motion-force control mode (reset wrench integrators).");
    if (wrench_controller_) {
      wrench_controller_->reset();
    }
  }
}

void WrenchControlNode::vs_enable_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
  const bool prev = visual_servo_enabled_;
  visual_servo_enabled_ = msg->data;
  if (visual_servo_enabled_ != prev) {
    last_vs_enable_change_sec_ = this->now().seconds();
  }
  if (!visual_servo_enabled_) {
    reset_visual_servo_pi();
  }
  if (visual_servo_enabled_ != prev) {
    RCLCPP_INFO(
      this->get_logger(),
      "Visual servo %s.",
      visual_servo_enabled_ ? "enabled" : "disabled");
  }
}

void WrenchControlNode::vs_active_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
  visual_servo_active_ = msg->data;
  last_vs_active_stamp_sec_ = this->now().seconds();
  if (!visual_servo_active_) {
    reset_visual_servo_pi();
  }
}

void WrenchControlNode::vs_velocity_callback(
  const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
  latest_vs_cmd_camera_.setValue(
    msg->twist.linear.x,
    msg->twist.linear.y,
    msg->twist.linear.z);
  last_vs_cmd_stamp_sec_ = this->now().seconds();
}

void WrenchControlNode::vs_depth_callback(
  const geometry_msgs::msg::Vector3Stamped::SharedPtr msg)
{
  latest_vs_depth_ = msg->vector.z;
  last_vs_depth_stamp_sec_ = this->now().seconds();
}

void WrenchControlNode::reset_visual_servo_pi()
{
  vs_integral_error_.setZero();
  filtered_vs_body_velocity_.setValue(0.0, 0.0, 0.0);
}

bool WrenchControlNode::get_visual_servo_body_velocity(tf2::Vector3 & desired_body_velocity)
{
  try {
    geometry_msgs::msg::TransformStamped camera_to_robot_msg =
      tf_buffer_->lookupTransform(robot_frame_, camera_frame_, tf2::TimePointZero);
    tf2::Transform camera_to_robot_tf;
    tf2::fromMsg(camera_to_robot_msg.transform, camera_to_robot_tf);
    camera_to_robot_tf.setOrigin(tf2::Vector3(0.0, 0.0, 0.0));
    desired_body_velocity = camera_to_robot_tf * latest_vs_cmd_camera_;
    return true;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      3000,
      "Visual servo TF lookup failed: %s",
      ex.what());
    return false;
  }
}

bool WrenchControlNode::rotate_vector_between_frames(
  const tf2::Vector3 & input,
  const std::string & target_frame,
  const std::string & source_frame,
  tf2::Vector3 & output)
{
  try {
    geometry_msgs::msg::TransformStamped tf_msg =
      tf_buffer_->lookupTransform(target_frame, source_frame, tf2::TimePointZero);
    tf2::Transform tf;
    tf2::fromMsg(tf_msg.transform, tf);
    tf.setOrigin(tf2::Vector3(0.0, 0.0, 0.0));
    output = tf * input;
    return true;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      3000,
      "Vector TF lookup failed (%s <- %s): %s",
      target_frame.c_str(),
      source_frame.c_str(),
      ex.what());
    return false;
  }
}

double WrenchControlNode::compute_vs_force_lambda(double depth) const
{
  if (vs_depth_max_ <= vs_depth_min_) {
    return depth <= vs_depth_min_ ? 1.0 : 0.0;
  }
  if (depth >= vs_depth_max_) {
    return 0.0;
  }
  if (depth <= vs_depth_min_) {
    return 1.0;
  }

  const double alpha = (depth - vs_depth_min_) / (vs_depth_max_ - vs_depth_min_);
  return 0.5 * (1.0 + std::cos(kPi * alpha));
}

double WrenchControlNode::compute_vs_enable_ramp(double now_sec) const
{
  if (!visual_servo_enabled_) {
    return 0.0;
  }
  if (vs_enable_ramp_sec_ <= 1e-3) {
    return 1.0;
  }
  const double dt = now_sec - last_vs_enable_change_sec_;
  return std::clamp(dt / vs_enable_ramp_sec_, 0.0, 1.0);
}

}  // namespace wrench_controller
