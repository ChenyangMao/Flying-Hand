#include "wrench_controller/wrench_controller.hpp"

#include <algorithm>
#include <chrono>
#include <deque>
#include <iostream>

#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>

namespace wrench_controller
{

WrenchController::WrenchController(
  const std::string & sensor_frame,
  const std::string & robot_frame,
  const std::string & world_frame,
  const std::string & camera_frame,
  const std::string & thrust_output_frame,
  const std::string & torque_output_frame,
  double hover_thrust,
  bool filter_ft_data,
  int median_buffer_size,
  int mean_buffer_size)
: fx_controller_("fx"),
  fy_controller_("fy"),
  fz_controller_("fz"),
  tx_controller_("tx"),
  ty_controller_("ty"),
  tz_controller_("tz"),
  sensor_frame_(sensor_frame),
  robot_frame_(robot_frame),
  world_frame_(world_frame),
  camera_frame_(camera_frame),
  thrust_output_frame_(thrust_output_frame),
  torque_output_frame_(torque_output_frame),
  hover_thrust_(hover_thrust),
  filter_ft_data_(filter_ft_data),
  median_filter_max_buffer_size_(median_buffer_size),
  mean_filter_max_buffer_size_(mean_buffer_size)
{
}

void WrenchController::update_target(
  const geometry_msgs::msg::WrenchStamped & target,
  const tf2_ros::Buffer & tf_buffer)
{
  geometry_msgs::msg::TransformStamped target_to_sensor_tf;
  try {
    target_wrench_ = target;

    target_to_sensor_tf = tf_buffer.lookupTransform(
      sensor_frame_, target.header.frame_id, tf2::TimePointZero);

    tf2::Vector3 target_force(
      target.wrench.force.x,
      target.wrench.force.y,
      target.wrench.force.z);
    tf2::Vector3 target_torque(
      target.wrench.torque.x,
      target.wrench.torque.y,
      target.wrench.torque.z);

    tf2::Transform tf;
    tf2::fromMsg(target_to_sensor_tf.transform, tf);

    target_force_sensor_frame_ = tf * target_force;
    target_torque_sensor_frame_ = tf * target_torque;

    got_target_wrench_ = true;
  } catch (const tf2::TransformException &) {
    // In ROS2 we typically log from the calling node; keep this helper silent.
  }
}

geometry_msgs::msg::WrenchStamped WrenchController::update_state(
  const geometry_msgs::msg::WrenchStamped & ft_data,
  const tf2_ros::Buffer & tf_buffer)
{
  geometry_msgs::msg::TransformStamped ftdata_to_sensor_tf;
  try {
    current_wrench_ = ft_data;

    ftdata_to_sensor_tf = tf_buffer.lookupTransform(
      sensor_frame_, ft_data.header.frame_id, tf2::TimePointZero);

    tf2::Transform tf;
    tf2::fromMsg(ftdata_to_sensor_tf.transform, tf);

    tf2::Vector3 meas_force(
      ft_data.wrench.force.x,
      ft_data.wrench.force.y,
      ft_data.wrench.force.z);
    tf2::Vector3 meas_torque(
      ft_data.wrench.torque.x,
      ft_data.wrench.torque.y,
      ft_data.wrench.torque.z);

    tf2::Vector3 force_in_sensor = tf * meas_force;
    tf2::Vector3 torque_in_sensor = tf * meas_torque;

    // Update force sample buffer for filtering
    force_samples_.push_back(force_in_sensor);
    if (static_cast<int>(force_samples_.size()) > median_filter_max_buffer_size_) {
      force_samples_.pop_front();
    }

    // Stage 1: Median filter per axis (outlier / spike rejection)
    {
      std::vector<double> sx;
      std::vector<double> sy;
      std::vector<double> sz;
      sx.reserve(force_samples_.size());
      sy.reserve(force_samples_.size());
      sz.reserve(force_samples_.size());
      for (const auto & v : force_samples_) {
        sx.push_back(v.x());
        sy.push_back(v.y());
        sz.push_back(v.z());
      }
      auto median = [](std::vector<double> & v) -> double {
        if (v.empty()) {
          return 0.0;
        }
        std::nth_element(v.begin(), v.begin() + v.size() / 2, v.end());
        return v[v.size() / 2];
      };
      force_median_filtered_.setX(median(sx));
      force_median_filtered_.setY(median(sy));
      force_median_filtered_.setZ(median(sz));
    }

    // Stage 2: Mean filter over *median outputs* (smoothing after spike removal)
    median_samples_.push_back(force_median_filtered_);
    if (static_cast<int>(median_samples_.size()) > mean_filter_max_buffer_size_) {
      median_samples_.pop_front();
    }
    {
      tf2::Vector3 acc(0.0, 0.0, 0.0);
      for (const auto & v : median_samples_) {
        acc += v;
      }
      const int count = static_cast<int>(median_samples_.size());
      if (count > 0) {
        force_mean_filtered_ = acc * (1.0 / static_cast<double>(count));
      } else {
        force_mean_filtered_ = force_median_filtered_;
      }
    }

    // Use cascaded median→mean output as the measured force for control
    meas_force_sensor_frame_ =
      filter_ft_data_ ? force_mean_filtered_ : force_in_sensor;
    meas_torque_sensor_frame_ = torque_in_sensor;

    got_ft_data_ = true;

    geometry_msgs::msg::WrenchStamped out_msg(ft_data);
    out_msg.header.frame_id = sensor_frame_;
    out_msg.wrench.force.x = meas_force_sensor_frame_.x();
    out_msg.wrench.force.y = meas_force_sensor_frame_.y();
    out_msg.wrench.force.z = meas_force_sensor_frame_.z();
    out_msg.wrench.torque.x = meas_torque_sensor_frame_.x();
    out_msg.wrench.torque.y = meas_torque_sensor_frame_.y();
    out_msg.wrench.torque.z = meas_torque_sensor_frame_.z();

    return out_msg;
  } catch (const tf2::TransformException &) {
    return ft_data;
  }
}

void WrenchController::update_odom_state(
  const nav_msgs::msg::Odometry & odom,
  const tf2_ros::Buffer & tf_buffer)
{
  try {
    geometry_msgs::msg::TransformStamped odom_pos_to_world_tf =
      tf_buffer.lookupTransform(world_frame_, odom.header.frame_id, tf2::TimePointZero);
    geometry_msgs::msg::TransformStamped odom_vel_to_world_tf =
      tf_buffer.lookupTransform(world_frame_, odom.child_frame_id, tf2::TimePointZero);

    tf2::Transform pos_tf;
    tf2::fromMsg(odom_pos_to_world_tf.transform, pos_tf);

    tf2::Transform vel_tf;
    tf2::fromMsg(odom_vel_to_world_tf.transform, vel_tf);
    vel_tf.setOrigin(tf2::Vector3(0, 0, 0));

    odometry_pos_world_frame_ = pos_tf * tf2::Vector3(
      odom.pose.pose.position.x,
      odom.pose.pose.position.y,
      odom.pose.pose.position.z);

    odometry_vel_world_frame_ = vel_tf * tf2::Vector3(
      odom.twist.twist.linear.x,
      odom.twist.twist.linear.y,
      odom.twist.twist.linear.z);
  } catch (const tf2::TransformException &) {
    odometry_pos_world_frame_ = tf2::Vector3(
      odom.pose.pose.position.x,
      odom.pose.pose.position.y,
      odom.pose.pose.position.z);
    odometry_vel_world_frame_ = tf2::Vector3(
      odom.twist.twist.linear.x,
      odom.twist.twist.linear.y,
      odom.twist.twist.linear.z);
  }
}

void WrenchController::update_tracking_target(
  const nav_msgs::msg::Odometry & target,
  const tf2_ros::Buffer & tf_buffer)
{
  try {
    geometry_msgs::msg::TransformStamped target_pos_to_world_tf =
      tf_buffer.lookupTransform(world_frame_, target.header.frame_id, tf2::TimePointZero);
    geometry_msgs::msg::TransformStamped target_vel_to_world_tf =
      tf_buffer.lookupTransform(world_frame_, target.child_frame_id, tf2::TimePointZero);

    tf2::Transform pos_tf;
    tf2::fromMsg(target_pos_to_world_tf.transform, pos_tf);

    tf2::Transform vel_tf;
    tf2::fromMsg(target_vel_to_world_tf.transform, vel_tf);
    vel_tf.setOrigin(tf2::Vector3(0, 0, 0));

    tracking_target_pos_world_frame_ = pos_tf * tf2::Vector3(
      target.pose.pose.position.x,
      target.pose.pose.position.y,
      target.pose.pose.position.z);

    tracking_target_vel_world_frame_ = vel_tf * tf2::Vector3(
      target.twist.twist.linear.x,
      target.twist.twist.linear.y,
      target.twist.twist.linear.z);
  } catch (const tf2::TransformException &) {
    tracking_target_pos_world_frame_ = tf2::Vector3(
      target.pose.pose.position.x,
      target.pose.pose.position.y,
      target.pose.pose.position.z);
    tracking_target_vel_world_frame_ = tf2::Vector3(
      target.twist.twist.linear.x,
      target.twist.twist.linear.y,
      target.twist.twist.linear.z);
  }
}

bool WrenchController::calculate_thrust_torque(
  tf2::Vector3 & thrust_des,
  tf2::Vector3 & torque_des,
  const tf2_ros::Buffer & tf_buffer,
  double force_ff_coefficient,
  double force_ff_coefficient_bias,
  const tf2::Vector3 & wrench_controller_ff_force,
  double velx_damping_coefficient)
{
  if (!got_ft_data_ || !got_target_wrench_) {
    static auto last_log_time = std::chrono::steady_clock::time_point{};
    const auto now = std::chrono::steady_clock::now();
    if (
      last_log_time.time_since_epoch().count() == 0 ||
      std::chrono::duration_cast<std::chrono::milliseconds>(now - last_log_time).count() > 1000)
    {
      std::cerr
        << "[wrench_controller] calculate_thrust_torque blocked:"
        << " got_ft_data=" << (got_ft_data_ ? "true" : "false")
        << " got_target_wrench=" << (got_target_wrench_ ? "true" : "false")
        << std::endl;
      last_log_time = now;
    }
    return false;
  }

  try {
    geometry_msgs::msg::TransformStamped sensor_to_thrust_output_tf_msg =
      tf_buffer.lookupTransform(
        thrust_output_frame_, sensor_frame_, tf2::TimePointZero);
    geometry_msgs::msg::TransformStamped sensor_to_torque_output_tf_msg =
      tf_buffer.lookupTransform(
        torque_output_frame_, sensor_frame_, tf2::TimePointZero);

    tf2::Transform sensor_to_thrust_output_tf;
    tf2::fromMsg(sensor_to_thrust_output_tf_msg.transform, sensor_to_thrust_output_tf);
    sensor_to_thrust_output_tf.setOrigin(tf2::Vector3(0, 0, 0));

    tf2::Transform sensor_to_torque_output_tf;
    tf2::fromMsg(sensor_to_torque_output_tf_msg.transform, sensor_to_torque_output_tf);
    sensor_to_torque_output_tf.setOrigin(tf2::Vector3(0, 0, 0));

    // Set PID targets
    fx_controller_.set_target(target_force_sensor_frame_.x());
    fy_controller_.set_target(target_force_sensor_frame_.y());
    fz_controller_.set_target(target_force_sensor_frame_.z());
    tx_controller_.set_target(target_torque_sensor_frame_.x());
    ty_controller_.set_target(target_torque_sensor_frame_.y());
    tz_controller_.set_target(target_torque_sensor_frame_.z());

    tf2::Vector3 delta_thrust_des_sensor_frame(
      fx_controller_.get_control(meas_force_sensor_frame_.x(), 0.0),
      fy_controller_.get_control(meas_force_sensor_frame_.y(), 0.0),
      fz_controller_.get_control(meas_force_sensor_frame_.z(), 0.0));

    // Feedforward on the contact-normal axis (sensor X for wall missions).
    tf2::Vector3 thrust_ff_sensor_frame(
      force_ff_coefficient * target_force_sensor_frame_.x() + force_ff_coefficient_bias,
      0.0, 0.0);

    // Same as control_stack_base ROS1: damp using world-frame vertical velocity,
    // injected on the sensor thrust vector's Z component before rotation to output frame.
    thrust_vel_damping_ =
      -velx_damping_coefficient * odometry_vel_world_frame_.z();
    if (thrust_vel_damping_ > 0.05) {
      thrust_vel_damping_ = 0.05;
    }
    if (thrust_vel_damping_ < -0.05) {
      thrust_vel_damping_ = -0.05;
    }

    tf2::Vector3 last_thrust_des =
      delta_thrust_des_sensor_frame +
      thrust_ff_sensor_frame +
      tf2::Vector3(0.0, 0.0, thrust_vel_damping_);

    if (last_thrust_des.z() < 0.005) {
      last_thrust_des.setZ(0.005);
    }
    if (last_thrust_des.z() > 0.5) {
      last_thrust_des.setZ(0.5);
    }

    thrust_des = sensor_to_thrust_output_tf * last_thrust_des;

    const double x_pos_error =
      tracking_target_pos_world_frame_.x() - odometry_pos_world_frame_.x();
    const double x_vel_error =
      tracking_target_vel_world_frame_.x() - odometry_vel_world_frame_.x();
    thrust_des.setX(thrust_des.x() + x_hold_p_ * x_pos_error + x_hold_d_ * x_vel_error);

    thrust_des[1] += wrench_controller_ff_force[1];
    thrust_des[2] += wrench_controller_ff_force[2];

    tf2::Vector3 delta_torque_des = sensor_to_torque_output_tf * tf2::Vector3(
      tx_controller_.get_control(meas_torque_sensor_frame_.x(), 0.0),
      ty_controller_.get_control(meas_torque_sensor_frame_.y(), 0.0),
      tz_controller_.get_control(meas_torque_sensor_frame_.z(), 0.0));

    static tf2::Vector3 last_torque_des(0, 0, 0);
    torque_des = last_torque_des + delta_torque_des;
    last_torque_des = torque_des;

    return true;
  } catch (const tf2::TransformException & ex) {
    static auto last_tf_log_time = std::chrono::steady_clock::time_point{};
    const auto now = std::chrono::steady_clock::now();
    if (
      last_tf_log_time.time_since_epoch().count() == 0 ||
      std::chrono::duration_cast<std::chrono::milliseconds>(now - last_tf_log_time).count() > 1000)
    {
      std::cerr
        << "[wrench_controller] calculate_thrust_torque TF lookup failed: "
        << ex.what()
        << " (thrust_output_frame=" << thrust_output_frame_
        << ", torque_output_frame=" << torque_output_frame_
        << ", sensor_frame=" << sensor_frame_ << ")"
        << std::endl;
      last_tf_log_time = now;
    }
    return false;
  }
}

void WrenchController::reset()
{
  // Match control_stack_base ROS1 WrenchController::Reset(): integrators only.
  fx_controller_.reset_integral();
  fy_controller_.reset_integral();
  fz_controller_.reset_integral();
  tx_controller_.reset_integral();
  ty_controller_.reset_integral();
  tz_controller_.reset_integral();
}

void WrenchController::configure_fx(
  double p,
  double i,
  double d,
  double integral_threshold,
  double minimum,
  double maximum,
  double ff,
  double constant)
{
  fx_controller_.set_P(p);
  fx_controller_.set_I(i);
  fx_controller_.set_D(d);
  fx_controller_.set_FF(ff);
  fx_controller_.set_constant(constant);
  fx_controller_.set_integral_threshold(integral_threshold);
  fx_controller_.set_minimum(minimum);
  fx_controller_.set_maximum(maximum);
}

void WrenchController::configure_fx_negative_gains(
  bool use_negative,
  double neg_p,
  double neg_i,
  double neg_d,
  double neg_ff)
{
  fx_controller_.set_negative_gains(neg_p, neg_i, neg_d, neg_ff);
  fx_controller_.set_use_negative_gains(use_negative);
}

void WrenchController::configure_fy(double p, double d, double minimum, double maximum)
{
  fy_controller_.set_P(p);
  fy_controller_.set_D(d);
  fy_controller_.set_minimum(minimum);
  fy_controller_.set_maximum(maximum);
}

void WrenchController::configure_fz(
  double p,
  double i,
  double d,
  double integral_threshold,
  double minimum,
  double maximum)
{
  fz_controller_.set_P(p);
  fz_controller_.set_I(i);
  fz_controller_.set_D(d);
  fz_controller_.set_integral_threshold(integral_threshold);
  fz_controller_.set_minimum(minimum);
  fz_controller_.set_maximum(maximum);
}

void WrenchController::configure_x_hold_pd(double p, double d)
{
  x_hold_p_ = p;
  x_hold_d_ = d;
}

}  // namespace wrench_controller
