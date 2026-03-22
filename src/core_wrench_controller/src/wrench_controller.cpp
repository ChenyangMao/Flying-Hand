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

    // Median filter per axis
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

    // Mean filter over last N samples (bounded by mean_filter_max_buffer_size_)
    {
      int count = 0;
      tf2::Vector3 acc(0.0, 0.0, 0.0);
      for (auto it = force_samples_.rbegin();
           it != force_samples_.rend() && count < mean_filter_max_buffer_size_;
           ++it, ++count) {
        acc += *it;
      }
      if (count > 0) {
        force_mean_filtered_ = acc * (1.0 / static_cast<double>(count));
      } else {
        force_mean_filtered_ = force_median_filtered_;
      }
    }

    // Decide which filtered signal to use as measured force
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
  const tf2_ros::Buffer &)
{
  odometry_vel_world_frame_ = tf2::Vector3(
    odom.twist.twist.linear.x,
    odom.twist.twist.linear.y,
    odom.twist.twist.linear.z);
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

    tf2::Vector3 thrust_ff_sensor_frame(0, 0, 0.1);
    thrust_ff_sensor_frame.setZ(
      force_ff_coefficient * target_force_sensor_frame_.z() + force_ff_coefficient_bias);

    thrust_vel_damping_ =
      -velx_damping_coefficient * odometry_vel_world_frame_.z();
    if (thrust_vel_damping_ > 0.05) {
      thrust_vel_damping_ = 0.05;
    } else if (thrust_vel_damping_ < -0.05) {
      thrust_vel_damping_ = -0.05;
    }

    tf2::Vector3 last_thrust_des =
      delta_thrust_des_sensor_frame +
      thrust_ff_sensor_frame +
      tf2::Vector3(0, 0, thrust_vel_damping_);

    if (last_thrust_des.z() < 0.005) {
      last_thrust_des.setZ(0.005);
    }
    if (last_thrust_des.z() > 0.5) {
      last_thrust_des.setZ(0.5);
    }

    thrust_des = sensor_to_thrust_output_tf * last_thrust_des;

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

bool WrenchController::check_in_contact(double thresh_force_z) const
{
  return target_force_sensor_frame_.z() > thresh_force_z;
}

void WrenchController::reset()
{
  got_target_wrench_ = false;
  got_ft_data_ = false;
}

void WrenchController::configure_fx(
  double p,
  double i,
  double d,
  double integral_threshold,
  double minimum,
  double maximum)
{
  fx_controller_.set_P(p);
  fx_controller_.set_I(i);
  fx_controller_.set_D(d);
  fx_controller_.set_integral_threshold(integral_threshold);
  fx_controller_.set_minimum(minimum);
  fx_controller_.set_maximum(maximum);
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

}  // namespace wrench_controller

