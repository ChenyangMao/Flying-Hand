#include "pose_controller/pose_controller.hpp"

#include <cmath>
#include <limits>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>

namespace pose_controller
{

GeoFence::GeoFence()
: enabled(false),
  x_min(std::numeric_limits<double>::lowest()),
  x_max(std::numeric_limits<double>::max()),
  y_min(std::numeric_limits<double>::lowest()),
  y_max(std::numeric_limits<double>::max()),
  z_min(std::numeric_limits<double>::lowest()),
  z_max(std::numeric_limits<double>::max())
{
}

GeoFence::GeoFence(
  bool enabled_in,
  double x_min_in, double x_max_in,
  double y_min_in, double y_max_in,
  double z_min_in, double z_max_in)
: enabled(enabled_in),
  x_min(x_min_in),
  x_max(x_max_in),
  y_min(y_min_in),
  y_max(y_max_in),
  z_min(z_min_in),
  z_max(z_max_in)
{
}

bool GeoFence::check(const tf2::Vector3 & curr_pose, tf2::Vector3 & out_pose) const
{
  out_pose = curr_pose;
  if (!enabled) {
    return true;
  }

  bool modified = false;

  if (out_pose.x() > x_max) {
    out_pose.setX(x_max);
    modified = true;
  } else if (out_pose.x() < x_min) {
    out_pose.setX(x_min);
    modified = true;
  }

  if (out_pose.y() > y_max) {
    out_pose.setY(y_max);
    modified = true;
  } else if (out_pose.y() < y_min) {
    out_pose.setY(y_min);
    modified = true;
  }

  if (out_pose.z() > z_max) {
    out_pose.setZ(z_max);
    modified = true;
  } else if (out_pose.z() < z_min) {
    out_pose.setZ(z_min);
    modified = true;
  }

  return !modified;
}

PoseController::PoseController(
  const GeoFence & pos_fence,
  const std::string & target_frame,
  double xy_vel_lim,
  double thrust_min,
  double thrust_max,
  double max_tilt_deg,
  double hover_thrust)
: P_(Eigen::Matrix3d::Zero()),
  I_(Eigen::Matrix3d::Zero()),
  D_(Eigen::Matrix3d::Zero()),
  FF_(Eigen::Vector3d::Zero()),
  integral_(Eigen::Vector3d::Zero()),
  integral_threshold_(Eigen::Vector3d::Constant(4.0)),
  minimum_(Eigen::Vector3d::Constant(std::numeric_limits<double>::lowest())),
  maximum_(Eigen::Vector3d::Constant(std::numeric_limits<double>::max())),
  target_frame_(target_frame),
  xy_vel_limit_(xy_vel_lim),
  hover_thrust_(hover_thrust),
  thrust_min_(thrust_min),
  thrust_max_(thrust_max),
  max_tilt_(max_tilt_deg * M_PI / 180.0),
  last_thrust_(hover_thrust),
  position_fence(pos_fence)
{
  time_prev_ = std::chrono::steady_clock::time_point{};
}

void PoseController::configure_gains(
  const Eigen::Vector3d & P,
  const Eigen::Vector3d & I,
  const Eigen::Vector3d & D,
  const Eigen::Vector3d & FF,
  const Eigen::Vector3d & integral_threshold,
  const Eigen::Vector3d & minimum,
  const Eigen::Vector3d & maximum)
{
  P_ = P.asDiagonal();
  I_ = I.asDiagonal();
  D_ = D.asDiagonal();
  FF_ = FF;
  integral_threshold_ = integral_threshold;
  minimum_ = minimum;
  maximum_ = maximum;
}

void PoseController::update_target(
  const nav_msgs::msg::Odometry & target,
  const tf2_ros::Buffer & tf_buffer)
{
  try {
    geometry_msgs::msg::TransformStamped tracking_point_pos_to_target_tf =
      tf_buffer.lookupTransform(
        target_frame_, target.header.frame_id,
        tf2::TimePointZero);

    geometry_msgs::msg::TransformStamped tracking_point_vel_to_target_tf =
      tf_buffer.lookupTransform(
        target_frame_, target.header.frame_id,
        tf2::TimePointZero);

    tf2::Transform pos_tf;
    tf2::fromMsg(tracking_point_pos_to_target_tf.transform, pos_tf);

    tf2::Transform vel_tf;
    tf2::fromMsg(tracking_point_vel_to_target_tf.transform, vel_tf);
    vel_tf.setOrigin(tf2::Vector3(0, 0, 0));

    tracking_point_pos_target_frame_ = pos_tf * tf2::Vector3(
      target.pose.pose.position.x,
      target.pose.pose.position.y,
      target.pose.pose.position.z);

    tf2::Vector3 modified_tracking_pos;
    if (!position_fence.check(tracking_point_pos_target_frame_, modified_tracking_pos)) {
      tracking_point_pos_target_frame_ = modified_tracking_pos;
    }

    tracking_point_vel_target_frame_ = vel_tf * tf2::Vector3(
      target.twist.twist.linear.x,
      target.twist.twist.linear.y,
      target.twist.twist.linear.z);

    tracking_point_thrust_target_frame_ = tf2::Vector3(0.0, 0.0, 0.0);

    tf2::Quaternion q_target(
      target.pose.pose.orientation.x,
      target.pose.pose.orientation.y,
      target.pose.pose.orientation.z,
      target.pose.pose.orientation.w);
    tf2::Quaternion q_in_target = pos_tf * q_target;

    // Attitude in target frame (same as ROS1)
    tracking_point_attitude_target_frame_ = q_in_target;

    // Yaw in target frame
    double roll, pitch, yaw;
    tf2::Matrix3x3(q_in_target).getRPY(roll, pitch, yaw);
    tracking_point_yaw_target_frame_ = yaw;

    got_tracking_point_ = true;
  } catch (const tf2::TransformException &) {
    // leave got_tracking_point_ unchanged
  }
}

void PoseController::update_state(
  const nav_msgs::msg::Odometry & odom,
  const tf2_ros::Buffer & tf_buffer)
{
  try {
    odometry_ = odom;

    geometry_msgs::msg::TransformStamped odom_pos_to_target_tf =
      tf_buffer.lookupTransform(
        target_frame_, odom.header.frame_id,
        tf2::TimePointZero);

    geometry_msgs::msg::TransformStamped odom_vel_to_target_tf =
      tf_buffer.lookupTransform(
        target_frame_, odom.child_frame_id,
        tf2::TimePointZero);

    tf2::Transform pos_tf;
    tf2::fromMsg(odom_pos_to_target_tf.transform, pos_tf);

    tf2::Transform vel_tf;
    tf2::fromMsg(odom_vel_to_target_tf.transform, vel_tf);
    vel_tf.setOrigin(tf2::Vector3(0, 0, 0));

    odometry_pos_target_frame_ = pos_tf * tf2::Vector3(
      odom.pose.pose.position.x,
      odom.pose.pose.position.y,
      odom.pose.pose.position.z);

    odometry_vel_target_frame_ = vel_tf * tf2::Vector3(
      odom.twist.twist.linear.x,
      odom.twist.twist.linear.y,
      odom.twist.twist.linear.z);

    tf2::Quaternion q_odom(
      odom.pose.pose.orientation.x,
      odom.pose.pose.orientation.y,
      odom.pose.pose.orientation.z,
      odom.pose.pose.orientation.w);
    tf2::Quaternion q_in_target = pos_tf * q_odom;

    double roll, pitch, yaw;
    tf2::Matrix3x3(q_in_target).getRPY(roll, pitch, yaw);
    odometry_yaw_target_frame_ = yaw;

    got_odometry_ = true;
  } catch (const tf2::TransformException &) {
    // leave got_odometry_ unchanged
  }
}

bool PoseController::calculate_thrust(tf2::Vector3 & thrust_des)
{
  tf2::Quaternion dummy_att;
  return calculate_thrust(thrust_des, dummy_att);
}

bool PoseController::calculate_thrust(
  tf2::Vector3 & thrust_des,
  tf2::Quaternion & att_des)
{
  if (!got_tracking_point_ || !got_odometry_) {
    return false;
  }

  const auto time_now = std::chrono::steady_clock::now();
  if (time_prev_.time_since_epoch().count() == 0) {
    time_prev_ = time_now;
    return false;
  }

  const double dt =
    std::chrono::duration_cast<std::chrono::duration<double>>(time_now - time_prev_).count();
  if (dt <= 0.0) {
    return false;
  }

  // Position and velocity errors in target frame
  tf2::Vector3 pos_error =
    tracking_point_pos_target_frame_ - odometry_pos_target_frame_;
  tf2::Vector3 vel_error =
    tracking_point_vel_target_frame_ - odometry_vel_target_frame_;

  Eigen::Vector3d pos_e(pos_error.x(), pos_error.y(), pos_error.z());
  Eigen::Vector3d vel_e(vel_error.x(), vel_error.y(), vel_error.z());
  Eigen::Vector3d thrust_target(
    tracking_point_thrust_target_frame_.x(),
    tracking_point_thrust_target_frame_.y(),
    tracking_point_thrust_target_frame_.z());

  integral_ += pos_e * dt;
  // clamp integral
  for (int i = 0; i < 3; ++i) {
    if (integral_[i] > integral_threshold_[i]) {
      integral_[i] = integral_threshold_[i];
    } else if (integral_[i] < -integral_threshold_[i]) {
      integral_[i] = -integral_threshold_[i];
    }
  }

  // Same formula as ROS1: P*pos_err + D*vel_err + I*integral + FF + thrust_target
  Eigen::Vector3d thrust_cmd =
    P_ * pos_e + I_ * integral_ + D_ * vel_e + FF_ + thrust_target;

  // Apply min/max limits
  for (int i = 0; i < 3; ++i) {
    if (thrust_cmd[i] < minimum_[i]) {
      thrust_cmd[i] = minimum_[i];
    } else if (thrust_cmd[i] > maximum_[i]) {
      thrust_cmd[i] = maximum_[i];
    }
  }

  // Same as ROS1: add hover_thrust to z component
  thrust_des = tf2::Vector3(
    thrust_cmd[0],
    thrust_cmd[1],
    thrust_cmd[2] + hover_thrust_);

  // Constrain thrust vector based on tilt and thrust magnitude
  constrain_thrust(thrust_des);

  // Same as ROS1: use tracking point attitude as desired attitude
  att_des = tracking_point_attitude_target_frame_;
  // Fallback to identity if quaternion is invalid (e.g. before first target)
  if (att_des.x() == 0.0 && att_des.y() == 0.0 &&
      att_des.z() == 0.0 && att_des.w() == 0.0) {
    att_des.setValue(0.0, 0.0, 0.0, 1.0);
  }

  time_prev_ = time_now;
  active_ = true;
  return true;
}

std::tuple<tf2::Quaternion, tf2::Vector3> PoseController::calculate_attitude_thrust(
  const tf2::Vector3 & thrust_sp)
{
  tf2::Vector3 thrust = thrust_sp;
  constrain_thrust(thrust);

  // Simple mapping: thrust direction -> roll/pitch, yaw from tracking point.
  double total_thrust_mag = thrust.length();
  tf2::Quaternion att(0, 0, 0, 1);

  if (total_thrust_mag > 1e-6) {
    double roll = 0.0;
    double pitch = 0.0;
    // For small angles, map lateral thrust to roll/pitch
    pitch = std::asin(std::max(std::min(thrust.x() / total_thrust_mag, 1.0), -1.0));
    roll = -std::asin(std::max(std::min(thrust.y() / total_thrust_mag, 1.0), -1.0));
    double yaw = tracking_point_yaw_target_frame_;
    att.setRPY(roll, pitch, yaw);
  }

  return std::make_tuple(att, thrust);
}

tf2::Vector3 PoseController::constrain_horizontal_thrust(
  const tf2::Vector3 & thr) const
{
  tf2::Vector3 limited = thr;

  const double horiz_norm = std::sqrt(
    limited.x() * limited.x() + limited.y() * limited.y());
  if (horiz_norm > std::tan(max_tilt_) * limited.z()) {
    const double scale = std::tan(max_tilt_) * limited.z() / horiz_norm;
    limited.setX(limited.x() * scale);
    limited.setY(limited.y() * scale);
  }

  return limited;
}

void PoseController::constrain_thrust(tf2::Vector3 & thrust_des) const
{
  thrust_des = constrain_horizontal_thrust(thrust_des);

  // Limit thrust magnitude between thrust_min_ and thrust_max_
  double mag = thrust_des.length();
  if (mag < thrust_min_) {
    if (mag > 1e-6) {
      thrust_des *= (thrust_min_ / mag);
    } else {
      thrust_des.setZ(thrust_min_);
    }
  } else if (mag > thrust_max_) {
    thrust_des *= (thrust_max_ / mag);
  }
}

tf2::Vector3 PoseController::constrain_xy_velocity(
  const tf2::Vector3 & v0,
  const tf2::Vector3 & v1) const
{
  tf2::Vector3 v = v1 - v0;
  double v_xy = std::sqrt(v.x() * v.x() + v.y() * v.y());
  if (v_xy > xy_vel_limit_) {
    const double scale = xy_vel_limit_ / v_xy;
    return tf2::Vector3(
      v0.x() + v.x() * scale,
      v0.y() + v.y() * scale,
      v1.z());
  }
  return v1;
}

tf2::Vector3 PoseController::constrain_velocity(
  const tf2::Vector3 & v0,
  const tf2::Vector3 & v1) const
{
  return constrain_xy_velocity(v0, v1);
}

void PoseController::reset()
{
  integral_.setZero();
  got_tracking_point_ = false;
  got_odometry_ = false;
  active_ = false;
  time_prev_ = std::chrono::steady_clock::time_point{};
}

}  // namespace pose_controller

