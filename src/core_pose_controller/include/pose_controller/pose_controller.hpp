#ifndef ROS2_CONTROL_STACK_POSE_CONTROLLER_HPP_
#define ROS2_CONTROL_STACK_POSE_CONTROLLER_HPP_

#include <string>
#include <tuple>

#include <Eigen/Dense>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"

namespace tf2_ros
{
class Buffer;
}

namespace tf2
{
class Vector3;
class Quaternion;
}

namespace pose_controller
{

class GeoFence
{
public:
  GeoFence();

  GeoFence(
    bool enabled,
    double x_min, double x_max,
    double y_min, double y_max,
    double z_min, double z_max);

  bool check(const tf2::Vector3 & curr_pose, tf2::Vector3 & out_pose) const;

  bool enabled;
  double x_min;
  double x_max;
  double y_min;
  double y_max;
  double z_min;
  double z_max;
};

class PoseController
{
public:
  PoseController(
    const GeoFence & pos_fence,
    const std::string & target_frame,
    double xy_vel_lim,
    double thrust_min,
    double thrust_max,
    double max_tilt_deg,
    double hover_thrust);

  // Configure P/I/D/FF and limits from external vectors (ROS2 parameters).
  void configure_gains(
    const Eigen::Vector3d & P,
    const Eigen::Vector3d & I,
    const Eigen::Vector3d & D,
    const Eigen::Vector3d & FF,
    const Eigen::Vector3d & integral_threshold,
    const Eigen::Vector3d & minimum,
    const Eigen::Vector3d & maximum);

  void update_target(
    const nav_msgs::msg::Odometry & target,
    const tf2_ros::Buffer & tf_buffer);

  void update_state(
    const nav_msgs::msg::Odometry & odom,
    const tf2_ros::Buffer & tf_buffer);

  bool calculate_thrust(tf2::Vector3 & thrust_des);
  bool calculate_thrust(tf2::Vector3 & thrust_des, tf2::Quaternion & att_des);

  std::tuple<tf2::Quaternion, tf2::Vector3> calculate_attitude_thrust(
    const tf2::Vector3 & thrust_sp);

  tf2::Vector3 constrain_horizontal_thrust(const tf2::Vector3 & thr) const;
  void constrain_thrust(tf2::Vector3 & thrust_des) const;

  void reset();

private:
  // Controller gains and limits
  Eigen::Matrix3d P_;
  Eigen::Matrix3d I_;
  Eigen::Matrix3d D_;
  Eigen::Vector3d FF_;
  Eigen::Vector3d integral_;
  Eigen::Vector3d integral_threshold_;
  Eigen::Vector3d minimum_;
  Eigen::Vector3d maximum_;
  bool active_{false};

  // Timing
  std::chrono::steady_clock::time_point time_prev_;

  // Variables
  std::string target_frame_;
  double xy_vel_limit_;
  double hover_thrust_;
  double thrust_min_;
  double thrust_max_;
  double max_tilt_;
  double last_thrust_;
  bool track_velocities_{false};
  bool got_tracking_point_{false};
  bool got_odometry_{false};
  bool hover_thrust_filter_start_{false};

  // Latest tracking point and odometry readings in target frame
  nav_msgs::msg::Odometry odometry_;
  tf2::Vector3 tracking_point_pos_target_frame_;
  tf2::Vector3 tracking_point_vel_target_frame_;
  tf2::Vector3 tracking_point_thrust_target_frame_;
  tf2::Vector3 odometry_pos_target_frame_;
  tf2::Vector3 odometry_vel_target_frame_;
  double tracking_point_yaw_target_frame_{0.0};
  double odometry_yaw_target_frame_{0.0};
  tf2::Quaternion tracking_point_attitude_target_frame_;

  // Helper functions
  tf2::Vector3 constrain_xy_velocity(
    const tf2::Vector3 & v0,
    const tf2::Vector3 & v1) const;

  tf2::Vector3 constrain_velocity(
    const tf2::Vector3 & v0,
    const tf2::Vector3 & v1) const;

public:
  GeoFence position_fence;
};

}  // namespace pose_controller

#endif  // ROS2_CONTROL_STACK_POSE_CONTROLLER_HPP_

