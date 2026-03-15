#ifndef ROS2_CONTROL_STACK_BASE_BASENODE_HPP_
#define ROS2_CONTROL_STACK_BASE_BASENODE_HPP_

#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "base/Base.hpp"

namespace base
{

/**
 * BaseNode: ROS2 node that wraps the Base logic.
 *
 * Derived classes typically inherit from this class and implement
 *   - bool initialize()
 *   - bool execute()
 */
class BaseNode : public rclcpp::Node, public Base
{
public:
  explicit BaseNode(
    const std::string & node_name,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  virtual ~BaseNode() = default;

protected:
  /// Helper to start the periodic execute timer based on the 'execute_target' parameter (Hz).
  bool start_execute_timer();

private:
  void execute_timer_callback();

  rclcpp::TimerBase::SharedPtr execute_timer_;
};

}  // namespace base

#endif  // ROS2_CONTROL_STACK_BASE_BASENODE_HPP_

