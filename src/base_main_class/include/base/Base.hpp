/**
 * ROS2 version of the Base class from control_stack_base/base_main_class.
 *
 * This class keeps the same conceptual API:
 *   - bool _initialize()
 *   - bool _execute()
 *   - virtual bool initialize() = 0;
 *   - virtual bool execute() = 0;
 *   - void fail(const std::string & reason);
 *
 * but is implemented in a way that is friendly to rclcpp-based nodes.
 */

#ifndef ROS2_CONTROL_STACK_BASE_BASE_HPP_
#define ROS2_CONTROL_STACK_BASE_BASE_HPP_

#include <string>

namespace base
{

class Base
{
public:
  virtual ~Base() = default;

  /// Called by the ROS2 node once, after construction.
  bool _initialize();

  /// Called by the ROS2 node periodically.
  bool _execute();

  /// Implemented by the derived class: setup publishers, subscribers, parameters, etc.
  virtual bool initialize() = 0;

  /// Implemented by the derived class: main loop body.
  virtual bool execute() = 0;

  /// Mark the node as failed; subsequent _execute() calls will return false.
  void fail(const std::string & reason);

  /// Check if the base has entered a failed state.
  bool failed() const { return failed_; }

protected:
  Base() = default;

private:
  bool failed_ {false};
};

}  // namespace base

#endif  // ROS2_CONTROL_STACK_BASE_BASE_HPP_

