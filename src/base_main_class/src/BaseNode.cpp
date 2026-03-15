#include "base/BaseNode.hpp"

namespace base
{

BaseNode::BaseNode(
  const std::string & node_name,
  const rclcpp::NodeOptions & options)
: rclcpp::Node(node_name, options)
, Base()
{
  // Declare the execute_target parameter with a default of 0.0 (disabled)
  this->declare_parameter<double>("execute_target", 0.0);
}

bool BaseNode::start_execute_timer()
{
  double execute_target{0.0};
  this->get_parameter("execute_target", execute_target);

  if (execute_target <= 0.0) {
    RCLCPP_FATAL(
      this->get_logger(),
      "The 'execute_target' parameter must be set to > 0.0 (Hz).");
    return false;
  }

  const auto period =
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / execute_target));

  execute_timer_ = this->create_wall_timer(
    period,
    std::bind(&BaseNode::execute_timer_callback, this));

  return true;
}

void BaseNode::execute_timer_callback()
{
  if (!this->_execute()) {
    if (execute_timer_) {
      execute_timer_->cancel();
    }

    RCLCPP_FATAL(this->get_logger(), "BaseNode::_execute() returned false. Stopping timer.");
  }
}

}  // namespace base

