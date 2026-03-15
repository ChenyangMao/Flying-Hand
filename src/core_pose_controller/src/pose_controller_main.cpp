#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "pose_controller/pose_controller_node.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<pose_controller::PoseControlNode>("pose_controller");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
