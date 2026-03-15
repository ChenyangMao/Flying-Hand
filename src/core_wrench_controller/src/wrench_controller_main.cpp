#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "wrench_controller/wrench_controller_node.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<wrench_controller::WrenchControlNode>("wrench_controller");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
