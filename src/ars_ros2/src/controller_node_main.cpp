#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "ars_ros2/controller_node.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  const auto node = std::make_shared<ars::ros2::ControllerNode>();
  node->configure();
  node->activate();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
