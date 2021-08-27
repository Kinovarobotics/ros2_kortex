#include <memory>
#include "kortex2_hacks/twist_to_array_republisher.hpp"
#include "rclcpp/rclcpp.hpp"

// Subscribe to twist, convert to Float64MultiArray so we can jog the arm with the Kortex api

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor exec;
  rclcpp::NodeOptions options;
  auto twist_republisher = std::make_shared<kortex2_hacks::TwistToArrayRepublisher>(options);
  exec.add_node(twist_republisher);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
