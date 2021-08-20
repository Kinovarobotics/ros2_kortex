#include <memory>
#include "kortex2_hacks/twist_to_array_republisher.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor exec;
  rclcpp::NodeOptions options;
  //  auto array_pub_node = std::make_shared<PublisherNode>(options);
  auto twist_subscriber = std::make_shared<kortex2_hacks::TwistToArrayRepublisher>(options);
  //  exec.add_node(array_pub_node);
  exec.add_node(twist_subscriber);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
