#include "kortex2_hacks/twist_to_array_republisher.hpp"
#include "rclcpp/rclcpp.hpp"

namespace kortex2_hacks
{
namespace
{
constexpr char INCOMING_TWIST_TOPIC[] = "/twist_cmd";
constexpr char OUTGOING_ARRAY_TOPIC[] = "/streaming_controller/command";
}  // namespace

TwistToArrayRepublisher::TwistToArrayRepublisher(rclcpp::NodeOptions options)
  : Node("twist_to_array_republisher", options)
{
  twist_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      INCOMING_TWIST_TOPIC, 1, [this](geometry_msgs::msg::Twist::UniquePtr msg) {
        RCLCPP_INFO_STREAM(this->get_logger(), "Received this twist cmd: " << msg->linear.x);
      });

  array_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(OUTGOING_ARRAY_TOPIC, 1);
}

}  // namespace kortex2_hacks

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(kortex2_hacks::TwistToArrayRepublisher)
