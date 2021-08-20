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
      INCOMING_TWIST_TOPIC, 1, std::bind(&TwistToArrayRepublisher::twistCallback, this, std::placeholders::_1));

  array_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(OUTGOING_ARRAY_TOPIC, 1);
}

void TwistToArrayRepublisher::twistCallback(const geometry_msgs::msg::Twist::ConstPtr& twist_msg) const
{
  // Convert twist to array
  std_msgs::msg::Float64MultiArray array_msg;
  array_msg.data.resize(6);
  array_msg.data[0] = twist_msg->linear.x;
  array_msg.data[1] = twist_msg->linear.y;
  array_msg.data[2] = twist_msg->linear.z;
  array_msg.data[3] = twist_msg->angular.x;
  array_msg.data[4] = twist_msg->angular.y;
  array_msg.data[5] = twist_msg->angular.z;

  array_pub_->publish(array_msg);
}

}  // namespace kortex2_hacks

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(kortex2_hacks::TwistToArrayRepublisher)
