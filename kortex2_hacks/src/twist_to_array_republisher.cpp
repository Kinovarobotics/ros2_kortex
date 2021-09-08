#include "kortex2_hacks/twist_to_array_republisher.hpp"
#include "rclcpp/rclcpp.hpp"

namespace kortex2_hacks
{
namespace
{
constexpr char INCOMING_TWIST_TOPIC[] = "/servo_server/delta_twist_cmds";
constexpr char OUTGOING_ARRAY_TOPIC[] = "/streaming_controller/commands";
}  // namespace

TwistToArrayRepublisher::TwistToArrayRepublisher(rclcpp::NodeOptions options)
  : Node("twist_to_array_republisher", options)
{
  twist_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
      INCOMING_TWIST_TOPIC, 1, std::bind(&TwistToArrayRepublisher::twistCallback, this, std::placeholders::_1));

  array_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(OUTGOING_ARRAY_TOPIC, 1);

  last_user_msg_time_ = rclcpp::Clock().now();

  timer_ =
      this->create_wall_timer(std::chrono::milliseconds(200), std::bind(&TwistToArrayRepublisher::timerCallback, this));
}

void TwistToArrayRepublisher::timerCallback()
{
  if ((rclcpp::Clock().now() - last_user_msg_time_).seconds() > 0.1)
  {
    std_msgs::msg::Float64MultiArray array_msg;
    array_msg.data.resize(7);
    array_msg.data[0] = 0.0;
    array_msg.data[1] = 0.0;
    array_msg.data[2] = 0.0;
    array_msg.data[3] = 0.0;
    array_msg.data[4] = 0.0;
    array_msg.data[5] = 0.0;
    array_msg.data[6] = 0.0;

    array_pub_->publish(array_msg);
    last_user_msg_time_ = rclcpp::Clock().now();
  }
}

void TwistToArrayRepublisher::twistCallback(const geometry_msgs::msg::TwistStamped::ConstPtr& twist_msg)
{
  float scale_linear_mult = 0.07;  // scale the incomming linear velocity commands
  float scale_angular_mult = 4.0;  // scale the incomming angular velocity commands
  // Convert twist to array
  std_msgs::msg::Float64MultiArray array_msg;
  array_msg.data.resize(7);
  array_msg.data[0] = twist_msg->twist.linear.x * scale_linear_mult;
  array_msg.data[1] = twist_msg->twist.linear.y * scale_linear_mult;
  array_msg.data[2] = twist_msg->twist.linear.z * scale_linear_mult;
  // array_msg.data[3] = twist_msg->twist.angular.x * scale_angular_mult;
  // array_msg.data[4] = twist_msg->twist.angular.y * scale_angular_mult;
  array_msg.data[5] = twist_msg->twist.angular.z * scale_angular_mult;
  array_msg.data[6] = 0.0;

  array_pub_->publish(array_msg);
  last_user_msg_time_ = rclcpp::Clock().now();
}

}  // namespace kortex2_hacks

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(kortex2_hacks::TwistToArrayRepublisher)
