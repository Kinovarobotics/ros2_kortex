#pragma once

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace kortex2_hacks
{
class TwistToArrayRepublisher : public rclcpp::Node
{
public:
  TwistToArrayRepublisher(rclcpp::NodeOptions options);

private:
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr array_pub_;
};

}  // namespace kortex2_hacks
