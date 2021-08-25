#pragma once

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace kortex2_hacks
{
class TwistToArrayRepublisher : public rclcpp::Node
{
public:
  TwistToArrayRepublisher(rclcpp::NodeOptions options);

private:
  void twistCallback(const geometry_msgs::msg::TwistStamped::ConstPtr& twist_msg);
  void timerCallback();

  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twist_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr array_pub_;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time last_user_msg_time_;
};

}  // namespace kortex2_hacks
