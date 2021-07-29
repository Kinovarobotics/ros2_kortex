#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
// Publish true if contact detected, false otherwise
#include <std_msgs/msg/bool.hpp>

#include "collision_sensor/contact_detection/contact_detection.hpp"

namespace collision_sensor
{
namespace
{
constexpr char LOGNAME[] = "bag_subscriber";
}

class BagSubscriber : public rclcpp::Node
{
public:
  BagSubscriber()
    : Node(LOGNAME)
    , contact_monitor_(7 /* dof */, 0.5 /* torque threshold [Nm] */, 5 /* consecutive outliers to trigger */)
  {
    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 1, std::bind(&BagSubscriber::jointStateCB, this, std::placeholders::_1));

    contact_pub_ = this->create_publisher<std_msgs::msg::Bool>("/contact_detected", 1);
  }

private:
  // Check for collisions here
  void jointStateCB(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    auto bool_msg = std_msgs::msg::Bool();

    if (contact_monitor_.registerMeasurement(msg->effort) == ReturnCode::CONTACT_DETECTED)
    {
      RCLCPP_ERROR_STREAM(this->get_logger(), "Contact!");
      bool_msg.data = true;
    }
    else
    {
      RCLCPP_INFO_STREAM(this->get_logger(), "No contact.");
      bool_msg.data = false;
    }
    contact_pub_->publish(bool_msg);
  }

  // Object for contact detection
  ContactDetection contact_monitor_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr contact_pub_;
};
}  // namespace collision_sensor

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<collision_sensor::BagSubscriber>());
  rclcpp::shutdown();
  return 0;
}
