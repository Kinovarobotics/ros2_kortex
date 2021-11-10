#ifndef KORTEX2_CONTROLLERS__TWIST_CONTROLLER_HPP_
#define KORTEX2_CONTROLLERS__TWIST_CONTROLLER_HPP_

#include "controller_interface/controller_interface.hpp"
#include "kortex2_controllers/visibility_control.h"
#include "realtime_tools/realtime_buffer.h"
#include "geometry_msgs/msg/twist_stamped.hpp"

namespace kortex2_controllers
{
using CmdType = geometry_msgs::msg::TwistStamped;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class TwistController : public controller_interface::ControllerInterface
{
public:
  KORTEX2_CONTROLLERS_PUBLIC
  TwistController();

  KORTEX2_CONTROLLERS_PUBLIC
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  KORTEX2_CONTROLLERS_PUBLIC
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  KORTEX2_CONTROLLERS_PUBLIC
  CallbackReturn on_init() override;

  KORTEX2_CONTROLLERS_PUBLIC
  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;

  KORTEX2_CONTROLLERS_PUBLIC
  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

  KORTEX2_CONTROLLERS_PUBLIC
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

  KORTEX2_CONTROLLERS_PUBLIC
  controller_interface::return_type update(const rclcpp::Time& time, const rclcpp::Duration& period) override;

protected:
  std::string joint_name_;
  std::vector<std::string> interface_names_;

  realtime_tools::RealtimeBuffer<std::shared_ptr<CmdType>> rt_command_ptr_;
  rclcpp::Subscription<CmdType>::SharedPtr twist_command_subscriber_;

  std::string logger_name_;
};

}  // namespace kortex2_controllers

#endif  // KORTEX2_CONTROLLERS__TWIST_CONTROLLER_HPP_
