#pragma once

#include <atomic>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"

#include "hardware_interface/base_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_status_values.hpp"

#include "kortex2_driver/visibility_control.h"

#include <BaseClientRpc.h>
#include <BaseCyclicClientRpc.h>
#include <SessionManager.h>
#include <RouterClient.h>
#include <TransportClientTcp.h>
#include <TransportClientUdp.h>

using hardware_interface::return_type;

namespace k_api = Kinova::Api;
#define PORT 10000
#define PORT_REAL_TIME 10001

namespace kortex2_driver
{
class KortexMultiInterfaceHardware : public hardware_interface::SystemInterface
{
public:
  KortexMultiInterfaceHardware();

  RCLCPP_SHARED_PTR_DEFINITIONS(KortexMultiInterfaceHardware);

  KORTEX2_DRIVER_PUBLIC
  CallbackReturn on_init(const hardware_interface::HardwareInfo& info) final;

  KORTEX2_DRIVER_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() final;

  KORTEX2_DRIVER_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() final;

  KORTEX2_DRIVER_PUBLIC
  return_type prepare_command_mode_switch(const std::vector<std::string>& start_interfaces,
                                          const std::vector<std::string>& stop_interfaces) final;
  KORTEX2_DRIVER_PUBLIC
  return_type perform_command_mode_switch(const std::vector<std::string>& /*start_interfaces*/,
                                          const std::vector<std::string>& /*stop_interfaces*/) final;

  KORTEX2_DRIVER_PUBLIC
  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) final;

  KORTEX2_DRIVER_PUBLIC
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) final;

  KORTEX2_DRIVER_PUBLIC
  return_type read() final;

  KORTEX2_DRIVER_PUBLIC
  return_type write() final;

private:
  k_api::TransportClientTcp transport_tcp_;
  k_api::RouterClient router_tcp_;
  k_api::SessionManager session_manager_;
  k_api::TransportClientUdp transport_udp_realtime_;
  k_api::RouterClient router_udp_realtime_;
  k_api::SessionManager session_manager_real_time_;

  // Control of the robot arm itself
  k_api::Base::BaseClient base_;
  k_api::BaseCyclic::BaseCyclicClient base_cyclic_;
  k_api::BaseCyclic::Command base_command_;
  std::size_t actuator_count_;
  // To minimize bandwidth we syncronize feedback with the robot only when write() is called
  k_api::BaseCyclic::Feedback feedback_;
  std::vector<double> arm_commands_positions_;
  std::vector<double> arm_commands_velocities_;
  std::vector<double> arm_commands_efforts_;
  std::vector<double> arm_positions_;
  std::vector<double> arm_velocities_;
  std::vector<double> arm_efforts_;

  // twist command interfaces
  std::vector<double> twist_commands_;

  // Gripper
  k_api::GripperCyclic::MotorCommand* gripper_motor_command_;
  double gripper_command_position_;
  double gripper_position_;

  rclcpp::Time controller_switch_time_;
  std::atomic<bool> block_write = false;
  k_api::Base::ServoingMode arm_mode_;

  // Enum defining at which control level we are
  // Dumb way of maintaining the command_interface type per joint.
  enum class integration_lvl_t : std::uint8_t
  {
    UNDEFINED = 0,
    POSITION = 1,
    VELOCITY = 2,
    EFFORT = 3
  };

  std::vector<integration_lvl_t> arm_joints_control_level_;

  // what controller is running
  bool joint_based_controller_running_;
  bool twist_controller_running_;

  // temp variables to use in update loop
  float cmd_degrees_tmp_;
  float cmd_vel_tmp_;
  int num_turns_tmp_ = 0;

  void sendTwistCommand();
  void sendJointCommand();
  void incrementId();
  void writeCommands();
  void prepareCommands();
  void sendGripperCommand(k_api::Base::ServoingMode arm_mode, double position, double velocity = 100.0,
                          double force = 100.0);

  void readGripperPosition();
};

}  // namespace kortex2_driver
