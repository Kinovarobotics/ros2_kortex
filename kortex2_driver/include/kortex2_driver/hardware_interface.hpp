#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/macros.hpp"

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

using hardware_interface::return_type;

namespace k_api = Kinova::Api;
#define PORT 10000

namespace kortex2_driver
{
class KortexMultiInterfaceHardware
: public hardware_interface::BaseInterface<hardware_interface::SystemInterface>
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(KortexMultiInterfaceHardware);

  KORTEX2_DRIVER_PUBLIC
  return_type configure(const hardware_interface::HardwareInfo & info) final;

  KORTEX2_DRIVER_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() final;

  KORTEX2_DRIVER_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() final;

  KORTEX2_DRIVER_PUBLIC
  return_type prepare_command_mode_switch(
    const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> & stop_interfaces) final;

  KORTEX2_DRIVER_PUBLIC
  return_type start() final;

  KORTEX2_DRIVER_PUBLIC
  return_type stop() final;

  KORTEX2_DRIVER_PUBLIC
  return_type read() final;

  KORTEX2_DRIVER_PUBLIC
  return_type write() final;

private:
  // Parameters for the Robot simulation
  double hw_start_sec_;
  double hw_stop_sec_;
  double hw_slowdown_;

  // Store the commands for the simulated robot
  std::vector<double> hw_commands_positions_;
  std::vector<double> hw_commands_velocities_;
  std::vector<double> hw_commands_efforts_;
  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;
  std::vector<double> hw_efforts_;

  // Enum defining at which control level we are
  // Dumb way of maintaining the command_interface type per joint.
  enum class integration_lvl_t : std::uint8_t
  {
    UNDEFINED = 0,
    POSITION = 1,
    VELOCITY = 2,
    EFFORT = 3
  };

  std::vector<integration_lvl_t> control_lvl_;
};

}  // namespace kortex2_driver
