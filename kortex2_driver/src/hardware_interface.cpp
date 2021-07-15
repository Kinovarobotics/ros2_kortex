#include "kortex2_driver/hardware_interface.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace kortex2_driver
{
return_type KortexMultiInterfaceHardware::configure(const hardware_interface::HardwareInfo & info)
{
  if (configure_default(info) != return_type::OK)
  {
    return return_type::ERROR;
  }

  hw_start_sec_ = stod(info_.hardware_parameters["example_param_hw_start_duration_sec"]);
  hw_stop_sec_ = stod(info_.hardware_parameters["example_param_hw_stop_duration_sec"]);
  hw_slowdown_ = stod(info_.hardware_parameters["example_param_hw_slowdown"]);
  hw_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_efforts_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_efforts_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  control_lvl_.resize(info_.joints.size(), integration_lvl_t::POSITION);

  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    // KortexMultiInterface has exactly 3 state interfaces
    // and 3 command interfaces on each joint
    if (joint.command_interfaces.size() != 3)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("KortexMultiInterfaceHardware"),
        "Joint '%s' has %d command interfaces. 3 expected.", joint.name.c_str());
      return return_type::ERROR;
    }

    if (!(joint.command_interfaces[0].name == hardware_interface::HW_IF_POSITION ||
          joint.command_interfaces[0].name == hardware_interface::HW_IF_VELOCITY ||
          joint.command_interfaces[0].name == hardware_interface::HW_IF_EFFORT))
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("KortexMultiInterfaceHardware"),
        "Joint '%s' has %s command interface. Expected %s, %s, or %s.", joint.name.c_str(),
        joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION,
        hardware_interface::HW_IF_VELOCITY, hardware_interface::HW_IF_EFFORT);
      return return_type::ERROR;
    }

    if (joint.state_interfaces.size() != 3)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("KortexMultiInterfaceHardware"),
        "Joint '%s'has %d state interfaces. 3 expected.", joint.name.c_str());
      return return_type::ERROR;
    }

    if (!(joint.state_interfaces[0].name == hardware_interface::HW_IF_POSITION ||
          joint.state_interfaces[0].name == hardware_interface::HW_IF_VELOCITY ||
          joint.state_interfaces[0].name == hardware_interface::HW_IF_EFFORT))
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("KortexMultiInterfaceHardware"),
        "Joint '%s' has %s state interface. Expected %s, %s, or %s.", joint.name.c_str(),
        joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION,
        hardware_interface::HW_IF_VELOCITY, hardware_interface::HW_IF_EFFORT);
      return return_type::ERROR;
    }
  }

  status_ = hardware_interface::status::CONFIGURED;
  return return_type::OK;
}

std::vector<hardware_interface::StateInterface>
KortexMultiInterfaceHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (std::size_t i = 0; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_efforts_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
KortexMultiInterfaceHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (std::size_t i = 0; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_positions_[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_velocities_[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_EFFORT,
      &hw_commands_efforts_[i]));
  }

  return command_interfaces;
}

return_type KortexMultiInterfaceHardware::prepare_command_mode_switch(
  const std::vector<std::string> & start_interfaces,
  const std::vector<std::string> & stop_interfaces)
{
  // Prepare for new command modes
  std::vector<integration_lvl_t> new_modes = {};
  for (std::string key : start_interfaces)
  {
    for (std::size_t i = 0; i < info_.joints.size(); i++)
    {
      if (key == info_.joints[i].name + "/" + hardware_interface::HW_IF_POSITION)
      {
        new_modes.push_back(integration_lvl_t::POSITION);
      }
      if (key == info_.joints[i].name + "/" + hardware_interface::HW_IF_VELOCITY)
      {
        new_modes.push_back(integration_lvl_t::VELOCITY);
      }
      if (key == info_.joints[i].name + "/" + hardware_interface::HW_IF_EFFORT)
      {
        new_modes.push_back(integration_lvl_t::EFFORT);
      }
    }
  }
  // Example criteria: All joints must be given new command mode at the same time
  if (new_modes.size() != info_.joints.size())
  {
    return return_type::ERROR;
  }
  // Example criteria: All joints must have the same command mode
  if (!std::all_of(new_modes.begin() + 1, new_modes.end(), [&](integration_lvl_t mode) {
        return mode == new_modes[0];
      }))
  {
    return return_type::ERROR;
  }

  // Stop motion on all relevant joints that are stopping
  for (std::string key : stop_interfaces)
  {
    for (std::size_t i = 0; i < info_.joints.size(); i++)
    {
      if (key.find(info_.joints[i].name) != std::string::npos)
      {
        hw_commands_velocities_[i] = 0;
        hw_commands_efforts_[i] = 0;
        control_lvl_[i] = integration_lvl_t::UNDEFINED;  // Revert to undefined
      }
    }
  }
  // Set the new command modes
  for (std::size_t i = 0; i < info_.joints.size(); i++)
  {
    if (control_lvl_[i] != integration_lvl_t::UNDEFINED)
    {
      // Something else is using the joint! Abort!
      return return_type::ERROR;
    }
    control_lvl_[i] = new_modes[i];
  }
  return return_type::OK;
}

return_type KortexMultiInterfaceHardware::start()
{
  RCLCPP_INFO(
    rclcpp::get_logger("KortexMultiInterfaceHardware"), "Starting... please wait...");

  for (int i = 0; i <= hw_start_sec_; i++)
  {
    rclcpp::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(
      rclcpp::get_logger("KortexMultiInterfaceHardware"), "%.1f seconds left...",
      hw_start_sec_ - i);
  }

  // The robot's IP address.
  std::string robot_ip = info_.hardware_parameters["robot_ip"];
  // Username to log into the robot controller
  std::string username = "admin"; //TODO: read in info_.hardware_parameters["username"];
  // Password to log into the robot controller
  std::string password = "admin"; //TODO: read in info_.hardware_parameters["password"];

  auto error_callback = [](k_api::KError err){ cout << "_________ callback error _________" << err.toString(); };
  auto transport = new k_api::TransportClientTcp();
  auto router = new k_api::RouterClient(transport, error_callback);
  transport->connect(robot_ip, PORT);

  // Set session data connection information
  auto create_session_info = k_api::Session::CreateSessionInfo();
  create_session_info.set_username(username);
  create_session_info.set_password(password);
  create_session_info.set_session_inactivity_timeout(60000);   // (milliseconds)
  create_session_info.set_connection_inactivity_timeout(2000); // (milliseconds)

  // Session manager service wrapper
  RCLCPP_INFO(rclcpp::get_logger("KortexMultiInterfaceHardware"), "Creating session for communication");
  auto session_manager = new k_api::SessionManager(router);
  session_manager->CreateSession(create_session_info);
  RCLCPP_INFO(rclcpp::get_logger("KortexMultiInterfaceHardware"), "Session created");

  // Create services
  auto base = new k_api::Base::BaseClient(router);
  auto base_cyclic = new k_api::BaseCyclic::BaseCyclicClient(router);

  // Set some default values
  for (std::size_t i = 0; i < hw_positions_.size(); i++)
  {
    if (std::isnan(hw_positions_[i]))
    {
      hw_positions_[i] = 0;
    }
    if (std::isnan(hw_velocities_[i]))
    {
      hw_velocities_[i] = 0;
    }
    if (std::isnan(hw_efforts_[i]))
    {
      hw_efforts_[i] = 0;
    }
    if (std::isnan(hw_commands_positions_[i]))
    {
      hw_commands_positions_[i] = 0;
    }
    if (std::isnan(hw_commands_velocities_[i]))
    {
      hw_commands_velocities_[i] = 0;
    }
    if (std::isnan(hw_commands_efforts_[i]))
    {
      hw_commands_efforts_[i] = 0;
    }
    control_lvl_[i] = integration_lvl_t::UNDEFINED;
  }
  status_ = hardware_interface::status::STARTED;

  RCLCPP_INFO(
    rclcpp::get_logger("KortexMultiInterfaceHardware"), "System successfully started! %u",
    control_lvl_[0]);
  return return_type::OK;
}

return_type KortexMultiInterfaceHardware::stop()
{
  RCLCPP_INFO(
    rclcpp::get_logger("KortexMultiInterfaceHardware"), "Stopping... please wait...");

  for (int i = 0; i <= hw_stop_sec_; i++)
  {
    rclcpp::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(
      rclcpp::get_logger("KortexMultiInterfaceHardware"), "%.1f seconds left...",
      hw_stop_sec_ - i);
  }

  status_ = hardware_interface::status::STOPPED;

  RCLCPP_INFO(
    rclcpp::get_logger("KortexMultiInterfaceHardware"), "System successfully stopped!");

  return return_type::OK;
}

return_type KortexMultiInterfaceHardware::read()
{
  for (std::size_t i = 0; i < hw_positions_.size(); i++)
  {
    switch (control_lvl_[i])
    {
      case integration_lvl_t::UNDEFINED:
        RCLCPP_INFO(
          rclcpp::get_logger("KortexMultiInterfaceHardware"),
          "Nothing is using the hardware interface!");
        return return_type::OK;
        break;
      case integration_lvl_t::POSITION:
        hw_efforts_[i] = 0;
        hw_velocities_[i] = 0;
        hw_positions_[i] = hw_commands_positions_[i];
        break;
      case integration_lvl_t::VELOCITY:
        hw_efforts_[i] = 0;
        hw_velocities_[i] = hw_commands_velocities_[i];
        break;
      case integration_lvl_t::EFFORT:
        hw_efforts_[i] = hw_commands_efforts_[i];
        break;
    }
    // Using the hw_slowdown_ parameter as a timestep
    hw_velocities_[i] += hw_slowdown_ * hw_efforts_[i];
    hw_positions_[i] += hw_slowdown_ * hw_velocities_[i];
    RCLCPP_INFO(
      rclcpp::get_logger("KortexMultiInterfaceHardware"),
      "Got pos: %.5f, vel: %.5f, eff: %.5f for joint %d!", hw_positions_[i], hw_velocities_[i],
      hw_efforts_[i], i);
  }
  return return_type::OK;
}

return_type KortexMultiInterfaceHardware::write()
{
  /*RCLCPP_INFO(
    rclcpp::get_logger("KortexMultiInterfaceHardware"),
    "Writing...");*/
  for (std::size_t i = 0; i < hw_commands_positions_.size(); i++)
  {
    // Simulate sending commands to the hardware
    RCLCPP_INFO(
      rclcpp::get_logger("KortexMultiInterfaceHardware"),
      "Got the commands pos: %.5f, vel: %.5f, eff: %.5f for joint %d, control_lvl: %d",
      hw_commands_positions_[i], hw_commands_velocities_[i], hw_commands_efforts_[i], i,
      control_lvl_[i]);
  }
  return return_type::OK;
}

}  // namespace kortex2_driver

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  kortex2_driver::KortexMultiInterfaceHardware,
  hardware_interface::SystemInterface)
