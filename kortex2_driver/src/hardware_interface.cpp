#include "kortex2_driver/hardware_interface.hpp"
#include "kortex2_driver/kortex_math_util.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace
{
const rclcpp::Logger LOGGER = rclcpp::get_logger("KortexMultiInterfaceHardware");
}

namespace kortex2_driver
{
KortexMultiInterfaceHardware::KortexMultiInterfaceHardware()
  : router_tcp_{ &transport_tcp_,
                 [](k_api::KError err) { cout << "_________ callback error _________" << err.toString(); } }
  , session_manager_{ &router_tcp_ }
  , router_udp_realtime_{ &transport_udp_realtime_,
                          [](k_api::KError err) { cout << "_________ callback error _________" << err.toString(); } }
  , session_manager_real_time_{ &router_udp_realtime_ }
  , base_{ &router_tcp_ }
  , base_cyclic_{ &router_udp_realtime_ }
{
  rclcpp::on_shutdown(std::bind(&KortexMultiInterfaceHardware::stop, this));

  // The robot's IP address.
  std::string robot_ip = "192.168.1.10";  // TODO: read in info_.hardware_parameters["robot_ip"];
  // Username to log into the robot controller
  std::string username = "admin";  // TODO: read in info_.hardware_parameters["username"];
  // Password to log into the robot controller
  std::string password = "admin";  // TODO: read in info_.hardware_parameters["password"];

  RCLCPP_INFO_STREAM(LOGGER, "Connecting to robot at " << robot_ip);

  transport_tcp_.connect(robot_ip, PORT);
  transport_udp_realtime_.connect(robot_ip, PORT_REAL_TIME);

  // Set session data connection information
  auto create_session_info = k_api::Session::CreateSessionInfo();
  create_session_info.set_username(username);
  create_session_info.set_password(password);
  create_session_info.set_session_inactivity_timeout(60000);    // (milliseconds)
  create_session_info.set_connection_inactivity_timeout(2000);  // (milliseconds)

  // Session manager service wrapper
  RCLCPP_INFO(LOGGER, "Creating session for communication");
  session_manager_.CreateSession(create_session_info);
  session_manager_real_time_.CreateSession(create_session_info);
  RCLCPP_INFO(LOGGER, "Session created");

  auto servoing_mode = k_api::Base::ServoingModeInformation();
  // Set the base in low-level servoing mode
  servoing_mode.set_servoing_mode(k_api::Base::ServoingMode::LOW_LEVEL_SERVOING);
  base_.SetServoingMode(servoing_mode);
  actuator_count_ = base_.GetActuatorCount().count();
}

return_type KortexMultiInterfaceHardware::configure(const hardware_interface::HardwareInfo& info)
{
  RCLCPP_INFO(LOGGER, "Configuring Hardware Interface");
  if (configure_default(info) != return_type::OK)
  {
    return return_type::ERROR;
  }

  info_ = info;

  arm_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  arm_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  arm_efforts_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  arm_commands_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  arm_commands_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  arm_commands_efforts_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  gripper_command_position_ = std::numeric_limits<double>::quiet_NaN();
  gripper_position_ = std::numeric_limits<double>::quiet_NaN();
  arm_joints_control_level_.resize(info_.joints.size(), integration_lvl_t::POSITION);

  for (const hardware_interface::ComponentInfo& joint : info_.joints)
  {
    if (!(joint.command_interfaces[0].name == hardware_interface::HW_IF_POSITION ||
          joint.command_interfaces[0].name == hardware_interface::HW_IF_VELOCITY ||
          joint.command_interfaces[0].name == hardware_interface::HW_IF_EFFORT))
    {
      RCLCPP_FATAL(LOGGER, "Joint '%s' has %s command interface. Expected %s, %s, or %s.", joint.name.c_str(),
                   joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION,
                   hardware_interface::HW_IF_VELOCITY, hardware_interface::HW_IF_EFFORT);
      return return_type::ERROR;
    }

    if (!(joint.state_interfaces[0].name == hardware_interface::HW_IF_POSITION ||
          joint.state_interfaces[0].name == hardware_interface::HW_IF_VELOCITY ||
          joint.state_interfaces[0].name == hardware_interface::HW_IF_EFFORT))
    {
      RCLCPP_FATAL(LOGGER, "Joint '%s' has %s state interface. Expected %s, %s, or %s.", joint.name.c_str(),
                   joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION,
                   hardware_interface::HW_IF_VELOCITY, hardware_interface::HW_IF_EFFORT);
      return return_type::ERROR;
    }
  }

  RCLCPP_INFO(LOGGER, "Hardware Interface successfully configured");
  status_ = hardware_interface::status::CONFIGURED;
  return return_type::OK;
}

std::vector<hardware_interface::StateInterface> KortexMultiInterfaceHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (std::size_t i = 0; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &arm_positions_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &arm_velocities_[i]));
    state_interfaces.emplace_back(
        hardware_interface::StateInterface(info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &arm_efforts_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> KortexMultiInterfaceHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (std::size_t i = 0; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &arm_commands_positions_[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &arm_commands_velocities_[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &arm_commands_efforts_[i]));
  }

  return command_interfaces;
}

return_type KortexMultiInterfaceHardware::prepare_command_mode_switch(const std::vector<std::string>& start_interfaces,
                                                                      const std::vector<std::string>& stop_interfaces)
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

  // Stop motion on all relevant joints that are stopping
  for (std::string key : stop_interfaces)
  {
    for (std::size_t i = 0; i < info_.joints.size(); i++)
    {
      if (key.find(info_.joints[i].name) != std::string::npos)
      {
        arm_commands_velocities_[i] = 0;
        arm_commands_efforts_[i] = 0;
        arm_joints_control_level_[i] = integration_lvl_t::UNDEFINED;  // Revert to undefined
      }
    }
  }

  // Set the new command modes
  for (std::size_t i = 0; i < info_.joints.size(); i++)
  {
    arm_joints_control_level_[i] = new_modes[i];
  }

  return return_type::OK;
}

return_type KortexMultiInterfaceHardware::start()
{
  auto base_feedback = base_cyclic_.RefreshFeedback();
  // Add each actuator to the base_command_ and set the command to its current position
  /*
      for (std::size_t i = 0; i < actuator_count_; i++)
      {
        base_command_.add_actuators()->set_position(base_feedback.actuators(i).position());
      }

    // Initialize gripper
    float gripper_initial_position = base_feedback.interconnect().gripper_feedback().motor()[0].position();
    // Initialize interconnect command to current gripper position.
    base_command_.mutable_interconnect()->mutable_command_id()->set_identifier(0);

    gripper_motor_command_ = base_command_.mutable_interconnect()->mutable_gripper_command()->add_motor_cmd();
    gripper_motor_command_->set_position(gripper_initial_position);
    gripper_motor_command_->set_velocity(0.0);
    gripper_motor_command_->set_force(100.0);

    // Send a first frame
    base_feedback = base_cyclic_.Refresh(base_command_);
  */
  // Set some default values
  for (std::size_t i = 0; i < actuator_count_; i++)
  {
    if (std::isnan(arm_positions_[i]))
    {
      arm_positions_[i] = KortexMathUtil::wrapRadiansFromMinusPiToPi(
          KortexMathUtil::toRad(base_feedback.actuators(i).position()));  // rad
    }
    if (std::isnan(arm_velocities_[i]))
    {
      arm_velocities_[i] = 0;
    }
    if (std::isnan(arm_efforts_[i]))
    {
      arm_efforts_[i] = 0;
    }
    if (std::isnan(arm_commands_positions_[i]))
    {
      arm_commands_positions_[i] = KortexMathUtil::wrapRadiansFromMinusPiToPi(
          KortexMathUtil::toRad(base_feedback.actuators(i).position()));  // rad
    }
    if (std::isnan(arm_commands_velocities_[i]))
    {
      arm_commands_velocities_[i] = 0;
    }
    if (std::isnan(arm_commands_efforts_[i]))
    {
      arm_commands_efforts_[i] = 0;
    }
    arm_joints_control_level_[i] = integration_lvl_t::UNDEFINED;
  }
  status_ = hardware_interface::status::STARTED;

  RCLCPP_INFO(LOGGER, "System successfully started! %u", arm_joints_control_level_[0]);
  return return_type::OK;
}

return_type KortexMultiInterfaceHardware::stop()
{
  RCLCPP_INFO(LOGGER, "Stopping... please wait...");

  auto servoing_mode = k_api::Base::ServoingModeInformation();
  // Set back the servoing mode to Single Level Servoing
  servoing_mode.set_servoing_mode(k_api::Base::ServoingMode::LOW_LEVEL_SERVOING);
  base_.SetServoingMode(servoing_mode);

  // Close API session
  session_manager_.CloseSession();
  session_manager_real_time_.CloseSession();

  // Deactivate the router and cleanly disconnect from the transport object
  router_tcp_.SetActivationStatus(false);
  transport_tcp_.disconnect();
  router_udp_realtime_.SetActivationStatus(false);
  transport_udp_realtime_.disconnect();

  status_ = hardware_interface::status::STOPPED;

  RCLCPP_INFO(LOGGER, "System successfully stopped!");

  return return_type::OK;
}

return_type KortexMultiInterfaceHardware::read()
{
  auto feedback = base_cyclic_.RefreshFeedback();
  for (std::size_t i = 0; i < actuator_count_; i++)
  {
    switch (arm_joints_control_level_[i])
    {
      case integration_lvl_t::UNDEFINED:
        RCLCPP_INFO(LOGGER, "Nothing is using the hardware interface!");
        return return_type::OK;
        break;
      case integration_lvl_t::POSITION:
        arm_efforts_[i] = feedback.actuators(i).torque();                              // N*m
        arm_velocities_[i] = KortexMathUtil::toRad(feedback.actuators(i).velocity());  // rad/sec
        arm_positions_[i] =
            KortexMathUtil::wrapRadiansFromMinusPiToPi(KortexMathUtil::toRad(feedback.actuators(i).position()));  // rad
        break;
      case integration_lvl_t::VELOCITY:
        arm_efforts_[i] = feedback.actuators(i).torque();                              // N*m
        arm_velocities_[i] = KortexMathUtil::toRad(feedback.actuators(i).velocity());  // rad/sec
        break;
      case integration_lvl_t::EFFORT:
        arm_efforts_[i] = feedback.actuators(i).torque();  // N*m
        break;
    }
  }
  return return_type::OK;
}

return_type KortexMultiInterfaceHardware::write()
{
  Kinova::Api::BaseCyclic::Feedback feedback;
  /*
    // If gripper command is different than current position, send it
    if (gripper_command_position_ != gripper_position_)
    {
      /*
      // example:
      //
    https://github.com/Kinovarobotics/kortex/blob/master/api_cpp/examples/107-Gripper_low_level_command/01-gripper_low_level_command.cpp
      base_command_.mutable_interconnect()->mutable_command_id()->set_identifier(0);
      gripper_motor_command_ = base_command_.mutable_interconnect()->mutable_gripper_command()->add_motor_cmd();
      gripper_motor_command_->set_position(1);
      gripper_motor_command_->set_velocity(1);

      auto base_feedback_ = base_cyclic_.Refresh(base_command_);
      // TODO(andyz): returning here means the arm cannot move while gripper is actuated
      return return_type::OK;
    }
  */
  /*
    // Incrementing identifier ensures actuators can reject out of time frames
    base_command_.set_frame_id(base_command_.frame_id() + 1);
    if (base_command_.frame_id() > 65535)
      base_command_.set_frame_id(0);

    // update the command for each joint
    for (std::size_t i = 0; i < actuator_count_; i++)
    {
      float cmd_degrees = KortexMathUtil::wrapDegreesFromZeroTo360(KortexMathUtil::toDeg(arm_commands_positions_[i]));
      float cmd_vel = KortexMathUtil::toDeg(arm_commands_velocities_[i]);

      base_command_.mutable_actuators(i)->set_position(cmd_degrees);
      base_command_.mutable_actuators(i)->set_velocity(cmd_vel);
      base_command_.mutable_actuators(i)->set_command_id(base_command_.frame_id());
    }

    // send the command to the robot
    try
    {
      feedback = base_cyclic_.Refresh(base_command_, 0);
    }
    catch (k_api::KDetailedException& ex)
    {
      RCLCPP_ERROR_STREAM(LOGGER, "Kortex exception: " << ex.what());

      RCLCPP_ERROR_STREAM(LOGGER, "Error sub-code: " << k_api::SubErrorCodes_Name(
                                      k_api::SubErrorCodes((ex.getErrorInfo().getError().error_sub_code()))));
    }
  */
  return return_type::OK;
}

}  // namespace kortex2_driver

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(kortex2_driver::KortexMultiInterfaceHardware, hardware_interface::SystemInterface)
