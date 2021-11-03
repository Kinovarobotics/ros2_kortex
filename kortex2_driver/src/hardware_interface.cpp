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
  , first_pass_(true)
  , gripper_joint_name_("finger_joint")
{
  // The robot's IP address.
  std::string robot_ip = "192.168.11.11";  // TODO: read in info_.hardware_parameters["robot_ip"];
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

  // make sure robot is in unspecified servoing mode
  // decide this on controller switching
  auto servoingMode = k_api::Base::ServoingModeInformation();
  servoingMode.set_servoing_mode(k_api::Base::ServoingMode::UNSPECIFIED_SERVOING_MODE);
  base_.SetServoingMode(servoingMode);
  arm_mode_ = k_api::Base::ServoingMode::UNSPECIFIED_SERVOING_MODE;
  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  actuator_count_ = base_.GetActuatorCount().count();
  RCLCPP_INFO(LOGGER, "Actuator count reported by robot is '%lu'", actuator_count_);

  // no controller is running
  joint_based_controller_running_ = false;
  twist_controller_running_ = false;
  gripper_controller_running_ = false;
}

CallbackReturn KortexMultiInterfaceHardware::on_init(const hardware_interface::HardwareInfo& info)
{
  RCLCPP_INFO(LOGGER, "Configuring Hardware Interface");
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  info_ = info;

  arm_positions_.resize(actuator_count_, std::numeric_limits<double>::quiet_NaN());
  arm_velocities_.resize(actuator_count_, std::numeric_limits<double>::quiet_NaN());
  arm_efforts_.resize(actuator_count_, std::numeric_limits<double>::quiet_NaN());
  arm_commands_positions_.resize(actuator_count_, std::numeric_limits<double>::quiet_NaN());
  arm_commands_velocities_.resize(actuator_count_, std::numeric_limits<double>::quiet_NaN());
  arm_commands_efforts_.resize(actuator_count_, std::numeric_limits<double>::quiet_NaN());
  arm_joints_control_level_.resize(actuator_count_, integration_lvl_t::UNDEFINED);  // start in undefined
  gripper_command_position_ = std::numeric_limits<double>::quiet_NaN();
  gripper_position_ = std::numeric_limits<double>::quiet_NaN();

  // set size of the twist interface
  twist_commands_.resize(6);

  for (const hardware_interface::ComponentInfo& joint : info_.joints)
  {
    if (!(joint.command_interfaces[0].name == hardware_interface::HW_IF_POSITION ||
          joint.command_interfaces[0].name == hardware_interface::HW_IF_VELOCITY ||
          joint.command_interfaces[0].name == hardware_interface::HW_IF_EFFORT))
    {
      RCLCPP_FATAL(LOGGER, "Joint '%s' has %s command interface. Expected %s, %s, or %s.", joint.name.c_str(),
                   joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION,
                   hardware_interface::HW_IF_VELOCITY, hardware_interface::HW_IF_EFFORT);
      return CallbackReturn::ERROR;
    }

    if (!(joint.state_interfaces[0].name == hardware_interface::HW_IF_POSITION ||
          joint.state_interfaces[0].name == hardware_interface::HW_IF_VELOCITY ||
          joint.state_interfaces[0].name == hardware_interface::HW_IF_EFFORT))
    {
      RCLCPP_FATAL(LOGGER, "Joint '%s' has %s state interface. Expected %s, %s, or %s.", joint.name.c_str(),
                   joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION,
                   hardware_interface::HW_IF_VELOCITY, hardware_interface::HW_IF_EFFORT);
      return CallbackReturn::ERROR;
    }
  }

  RCLCPP_INFO(LOGGER, "Hardware Interface successfully configured");
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> KortexMultiInterfaceHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  std::vector<string> arm_joint_names;

  for (std::size_t i = 0; i < info_.joints.size(); i++)
  {
    RCLCPP_DEBUG(LOGGER, "export_state_interfaces for joint: %s", info_.joints[i].name.c_str());
    if (info_.joints[i].name == gripper_joint_name_)  // TODO find a better way to identify gripper joint(s)
    {
      state_interfaces.emplace_back(hardware_interface::StateInterface(
          info_.joints[i].name, hardware_interface::HW_IF_POSITION, &gripper_position_));
    }
    else
    {
      arm_joint_names.emplace_back(info_.joints[i].name);
    }
  }

  for (std::size_t i = 0; i < arm_joint_names.size(); i++)
  {
    state_interfaces.emplace_back(
        hardware_interface::StateInterface(arm_joint_names[i], hardware_interface::HW_IF_POSITION, &arm_positions_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        arm_joint_names[i], hardware_interface::HW_IF_VELOCITY, &arm_velocities_[i]));
    state_interfaces.emplace_back(
        hardware_interface::StateInterface(arm_joint_names[i], hardware_interface::HW_IF_EFFORT, &arm_efforts_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> KortexMultiInterfaceHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  std::vector<string> arm_joint_names;

  for (std::size_t i = 0; i < info_.joints.size(); i++)
  {
    // TODO find a better way to identify gripper joint(s)
    if (info_.joints[i].name == gripper_joint_name_)
    {
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
          info_.joints[i].name, hardware_interface::HW_IF_POSITION, &gripper_command_position_));
    }
    else
    {
      arm_joint_names.emplace_back(info_.joints[i].name);
    }
  }
  for (std::size_t i = 0; i < arm_joint_names.size(); i++)
  {
    {
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
          arm_joint_names[i], hardware_interface::HW_IF_POSITION, &arm_commands_positions_[i]));
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
          arm_joint_names[i], hardware_interface::HW_IF_VELOCITY, &arm_commands_velocities_[i]));
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
          arm_joint_names[i], hardware_interface::HW_IF_EFFORT, &arm_commands_efforts_[i]));
    }
  }

  // register twist command interfaces
  command_interfaces.emplace_back(hardware_interface::CommandInterface("tcp", "twist.linear.x", &twist_commands_[0]));
  command_interfaces.emplace_back(hardware_interface::CommandInterface("tcp", "twist.linear.y", &twist_commands_[1]));
  command_interfaces.emplace_back(hardware_interface::CommandInterface("tcp", "twist.linear.z", &twist_commands_[2]));
  command_interfaces.emplace_back(hardware_interface::CommandInterface("tcp", "twist.angular.x", &twist_commands_[3]));
  command_interfaces.emplace_back(hardware_interface::CommandInterface("tcp", "twist.angular.y", &twist_commands_[4]));
  command_interfaces.emplace_back(hardware_interface::CommandInterface("tcp", "twist.angular.z", &twist_commands_[5]));

  return command_interfaces;
}

return_type KortexMultiInterfaceHardware::prepare_command_mode_switch(const std::vector<std::string>& start_interfaces,
                                                                      const std::vector<std::string>& stop_interfaces)
{
  hardware_interface::return_type ret_val = hardware_interface::return_type::OK;

  // sleep to ensure all outgoing write commands have finished
  block_write = true;
  std::this_thread::sleep_for(std::chrono::milliseconds(200));

  start_modes_.clear();
  stop_modes_.clear();
  RCLCPP_INFO(LOGGER, "prepare START");

  // Starting interfaces
  // add start interface per joint in tmp var for later check
  for (const auto& key : start_interfaces)
  {
    RCLCPP_INFO(LOGGER, "Starting '%s'", key.c_str());
    for (auto& joint : info_.joints)
    {
      if (key == joint.name + "/" + hardware_interface::HW_IF_POSITION)
      {
        start_modes_.emplace_back(hardware_interface::HW_IF_POSITION);
      }
      if (key == joint.name + "/" + hardware_interface::HW_IF_VELOCITY)
      {
        start_modes_.emplace_back(hardware_interface::HW_IF_VELOCITY);
      }
      if (key == joint.name + "/" + hardware_interface::HW_IF_EFFORT)
      {
        continue;
        // not supporting effort command interface
        //              start_modes_.emplace_back(hardware_interface::HW_IF_EFFORT);
        RCLCPP_ERROR(LOGGER, "KortexMultiInterfaceHardware does not support effort command interface!");
      }
      if ((key == "tcp/twist.linear.x") || (key == "tcp/twist.linear.y") || (key == "tcp/twist.linear.z") ||
          (key == "tcp/twist.angular.x") || (key == "tcp/twist.angular.y") || (key == "tcp/twist.angular.z"))
      {
        start_modes_.emplace_back(hardware_interface::HW_IF_TWIST);
      }
    }
  }
  // pos-vel based controller requires (2 x actuator) interfaces
  // twist controller requires 6 interfaces
  // hand controller requires 1 interface
  if (!start_modes_.empty() && (start_modes_.size() != actuator_count_ * 2) && (start_modes_.size() != 6) &&
      (start_modes_.size() != 1))
  {
    return hardware_interface::return_type::ERROR;
  }

  // check for pos-vel based controller
  if (!start_modes_.empty() && (start_modes_.size() == 2 * actuator_count_) &&
      ((std::count(start_modes_.begin(), start_modes_.end(), hardware_interface::HW_IF_POSITION) != 6) ||
       (std::count(start_modes_.begin(), start_modes_.end(), hardware_interface::HW_IF_VELOCITY) != 6)))
  {
    return hardware_interface::return_type::ERROR;
  }

  // check for twist controller
  if (!start_modes_.empty() && (start_modes_.size() == 6) &&
      (std::count(start_modes_.begin(), start_modes_.end(), hardware_interface::HW_IF_TWIST) != 6))
  {
    return hardware_interface::return_type::ERROR;
  }

  // check for hand controller
  auto it = std::find_if(start_interfaces.begin(), start_interfaces.end(),
                         [this](const std::string& s) { return s.find(gripper_joint_name_) != std::string::npos; });
  if (!start_modes_.empty() && (start_modes_.size() == 1) && (it == start_interfaces.end()))
  {
    return hardware_interface::return_type::ERROR;
  }

  // Stopping interfaces
  // add stop interface per joint in tmp var for later check
  for (const auto& key : stop_interfaces)
  {
    RCLCPP_INFO(LOGGER, "Stopping '%s'", key.c_str());

    for (auto& joint : info_.joints)
    {
      if (key == joint.name + "/" + hardware_interface::HW_IF_POSITION && joint.name == gripper_joint_name_)
      {
        stop_modes_.push_back(StoppingInterface::STOP_GRIPPER);
        continue;
      }
      if (key == joint.name + "/" + hardware_interface::HW_IF_POSITION)
      {
        stop_modes_.push_back(StoppingInterface::STOP_POS_VEL);
      }
      if (key == joint.name + "/" + hardware_interface::HW_IF_VELOCITY)
      {
        stop_modes_.push_back(StoppingInterface::STOP_POS_VEL);
      }
      if (key == joint.name + "/" + hardware_interface::HW_IF_EFFORT)
      {
        continue;
        // not supporting effort command interface
        //              start_modes_.emplace_back(hardware_interface::HW_IF_EFFORT);
        RCLCPP_ERROR(LOGGER, "KortexMultiInterfaceHardware does not support effort command interface!");
      }
      if ((key == "tcp/twist.linear.x") || (key == "tcp/twist.linear.y") || (key == "tcp/twist.linear.z") ||
          (key == "tcp/twist.angular.x") || (key == "tcp/twist.angular.y") || (key == "tcp/twist.angular.z"))
      {
        stop_modes_.push_back(StoppingInterface::STOP_TWIST);
      }
    }

    // check if pos-vel based controller is stopping
    if (!stop_modes_.empty() && (stop_modes_.size() == 2 * actuator_count_) &&
        (std::count(stop_modes_.begin(), stop_modes_.end(), StoppingInterface::STOP_POS_VEL) != 2 * actuator_count_))
    {
      return hardware_interface::return_type::ERROR;
    }

    // check if twist controller is stopping
    if (!stop_modes_.empty() && (stop_modes_.size() == 6) &&
        (std::count(stop_modes_.begin(), stop_modes_.end(), StoppingInterface::STOP_TWIST) != 6))
    {
      return hardware_interface::return_type::ERROR;
    }
    // check if hand controller is stopping
    if (!stop_modes_.empty() && (stop_modes_.size() == 1) &&
        (std::count(stop_modes_.begin(), stop_modes_.end(), StoppingInterface::STOP_GRIPPER) != 1))
    {
      return hardware_interface::return_type::ERROR;
    }
  }
  RCLCPP_INFO(LOGGER, "prepare END");

  return ret_val;
  /*{

  // Prepare for new command modes
  std::vector<integration_lvl_t> new_modes = {};
  std::vector<std::size_t> new_mode_joint_index = {};
  RCLCPP_INFO(LOGGER, "Controller switch requested");
  for (std::string key: start_interfaces) {
      RCLCPP_DEBUG(LOGGER, "New command mode for joint: %s", key.c_str());
      for (std::size_t i = 0; i < info_.joints.size(); i++) {
          if (key == info_.joints[i].name + "/" + hardware_interface::HW_IF_POSITION) {
              new_modes.push_back(integration_lvl_t::POSITION);
              new_mode_joint_index.push_back(i);
          }
          if (key == info_.joints[i].name + "/" + hardware_interface::HW_IF_VELOCITY) {
              if (new_mode_joint_index.back() == i) {
                  new_modes.back() = integration_lvl_t::VELOCITY;
              } else {
                  new_modes.push_back(integration_lvl_t::VELOCITY);
                  new_mode_joint_index.push_back(i);
              }
          }
          if (key == info_.joints[i].name + "/" + hardware_interface::HW_IF_EFFORT) {
              new_modes.push_back(integration_lvl_t::EFFORT);
              new_mode_joint_index.push_back(i);
          }
      }
  }

  // Stop motion on all relevant joints that are stopping
  for (std::string key: stop_interfaces) {
      for (std::size_t i = 0; i < info_.joints.size(); i++) {
          if (key.find(info_.joints[i].name) != std::string::npos) {
              arm_commands_velocities_[i] = 0;
              arm_commands_efforts_[i] = 0;
              arm_joints_control_level_[i] = integration_lvl_t::UNDEFINED;  // Revert to undefined
          }
      }
  }

  // if we are sending twist messages to Kinova and our controller controller is changing we need to ensure the arm is
stopped! if (arm_mode_ == k_api::Base::ServoingMode::SINGLE_LEVEL_SERVOING && arm_joints_control_level_[6] ==
integration_lvl_t::UNDEFINED) {
      // block_write = true;
      // std::this_thread::sleep_for(
      //     std::chrono::milliseconds(50));  // sleep to ensure all outgoing write commands have finished
      arm_mode_ = k_api::Base::ServoingMode::UNSPECIFIED_SERVOING_MODE;
      RCLCPP_INFO(LOGGER, "Switching to NO_SERVOING_MODE");
      auto command = k_api::Base::TwistCommand();
      command.set_reference_frame(k_api::Common::CARTESIAN_REFERENCE_FRAME_TOOL);
      // command.set_duration = execute time (milliseconds) according to the api -> (not implemented yet)
      // see: https://github.com/Kinovarobotics/kortex/blob/master/api_cpp/doc/markdown/messages/Base/TwistCommand.md
      command.set_duration(0);

      auto twist = command.mutable_twist();
      twist->set_linear_x(0.0f);
      twist->set_linear_y(0.0f);
      twist->set_linear_z(0.0f);
      twist->set_angular_x(0.0f);
      twist->set_angular_y(0.0f);
      twist->set_angular_z(0.0f);
      base_.SendTwistCommand(command);
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
      // block_write = false;
  }

  // If we are not starting any controllers we are done
  if (new_modes.empty()) {
      block_write = false;
      return return_type::OK;
  }
  // Set the new command modes
  for (std::size_t i = 0; i < new_modes.size(); i++) {
      // if this interface is not free then we cant switch modes!
      if (arm_joints_control_level_[new_mode_joint_index[i]] != integration_lvl_t::UNDEFINED) {
          RCLCPP_ERROR(
                  LOGGER,
                  "Attempting to start interface that is already claimed. Joint mode index: %ld,
arm_joints_control_level_[%u]", new_mode_joint_index[i], arm_joints_control_level_[new_mode_joint_index[i]]);
block_write = false; return return_type::ERROR;
      }
      arm_joints_control_level_[new_mode_joint_index[i]] = new_modes[i];
      RCLCPP_DEBUG(LOGGER, "arm_joints_control_level_[%ld] mode: %u", new_mode_joint_index[i], new_modes[i]);
  }

  // TODO (marqrazz): NEED better way to detect that we are switching kinova servo mode!
  // Current hack is that if the arm is in integration_lvl_t::VELOCITY then we using the `joint_trajectory_controller`
  // if the arm is in integration_lvl_t::POSITION then we are using the `streaming_controller`
  if (arm_joints_control_level_[6] == integration_lvl_t::VELOCITY &&
      info_.joints[new_mode_joint_index.front()].name !=
      "finger_joint")  // TODO find a better way to identify gripper joint(s)
  {
      // block_write = true;
      // std::this_thread::sleep_for(
      //     std::chrono::milliseconds(50));  // sleep to ensure all outgoing write commands have finished
      auto servoing_mode = k_api::Base::ServoingModeInformation();
      // Set the base in low-level servoing mode
      arm_mode_ = k_api::Base::ServoingMode::LOW_LEVEL_SERVOING;
      servoing_mode.set_servoing_mode(k_api::Base::ServoingMode::LOW_LEVEL_SERVOING);
      base_.SetServoingMode(servoing_mode);
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
      feedback_ = base_cyclic_.RefreshFeedback();
      for (std::size_t i = 0; i < actuator_count_; i++) {
          arm_commands_positions_[i] =
                  KortexMathUtil::wrapRadiansFromMinusPiToPi(
                          KortexMathUtil::toRad(feedback_.actuators(i).position()));  // rad
          // RCLCPP_ERROR(LOGGER, "setting joint[%ld] position: %f", i, arm_commands_positions_[i]);
      }
      RCLCPP_INFO(LOGGER, "Switching to LOW_LEVEL_SERVOING");
      controller_switch_time_ = rclcpp::Clock().now();
      // block_write = false;
  } else if (arm_joints_control_level_[6] == integration_lvl_t::POSITION &&
             info_.joints[new_mode_joint_index.front()].name !=
             "finger_joint")  // TODO find a better way to identify gripper joint(s)
  {
      // block_write = true;
      // std::this_thread::sleep_for(
      //     std::chrono::milliseconds(50));  // sleep to ensure all outgoing write commands have finished
      auto servoing_mode = k_api::Base::ServoingModeInformation();
      // Set the base in low-level servoing mode
      arm_mode_ = k_api::Base::ServoingMode::SINGLE_LEVEL_SERVOING;
      servoing_mode.set_servoing_mode(k_api::Base::ServoingMode::SINGLE_LEVEL_SERVOING);
      base_.SetServoingMode(servoing_mode);
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
      feedback_ = base_cyclic_.RefreshFeedback();
      for (std::size_t i = 0; i < actuator_count_; i++) {
          arm_commands_positions_[i] = 0.0;  // this is a twist command at this point
          // RCLCPP_ERROR(LOGGER, "setting joint[%d] position: %f", i, arm_commands_positions_[i]);
      }
      RCLCPP_INFO(LOGGER, "Switching to SINGLE_LEVEL_SERVOING");
      controller_switch_time_ = rclcpp::Clock().now();
      // block_write = false;
  } else {
      RCLCPP_INFO(LOGGER, "Arm controller is not changing modes. arm_mode: %u", arm_mode_);
  }

  block_write = false;
} */
  return return_type::OK;
}

return_type KortexMultiInterfaceHardware::perform_command_mode_switch(const vector<std::string>& start_interfaces,
                                                                      const vector<std::string>& stop_interfaces)
{
  hardware_interface::return_type ret_val = hardware_interface::return_type::OK;

  RCLCPP_INFO(LOGGER, "perform START");

  // Starting interfaces
  // add start interface per joint in tmp var for later check
  for (const auto& key : start_interfaces)
  {
    RCLCPP_INFO(LOGGER, "Starting '%s'", key.c_str());
  }

  // Stopping interfaces
  // add stop interface per joint in tmp var for later check
  for (const auto& key : stop_interfaces)
  {
    RCLCPP_INFO(LOGGER, "Stopping '%s'", key.c_str());
  }
  RCLCPP_INFO(LOGGER, "perform END");

  return ret_val;
}

CallbackReturn KortexMultiInterfaceHardware::on_activate(const rclcpp_lifecycle::State& /* previous_state */)
{
  RCLCPP_INFO(LOGGER, "Activating KortexMultiInterfaceHardware...");
  base_.ClearFaults();

  // first read
  auto base_feedback = base_cyclic_.RefreshFeedback();

  // Add each actuator to the base_command_ and set the command to its current position
  for (std::size_t i = 0; i < actuator_count_; i++)
  {
    base_command_.add_actuators()->set_position(base_feedback.actuators(i).position());
  }

  // Initialize gripper
  float gripper_initial_position = base_feedback.interconnect().gripper_feedback().motor()[0].position();
  gripper_command_position_ = gripper_initial_position;
  // Initialize interconnect command to current gripper position.
  base_command_.mutable_interconnect()->mutable_command_id()->set_identifier(0);
  gripper_motor_command_ = base_command_.mutable_interconnect()->mutable_gripper_command()->add_motor_cmd();
  gripper_motor_command_->set_position(gripper_initial_position);  // % position
  gripper_motor_command_->set_velocity(100.0);                     // % speed
  gripper_motor_command_->set_force(100.0);                        // % torque

  // Send a first frame
  base_feedback = base_cyclic_.Refresh(base_command_);
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

  RCLCPP_INFO(LOGGER, "KortexMultiInterfaceHardware successfully activated!");
  return CallbackReturn::SUCCESS;
}

CallbackReturn KortexMultiInterfaceHardware::on_deactivate(const rclcpp_lifecycle::State& /* previous_state */)
{
  RCLCPP_INFO(LOGGER, "Deactivating KortexMultiInterfaceHardware...");

  auto servoing_mode = k_api::Base::ServoingModeInformation();
  // Set back the servoing mode to Single Level Servoing
  servoing_mode.set_servoing_mode(k_api::Base::ServoingMode::SINGLE_LEVEL_SERVOING);
  base_.SetServoingMode(servoing_mode);

  // Close API session
  session_manager_.CloseSession();
  session_manager_real_time_.CloseSession();

  // Deactivate the router and cleanly disconnect from the transport object
  router_tcp_.SetActivationStatus(false);
  transport_tcp_.disconnect();
  router_udp_realtime_.SetActivationStatus(false);
  transport_udp_realtime_.disconnect();

  RCLCPP_INFO(LOGGER, "KortexMultiInterfaceHardware successfully deactivated!");

  return CallbackReturn::SUCCESS;
}

return_type KortexMultiInterfaceHardware::read()
{
  if (first_pass_)
  {
    feedback_ = base_cyclic_.RefreshFeedback();
    first_pass_ = false;
  }

  // read gripper state
  readGripperPosition();

  for (std::size_t i = 0; i < actuator_count_; i++)
  {
    if (arm_joints_control_level_[i] == integration_lvl_t::UNDEFINED)
    {
      return return_type::OK;
    }
    // read torque
    arm_efforts_[i] = feedback_.actuators(i).torque();  // N*m
    // read velocity
    arm_velocities_[i] = KortexMathUtil::toRad(feedback_.actuators(i).velocity());  // rad/sec
    // read position
    num_turns_tmp_ = 0;
    arm_positions_[i] = KortexMathUtil::wrapRadiansFromMinusPiToPi(
        KortexMathUtil::toRad(feedback_.actuators(i).position()), num_turns_tmp_);  // rad
  }

  return return_type::OK;
}

void KortexMultiInterfaceHardware::readGripperPosition()
{
  // max joint angle = 0.81 for robotiq_2f_85
  // TODO read in as paramter from kortex_controllers.yaml
  gripper_position_ = feedback_.interconnect().gripper_feedback().motor()[0].position() / 100.0 * 0.81;  // rad
}

return_type KortexMultiInterfaceHardware::write()
{
  if (block_write)
  {
    feedback_ = base_cyclic_.RefreshFeedback();
    return return_type::OK;
  }

  if (arm_mode_ == k_api::Base::ServoingMode::SINGLE_LEVEL_SERVOING && twist_controller_running_)
  {
    // Twist controller active
    RCLCPP_INFO(LOGGER, "Twist controller active!");

    // twist control
    sendTwistCommand();

    // gripper control
    sendGripperCommand(arm_mode_, gripper_command_position_);
  }
  else if (joint_based_controller_running_ && (arm_mode_ == k_api::Base::ServoingMode::LOW_LEVEL_SERVOING) &&
           (feedback_.base().active_state() == k_api::Common::ARMSTATE_SERVOING_LOW_LEVEL))
  {
    // Per joint controller active
    RCLCPP_INFO(LOGGER, "Joint controller active!");

    // gripper control
    sendGripperCommand(arm_mode_, gripper_command_position_);

    // send commands to the joints
    sendJointCommands();
  }
  else if ((!joint_based_controller_running_ && !twist_controller_running_) ||
           arm_mode_ != k_api::Base::ServoingMode::LOW_LEVEL_SERVOING ||
           feedback_.base().active_state() != k_api::Common::ARMSTATE_SERVOING_LOW_LEVEL)
  {
    // Keep alive mode - no controller active
    RCLCPP_DEBUG(LOGGER, "No controller active!");
  }

  // read one step late because reading before sending commands
  // generates errors
  feedback_ = base_cyclic_.RefreshFeedback();
  return return_type::OK;
}

void KortexMultiInterfaceHardware::prepareCommands()
{  // update the command for each joint
  for (size_t i = 0; i < actuator_count_; i++)
  {
    // set command per joint
    cmd_degrees_tmp_ =
        static_cast<float>(KortexMathUtil::wrapDegreesFromZeroTo360(KortexMathUtil::toDeg(arm_commands_positions_[i])));
    cmd_vel_tmp_ = static_cast<float>(KortexMathUtil::toDeg(arm_commands_velocities_[i]));

    base_command_.mutable_actuators(static_cast<int>(i))->set_position(cmd_degrees_tmp_);
    // Velocity command interface not implemented properly in the kortex api
    // base_command_.mutable_actuators(i)->set_velocity(cmd_vel_tmp_);
    base_command_.mutable_actuators(static_cast<int>(i))->set_command_id(base_command_.frame_id());
  }
}

void KortexMultiInterfaceHardware::sendJointCommands()
{
  // identifier++
  incrementId();

  prepareCommands();

  // send the command to the robot
  try
  {
    feedback_ = base_cyclic_.Refresh(base_command_);
  }
  catch (k_api::KDetailedException& ex)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Kortex exception: " << ex.what());

    RCLCPP_ERROR_STREAM(LOGGER, "Error sub-code: " << k_api::SubErrorCodes_Name(
                                    k_api::SubErrorCodes((ex.getErrorInfo().getError().error_sub_code()))));

    // attempt to clear any robot faults
    base_.ClearFaults();
    feedback_ = base_cyclic_.RefreshFeedback();
    RCLCPP_WARN(LOGGER, "Attempting to clear faults. [base_active_state: %u]", feedback_.base().active_state());
  }
}

void KortexMultiInterfaceHardware::incrementId()
{
  // Incrementing identifier ensures actuators can reject out of time frames
  base_command_.set_frame_id(base_command_.frame_id() + 1);
  if (base_command_.frame_id() > 65535)
    base_command_.set_frame_id(0);
}

void KortexMultiInterfaceHardware::sendGripperCommand(k_api::Base::ServoingMode arm_mode, double position,
                                                      double velocity, double force)
{
  if (arm_mode == k_api::Base::ServoingMode::SINGLE_LEVEL_SERVOING)
  {
    k_api::Base::GripperCommand gripper_command;
    gripper_command.set_mode(k_api::Base::GRIPPER_POSITION);
    auto finger = gripper_command.mutable_gripper()->add_finger();
    finger->set_finger_identifier(1);
    ;
    finger->set_value(static_cast<float>(position / 100.0));  // This values needs to be between 0 and 1
    base_.SendGripperCommand(gripper_command);
  }
  else if (arm_mode == k_api::Base::ServoingMode::LOW_LEVEL_SERVOING)
  {
    // % open/closed, this values needs to be between 0 and 1
    gripper_motor_command_->set_position(static_cast<float>(position));
    // % speed TODO read in as paramter from kortex_controllers.yaml
    gripper_motor_command_->set_velocity(static_cast<float>(velocity));
    // % torque TODO read in as paramter from kortex_controllers.yaml
    gripper_motor_command_->set_force(static_cast<float>(force));
  }
}

void KortexMultiInterfaceHardware::sendTwistCommand()
{
  auto command = k_api::Base::TwistCommand();
  command.set_reference_frame(k_api::Common::CARTESIAN_REFERENCE_FRAME_TOOL);
  // command.set_duration = execute time (milliseconds) according to the api -> (not implemented yet)
  // see: https://github.com/Kinovarobotics/kortex/blob/master/api_cpp/doc/markdown/messages/Base/TwistCommand.md
  command.set_duration(0);

  auto twist = command.mutable_twist();
  twist->set_linear_x(float(twist_commands_[0]));
  twist->set_linear_y(float(twist_commands_[1]));
  twist->set_linear_z(float(twist_commands_[2]));
  twist->set_angular_x(float(twist_commands_[3]));
  twist->set_angular_y(float(twist_commands_[4]));
  twist->set_angular_z(float(twist_commands_[5]));
  base_.SendTwistCommand(command);
}

void KortexMultiInterfaceHardware::sendJointCommand()
{
  prepareCommands();
  sendJointCommands();
}

}  // namespace kortex2_driver

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(kortex2_driver::KortexMultiInterfaceHardware, hardware_interface::SystemInterface)
