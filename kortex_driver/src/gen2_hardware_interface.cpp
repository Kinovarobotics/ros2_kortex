// Copyright 2024, AABL Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

//----------------------------------------------------------------------
/*!\file
 *
 * \brief ros2_control hardware interface for Kinova Gen2 (Jaco2/Mico) arms.
 *        See gen2_hardware_interface.hpp for design notes.
 *
 */
//----------------------------------------------------------------------

#include <dlfcn.h>

#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include "kortex_driver/gen2_hardware_interface.hpp"
#include "kortex_driver/kortex_math_util.hpp"  // reuse toRad / toDeg helpers

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace
{
// File-scope logger — same pattern as KortexMultiInterfaceHardware.
const rclcpp::Logger LOGGER = rclcpp::get_logger("Gen2HardwareInterface");

// Default path to the Gen2 USB API shared library installed by kinova-ros.
// Users can override via <param name="api_lib_path"> in the URDF.
constexpr char DEFAULT_API_LIB[] = "Kinova.API.USBCommandLayerUbuntu.so";

// Gen2 API success code
constexpr int GEN2_API_SUCCESS = 1;
}  // namespace

namespace kortex_driver
{

// ─────────────────────────────────────────────────────────────────────────────
//  on_init
// ─────────────────────────────────────────────────────────────────────────────
CallbackReturn Gen2HardwareInterface::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  RCLCPP_INFO(LOGGER, "Initializing Gen2HardwareInterface");

  if (hardware_interface::SystemInterface::on_init(params) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  info_ = params.hardware_info;

  // ── Read URDF hardware parameters ────────────────────────────────────────

  // robot_type — informational; logged so it shows up in driver output
  robot_type_ = info_.hardware_parameters.count("robot_type")
    ? info_.hardware_parameters.at("robot_type")
    : "j2s7s300";
  RCLCPP_INFO(LOGGER, "robot_type: '%s'", robot_type_.c_str());

  // api_lib_path — allow override for non-standard install locations
  api_lib_path_ = info_.hardware_parameters.count("api_lib_path")
    ? info_.hardware_parameters.at("api_lib_path")
    : DEFAULT_API_LIB;
  RCLCPP_INFO(LOGGER, "api_lib_path: '%s'", api_lib_path_.c_str());

  // ── Count arm joints (everything declared in <ros2_control>) ────────────
  // The Robotiq gripper is handled by a separate hardware interface, so every
  // joint listed here is an arm joint.
  num_arm_joints_ = info_.joints.size();
  RCLCPP_INFO(LOGGER, "Number of arm joints: %zu", num_arm_joints_);

  if (num_arm_joints_ == 0) {
    RCLCPP_FATAL(LOGGER, "No joints found in hardware info!");
    return CallbackReturn::ERROR;
  }

  // ── Validate joint interface declarations ────────────────────────────────
  for (const auto & joint : info_.joints) {
    // Require at least one position command interface
    bool has_pos_cmd = false;
    for (const auto & ci : joint.command_interfaces) {
      if (ci.name == hardware_interface::HW_IF_POSITION) {
        has_pos_cmd = true;
      } else if (
        ci.name != hardware_interface::HW_IF_VELOCITY &&
        ci.name != hardware_interface::HW_IF_EFFORT)
      {
        RCLCPP_WARN(
          LOGGER, "Joint '%s' has unexpected command interface '%s' — ignoring.",
          joint.name.c_str(), ci.name.c_str());
      }
    }
    if (!has_pos_cmd) {
      RCLCPP_FATAL(
        LOGGER, "Joint '%s' is missing required 'position' command interface.",
        joint.name.c_str());
      return CallbackReturn::ERROR;
    }

    // Require position state interface
    bool has_pos_state = false;
    for (const auto & si : joint.state_interfaces) {
      if (si.name == hardware_interface::HW_IF_POSITION) {
        has_pos_state = true;
      }
    }
    if (!has_pos_state) {
      RCLCPP_FATAL(
        LOGGER, "Joint '%s' is missing required 'position' state interface.",
        joint.name.c_str());
      return CallbackReturn::ERROR;
    }
  }

  // ── Allocate storage ─────────────────────────────────────────────────────
  arm_positions_.assign(num_arm_joints_, std::numeric_limits<double>::quiet_NaN());
  arm_velocities_.assign(num_arm_joints_, std::numeric_limits<double>::quiet_NaN());
  arm_efforts_.assign(num_arm_joints_, std::numeric_limits<double>::quiet_NaN());
  arm_commands_positions_.assign(num_arm_joints_, std::numeric_limits<double>::quiet_NaN());
  arm_commands_velocities_.assign(num_arm_joints_, 0.0);

  // ── Load libkinovadrv.so and resolve function pointers ───────────────────
  if (!loadApiLibrary()) {
    return CallbackReturn::ERROR;
  }

  RCLCPP_INFO(LOGGER, "Gen2HardwareInterface initialized successfully.");
  return CallbackReturn::SUCCESS;
}

// ─────────────────────────────────────────────────────────────────────────────
//  export_state_interfaces
// ─────────────────────────────────────────────────────────────────────────────
std::vector<hardware_interface::StateInterface>
Gen2HardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  for (std::size_t i = 0; i < num_arm_joints_; ++i) {
    const std::string & name = info_.joints[i].name;
    RCLCPP_DEBUG(LOGGER, "Exporting state interfaces for joint: %s", name.c_str());

    state_interfaces.emplace_back(
      hardware_interface::StateInterface(name, hardware_interface::HW_IF_POSITION,
        &arm_positions_[i]));
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(name, hardware_interface::HW_IF_VELOCITY,
        &arm_velocities_[i]));
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(name, hardware_interface::HW_IF_EFFORT,
        &arm_efforts_[i]));
  }

  return state_interfaces;
}

// ─────────────────────────────────────────────────────────────────────────────
//  export_command_interfaces
// ─────────────────────────────────────────────────────────────────────────────
std::vector<hardware_interface::CommandInterface>
Gen2HardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  for (std::size_t i = 0; i < num_arm_joints_; ++i) {
    const std::string & name = info_.joints[i].name;

    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(name, hardware_interface::HW_IF_POSITION,
        &arm_commands_positions_[i]));
    // Velocity command exported for controller compatibility; the Gen2 API
    // accepts velocity trajectories but we start with position-only control.
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(name, hardware_interface::HW_IF_VELOCITY,
        &arm_commands_velocities_[i]));
  }

  return command_interfaces;
}

// ─────────────────────────────────────────────────────────────────────────────
//  on_activate
// ─────────────────────────────────────────────────────────────────────────────
CallbackReturn Gen2HardwareInterface::on_activate(
  const rclcpp_lifecycle::State & /* previous_state */)
{
  RCLCPP_INFO(LOGGER, "Activating Gen2HardwareInterface...");

  // ── Initialize the USB API ───────────────────────────────────────────────
  int result = InitAPI_();
  if (result != GEN2_API_SUCCESS) {
    RCLCPP_FATAL(LOGGER, "InitAPI() failed with error code %d. "
      "Check USB connection and udev rules.", result);
    return CallbackReturn::ERROR;
  }
  RCLCPP_INFO(LOGGER, "Gen2 USB API initialized (InitAPI returned %d).", result);

  // Give the API a moment to enumerate the device (mirrors kinova_api.cpp)
  using namespace std::chrono_literals;
  std::this_thread::sleep_for(2000ms);

  // Take control of the arm away from the joystick
  result = StartControlAPI_();
  if (result != GEN2_API_SUCCESS) {
    RCLCPP_WARN(LOGGER, "StartControlAPI() returned %d (non-fatal, arm may already be under "
      "API control).", result);
  }

  // Switch to angular (joint) control mode
  result = SetAngularControl_();
  if (result != GEN2_API_SUCCESS) {
    RCLCPP_FATAL(LOGGER, "SetAngularControl() failed with error code %d.", result);
    CloseAPI_();
    return CallbackReturn::ERROR;
  }

  api_initialized_ = true;

  // ── Seed state and commands from current arm position ───────────────────
  AngularPosition current_pos;
  result = GetAngularPosition_(current_pos);
  if (result != GEN2_API_SUCCESS) {
    RCLCPP_FATAL(LOGGER, "GetAngularPosition() failed with error code %d.", result);
    CloseAPI_();
    api_initialized_ = false;
    return CallbackReturn::ERROR;
  }

  // AngularPosition.Actuators is a struct with fields Actuator1..Actuator7.
  // We pull them out via a helper array to avoid 7 separate if-branches.
  const float raw_deg[7] = {
    current_pos.Actuators.Actuator1,
    current_pos.Actuators.Actuator2,
    current_pos.Actuators.Actuator3,
    current_pos.Actuators.Actuator4,
    current_pos.Actuators.Actuator5,
    current_pos.Actuators.Actuator6,
    current_pos.Actuators.Actuator7,
  };

  for (std::size_t i = 0; i < num_arm_joints_; ++i) {
    double rad = wrapDeg(static_cast<double>(raw_deg[i])) * DEG_TO_RAD;
    arm_positions_[i]          = rad;
    arm_velocities_[i]         = 0.0;
    arm_efforts_[i]            = 0.0;
    // Seed commands to current position so the arm doesn't move on startup
    arm_commands_positions_[i] = rad;
    arm_commands_velocities_[i] = 0.0;
  }

  first_pass_ = true;
  block_write_ = false;

  RCLCPP_INFO(LOGGER, "Gen2HardwareInterface activated. Initial joint positions (deg):");
  for (std::size_t i = 0; i < num_arm_joints_; ++i) {
    RCLCPP_INFO(LOGGER, "  %s: %.2f deg",
      info_.joints[i].name.c_str(),
      arm_positions_[i] * RAD_TO_DEG);
  }

  return CallbackReturn::SUCCESS;
}

// ─────────────────────────────────────────────────────────────────────────────
//  on_deactivate
// ─────────────────────────────────────────────────────────────────────────────
CallbackReturn Gen2HardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /* previous_state */)
{
  RCLCPP_INFO(LOGGER, "Deactivating Gen2HardwareInterface...");

  block_write_ = true;

  if (api_initialized_) {
    // Clear any pending trajectory so the arm stops cleanly
    EraseAllTrajectories_();
    // Release joystick / API control
    StopControlAPI_();
    CloseAPI_();
    api_initialized_ = false;
    RCLCPP_INFO(LOGGER, "Gen2 USB API closed.");
  }

  return CallbackReturn::SUCCESS;
}

// ─────────────────────────────────────────────────────────────────────────────
//  read
// ─────────────────────────────────────────────────────────────────────────────
return_type Gen2HardwareInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!api_initialized_) {
    return return_type::OK;
  }

  // ── Positions ────────────────────────────────────────────────────────────
  AngularPosition pos;
  if (GetAngularPosition_(pos) == GEN2_API_SUCCESS) {
    const float raw[7] = {
      pos.Actuators.Actuator1, pos.Actuators.Actuator2, pos.Actuators.Actuator3,
      pos.Actuators.Actuator4, pos.Actuators.Actuator5, pos.Actuators.Actuator6,
      pos.Actuators.Actuator7,
    };
    for (std::size_t i = 0; i < num_arm_joints_; ++i) {
      arm_positions_[i] = wrapDeg(static_cast<double>(raw[i])) * DEG_TO_RAD;
    }
  } else {
    RCLCPP_WARN_THROTTLE(LOGGER, *rclcpp::Clock::make_shared(), 1000,
      "GetAngularPosition() failed — keeping last known position.");
  }

  // ── Velocities ───────────────────────────────────────────────────────────
  AngularPosition vel;
  if (GetAngularVelocity_(vel) == GEN2_API_SUCCESS) {
    const float raw[7] = {
      vel.Actuators.Actuator1, vel.Actuators.Actuator2, vel.Actuators.Actuator3,
      vel.Actuators.Actuator4, vel.Actuators.Actuator5, vel.Actuators.Actuator6,
      vel.Actuators.Actuator7,
    };
    for (std::size_t i = 0; i < num_arm_joints_; ++i) {
      arm_velocities_[i] = static_cast<double>(raw[i]) * DEG_TO_RAD;
    }
  }

  // ── Efforts (joint torques) ───────────────────────────────────────────────
  AngularPosition force;
  if (GetAngularForce_(force) == GEN2_API_SUCCESS) {
    const float raw[7] = {
      force.Actuators.Actuator1, force.Actuators.Actuator2, force.Actuators.Actuator3,
      force.Actuators.Actuator4, force.Actuators.Actuator5, force.Actuators.Actuator6,
      force.Actuators.Actuator7,
    };
    for (std::size_t i = 0; i < num_arm_joints_; ++i) {
      arm_efforts_[i] = static_cast<double>(raw[i]);  // already in N·m
    }
  }

  // On the very first read, seed commands from actual position so the
  // joint_trajectory_controller doesn't lurch when it first activates.
  if (first_pass_) {
    first_pass_ = false;
    for (std::size_t i = 0; i < num_arm_joints_; ++i) {
      if (std::isnan(arm_commands_positions_[i])) {
        arm_commands_positions_[i] = arm_positions_[i];
      }
    }
  }

  return return_type::OK;
}

// ─────────────────────────────────────────────────────────────────────────────
//  write
// ─────────────────────────────────────────────────────────────────────────────
return_type Gen2HardwareInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (block_write_ || !api_initialized_) {
    return return_type::OK;
  }

  // Sanity: skip write if any command is still NaN (controller not yet ready)
  for (std::size_t i = 0; i < num_arm_joints_; ++i) {
    if (std::isnan(arm_commands_positions_[i])) {
      return return_type::OK;
    }
  }

  sendJointPositionCommands();
  return return_type::OK;
}

// ─────────────────────────────────────────────────────────────────────────────
//  sendJointPositionCommands  (private)
// ─────────────────────────────────────────────────────────────────────────────
void Gen2HardwareInterface::sendJointPositionCommands()
{
  // Build a TrajectoryPoint with ANGULAR_POSITION type.
  // As noted in the Gen2 API docs: "always memset to 0 first; there are no
  // default values — random bytes will be sent to the arm otherwise."
  TrajectoryPoint point;
  memset(&point, 0, sizeof(point));

  point.InitStruct();  // zero-fills via the struct's own helper if present;
                       // the memset above is the belt-and-suspenders guard

  point.Position.Type = ANGULAR_POSITION;

  // Convert radians → degrees, then wrap to [0, 360) as the Gen2 API expects.
  // kinova-ros uses wrapDegreesFromZeroTo360 for commands (not wrapDeg which
  // gives [-180, 180]).  We replicate that here.
  const double cmd_deg[7] = {
    KortexMathUtil::wrapDegreesFromZeroTo360(arm_commands_positions_[0] * RAD_TO_DEG),
    KortexMathUtil::wrapDegreesFromZeroTo360(arm_commands_positions_[1] * RAD_TO_DEG),
    KortexMathUtil::wrapDegreesFromZeroTo360(arm_commands_positions_[2] * RAD_TO_DEG),
    KortexMathUtil::wrapDegreesFromZeroTo360(arm_commands_positions_[3] * RAD_TO_DEG),
    KortexMathUtil::wrapDegreesFromZeroTo360(arm_commands_positions_[4] * RAD_TO_DEG),
    KortexMathUtil::wrapDegreesFromZeroTo360(arm_commands_positions_[5] * RAD_TO_DEG),
    KortexMathUtil::wrapDegreesFromZeroTo360(arm_commands_positions_[6] * RAD_TO_DEG),
  };

  point.Position.Actuators.Actuator1 = static_cast<float>(cmd_deg[0]);
  point.Position.Actuators.Actuator2 = static_cast<float>(cmd_deg[1]);
  point.Position.Actuators.Actuator3 = static_cast<float>(cmd_deg[2]);
  point.Position.Actuators.Actuator4 = static_cast<float>(cmd_deg[3]);
  point.Position.Actuators.Actuator5 = static_cast<float>(cmd_deg[4]);
  point.Position.Actuators.Actuator6 = static_cast<float>(cmd_deg[5]);
  point.Position.Actuators.Actuator7 = static_cast<float>(cmd_deg[6]);

  // Fingers: we control them via a separate hardware interface, but the Gen2
  // API still requires finger fields to be set.  Leave them at 0 (fully open)
  // when no gripper is under our control.
  point.Position.Fingers.Finger1 = 0.0f;
  point.Position.Fingers.Finger2 = 0.0f;
  point.Position.Fingers.Finger3 = 0.0f;

  int result = SendAdvanceTrajectory_(point);
  if (result != GEN2_API_SUCCESS) {
    RCLCPP_WARN_THROTTLE(LOGGER, *rclcpp::Clock::make_shared(), 1000,
      "SendAdvanceTrajectory() returned %d.", result);
  }
}

// ─────────────────────────────────────────────────────────────────────────────
//  loadApiLibrary  (private)
// ─────────────────────────────────────────────────────────────────────────────
bool Gen2HardwareInterface::loadApiLibrary()
{
  // RTLD_NOW  — resolve all symbols immediately so we catch missing exports
  //             at load time rather than at the first API call.
  // RTLD_GLOBAL — make the library's symbols visible to other loaded libs
  //               (required by libkinovadrv.so's internal structure).
  api_handle_ = dlopen(api_lib_path_.c_str(), RTLD_NOW | RTLD_GLOBAL);
  if (!api_handle_) {
    RCLCPP_FATAL(LOGGER,
      "Failed to open Gen2 API library '%s': %s\n"
      "  → Is kinova-ros installed and libkinovadrv.so present?\n"
      "  → Try: sudo ldconfig && ls %s",
      api_lib_path_.c_str(), dlerror(), api_lib_path_.c_str());
    return false;
  }
  RCLCPP_INFO(LOGGER, "Loaded Gen2 API library: %s", api_lib_path_.c_str());

  // Resolve all required function pointers.  If any are missing the .so is
  // corrupt or the wrong version.
  bool ok = true;
  ok &= resolveSymbol(InitAPI_,               "InitAPI");
  ok &= resolveSymbol(CloseAPI_,              "CloseAPI");
  ok &= resolveSymbol(StartControlAPI_,       "StartControlAPI");
  ok &= resolveSymbol(StopControlAPI_,        "StopControlAPI");
  ok &= resolveSymbol(SetAngularControl_,     "SetAngularControl");
  ok &= resolveSymbol(GetAngularPosition_,    "GetAngularPosition");
  ok &= resolveSymbol(GetAngularVelocity_,    "GetAngularVelocity");
  ok &= resolveSymbol(GetAngularForce_,       "GetAngularForce");
  ok &= resolveSymbol(SendAdvanceTrajectory_, "SendAdvanceTrajectory");
  ok &= resolveSymbol(EraseAllTrajectories_,  "EraseAllTrajectories");

  if (!ok) {
    dlclose(api_handle_);
    api_handle_ = nullptr;
    return false;
  }

  return true;
}

// ─────────────────────────────────────────────────────────────────────────────
//  resolveSymbol  (private template)
// ─────────────────────────────────────────────────────────────────────────────
template <typename FnPtr>
bool Gen2HardwareInterface::resolveSymbol(FnPtr & fn, const char * symbol_name)
{
  dlerror();  // clear any previous error
  void * sym = dlsym(api_handle_, symbol_name);
  const char * err = dlerror();
  if (err || !sym) {
    RCLCPP_FATAL(LOGGER, "Failed to resolve symbol '%s' from Gen2 API library: %s",
      symbol_name, err ? err : "(unknown)");
    return false;
  }
  // Strict-aliasing-safe cast via memcpy
  memcpy(&fn, &sym, sizeof(FnPtr));
  return true;
}

}  // namespace kortex_driver

// ─────────────────────────────────────────────────────────────────────────────
//  Plugin registration
// ─────────────────────────────────────────────────────────────────────────────
#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  kortex_driver::Gen2HardwareInterface, hardware_interface::SystemInterface)