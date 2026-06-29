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
 *
 * Uses the Kinova USB C API (libkinovadrv.so) loaded via dlopen — the same
 * library that powers kinova-ros.  No Kortex gRPC API is used here at all;
 * this class is a completely independent plugin that happens to live in the
 * same kortex_driver shared library.
 *
 * Supports: j2s7s300 (7-DOF spherical wrist, 3 fingers).
 * Gripper is NOT controlled here — the Robotiq is on a serial port and is
 * handled by a separate ros2_control hardware interface (ros2_robotiq_gripper).
 *
 */
//----------------------------------------------------------------------
#ifndef KORTEX_DRIVER__GEN2_HARDWARE_INTERFACE_HPP_
#define KORTEX_DRIVER__GEN2_HARDWARE_INTERFACE_HPP_

#pragma once

#include <dlfcn.h>

#include <atomic>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"

#include "kortex_driver/visibility_control.h"

// Gen2 C API types — from kinova-ros
// We only need KinovaTypes.h for the struct definitions (AngularPosition,
// TrajectoryPoint, etc.).  The actual functions are loaded at runtime via
// dlopen so we never link directly against libkinovadrv.so at compile time,
// which keeps the Gen3 build path completely unaffected.
#include "KinovaTypes.h"

using hardware_interface::return_type;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace kortex_driver
{

class Gen2HardwareInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(Gen2HardwareInterface)

  KORTEX_DRIVER_PUBLIC
  CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) final;

  KORTEX_DRIVER_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() final;

  KORTEX_DRIVER_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() final;

  KORTEX_DRIVER_PUBLIC
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) final;

  KORTEX_DRIVER_PUBLIC
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) final;

  KORTEX_DRIVER_PUBLIC
  return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) final;

  KORTEX_DRIVER_PUBLIC
  return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) final;

private:
  // ── Gen2 C API function pointers (resolved via dlopen in on_init) ────────
  // All functions return a DWORD (uint32_t) error code; 1 = SUCCESS.
  // Signatures come from kinova-ros/kinova_driver/include/kinova/
  //   Kinova.API.USBCommandLayerUbuntu.h
  void * api_handle_{nullptr};

  using Fn_InitAPI              = int (*)();
  using Fn_CloseAPI             = int (*)();
  using Fn_StartControlAPI      = int (*)();
  using Fn_StopControlAPI       = int (*)();
  using Fn_SetAngularControl    = int (*)();
  using Fn_GetAngularPosition   = int (*)(AngularPosition &);
  using Fn_GetAngularVelocity   = int (*)(AngularPosition &);
  using Fn_GetAngularForce      = int (*)(AngularPosition &);
  using Fn_SendAdvanceTrajectory = int (*)(TrajectoryPoint);
  using Fn_EraseAllTrajectories = int (*)();

  Fn_InitAPI               InitAPI_{nullptr};
  Fn_CloseAPI              CloseAPI_{nullptr};
  Fn_StartControlAPI       StartControlAPI_{nullptr};
  Fn_StopControlAPI        StopControlAPI_{nullptr};
  Fn_SetAngularControl     SetAngularControl_{nullptr};
  Fn_GetAngularPosition    GetAngularPosition_{nullptr};
  Fn_GetAngularVelocity    GetAngularVelocity_{nullptr};
  Fn_GetAngularForce       GetAngularForce_{nullptr};
  Fn_SendAdvanceTrajectory SendAdvanceTrajectory_{nullptr};
  Fn_EraseAllTrajectories  EraseAllTrajectories_{nullptr};

  // ── State & command storage ───────────────────────────────────────────────
  // Sized to num_arm_joints_ in on_init.
  // All values in RADIANS (converted at read/write boundaries).
  std::size_t num_arm_joints_{0};

  std::vector<double> arm_positions_;           // rad — state
  std::vector<double> arm_velocities_;          // rad/s — state
  std::vector<double> arm_efforts_;             // N·m — state

  std::vector<double> arm_commands_positions_;  // rad — command
  std::vector<double> arm_commands_velocities_; // rad/s — command (not yet used)

  // ── Configuration ─────────────────────────────────────────────────────────
  // Path to libkinovadrv.so, configurable via URDF <param>.
  // Defaults to the installed location from kinova-ros.
  std::string api_lib_path_;

  // robot_type string (e.g. "j2s7s300") — informational / sanity check only
  std::string robot_type_;

  // ── Runtime helpers ───────────────────────────────────────────────────────
  bool api_initialized_{false};

  // Protects write() from running while a controller switch is in progress.
  // Mirrors the same pattern used in KortexMultiInterfaceHardware.
  std::atomic<bool> block_write_{false};

  // Tracks whether this is the very first read() call so we can seed commands
  // from current position (prevents a lurch on controller startup).
  bool first_pass_{true};

  // ── Degree / radian helpers ───────────────────────────────────────────────
  static constexpr double DEG_TO_RAD = M_PI / 180.0;
  static constexpr double RAD_TO_DEG = 180.0 / M_PI;

  // Wrap degrees reported by the arm into (-180, 180] to match kinova-ros
  // convention (same logic as KortexMathUtil::wrapRadiansFromMinusPiToPi but
  // applied in degree-space before converting).
  static double wrapDeg(double deg)
  {
    while (deg >  180.0) deg -= 360.0;
    while (deg <= -180.0) deg += 360.0;
    return deg;
  }

  // ── Gen2 API load helpers ─────────────────────────────────────────────────
  bool loadApiLibrary();

  template <typename FnPtr>
  bool resolveSymbol(FnPtr & fn, const char * symbol_name);

  // ── Private helpers called from write() ──────────────────────────────────
  void sendJointPositionCommands();
};

}  // namespace kortex_driver

#endif  // KORTEX_DRIVER__GEN2_HARDWARE_INTERFACE_HPP_