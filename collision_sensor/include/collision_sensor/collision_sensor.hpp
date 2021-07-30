// Copyright 2021 PAL Robotics SL.
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

/*
 * Author: Andy Zelenak
 */

#ifndef COLLISION_SENSOR__COLLISION_SENSOR_HPP_
#define COLLISION_SENSOR__COLLISION_SENSOR_HPP_

#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "collision_sensor/visibility_control.h"
#include "collision_sensor/contact_detection/contact_detection.hpp"
#include "semantic_components/imu_sensor.hpp"
#include "std_msgs/msg/bool.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_publisher.h"

namespace collision_sensor
{
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class CollisionSensor : public controller_interface::ControllerInterface
{
public:
  COLLISION_SENSOR_PUBLIC
  controller_interface::return_type init(const std::string& controller_name) override;

  COLLISION_SENSOR_PUBLIC
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  COLLISION_SENSOR_PUBLIC
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  COLLISION_SENSOR_PUBLIC
  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;

  COLLISION_SENSOR_PUBLIC
  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

  COLLISION_SENSOR_PUBLIC
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

  COLLISION_SENSOR_PUBLIC
  controller_interface::return_type update() override;

protected:
  std::vector<std::string> joint_names_;
  std::vector<std::string> state_interface_types_;
  double torque_threshold_newton_meters_;
  int consecutive_outliers_to_trigger_;
  size_t num_dof_;
  // This object handles the contact detection calculations
  std::shared_ptr<ContactDetection> contact_monitor_;
  std::atomic<bool> is_active_;

  // Publish true when contact is detected
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr contact_publisher_;
  using BoolPublisher = realtime_tools::RealtimePublisher<std_msgs::msg::Bool>;
  std::unique_ptr<BoolPublisher> realtime_publisher_;
};

}  // namespace collision_sensor

#endif  // COLLISION_SENSOR__COLLISION_SENSOR_HPP_
