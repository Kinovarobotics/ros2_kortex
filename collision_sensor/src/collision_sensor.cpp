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

#include "collision_sensor/collision_sensor.hpp"

#include <memory>
#include <string>

namespace collision_sensor
{
controller_interface::return_type CollisionSensor::init(const std::string& controller_name)
{
  auto ret = ControllerInterface::init(controller_name);
  if (ret != controller_interface::return_type::OK)
  {
    return ret;
  }

  node_->declare_parameter<std::vector<std::string>>("joints", joint_names_);
  node_->declare_parameter<std::vector<std::string>>("state_interfaces", state_interface_types_);
  node_->declare_parameter<double>("torque_threshold_newton_meters", torque_threshold_newton_meters_);
  node_->declare_parameter<int>("consecutive_outliers_to_trigger", consecutive_outliers_to_trigger_);

  // TODO(andyz): set to false when done testing
  is_active_ = true;

  return controller_interface::return_type::OK;
}

CallbackReturn CollisionSensor::on_configure(const rclcpp_lifecycle::State& /*previous_state*/)
{
  const auto logger = node_->get_logger();

  // update parameters
  joint_names_ = node_->get_parameter("joints").as_string_array();
  num_dof_ = joint_names_.size();
  torque_threshold_newton_meters_ = node_->get_parameter("torque_threshold_newton_meters").as_double();
  consecutive_outliers_to_trigger_ = node_->get_parameter("consecutive_outliers_to_trigger").as_int();

  contact_monitor_ =
      std::make_shared<ContactDetection>(num_dof_, torque_threshold_newton_meters_, consecutive_outliers_to_trigger_);

  if (joint_names_.empty())
  {
    RCLCPP_WARN(logger, "'joints' parameter is empty.");
  }

  state_interface_types_ = node_->get_parameter("state_interfaces").as_string_array();

  if (state_interface_types_.empty())
  {
    RCLCPP_ERROR(logger, "'state_interfaces' parameter is empty.");
    return CallbackReturn::FAILURE;
  }

  auto get_interface_list = [](const std::vector<std::string>& interface_types) {
    std::stringstream ss_command_interfaces;
    for (size_t index = 0; index < interface_types.size(); ++index)
    {
      if (index != 0)
      {
        ss_command_interfaces << " ";
      }
      ss_command_interfaces << interface_types[index];
    }
    return ss_command_interfaces.str();
  };

  // Print output so users can be sure the interface setup is correct
  RCLCPP_INFO(logger, "State interfaces are [%s].", get_interface_list(state_interface_types_).c_str());

  RCLCPP_DEBUG(node_->get_logger(), "configure successful");
  return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration CollisionSensor::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration command_interfaces_config;
  command_interfaces_config.type = controller_interface::interface_configuration_type::NONE;
  return command_interfaces_config;
}

controller_interface::InterfaceConfiguration CollisionSensor::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration conf;
  conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  conf.names.reserve(joint_names_.size() * state_interface_types_.size());
  for (const auto& joint_name : joint_names_)
  {
    for (const auto& interface_type : state_interface_types_)
    {
      conf.names.push_back(joint_name + "/" + interface_type);
    }
  }
  return conf;
}

CallbackReturn CollisionSensor::on_activate(const rclcpp_lifecycle::State& /*previous_state*/)
{
  return CallbackReturn::SUCCESS;
}

CallbackReturn CollisionSensor::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/)
{
  return CallbackReturn::SUCCESS;
}

controller_interface::return_type CollisionSensor::update()
{
  if (is_active_)
  {
    std::vector<double> torques(num_dof_);

    for (size_t joint_index = 0; joint_index < state_interfaces_.size(); ++joint_index)
    {
      torques.at(joint_index) = state_interfaces_.at(joint_index).get_value();
    }

    if (contact_monitor_->registerMeasurement(torques) == ReturnCode::CONTACT_DETECTED)
    {
      // TODO(andyz): what do we want to do when contact is detected? Publish a bool?
      RCLCPP_ERROR_STREAM(node_->get_logger(), "Collision detected!");
    }
  }

  return controller_interface::return_type::OK;
}

}  // namespace collision_sensor

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(collision_sensor::CollisionSensor, controller_interface::ControllerInterface)
