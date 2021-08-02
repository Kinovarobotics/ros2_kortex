#include "collision_sensor/contact_detection/contact_detection.hpp"

namespace collision_sensor
{
ContactDetection::ContactDetection(size_t num_dof, double torque_threshold_newton_meters,
                                   size_t consecutive_outliers_to_trigger)
  : num_dof_(num_dof)
  , torque_threshold_newton_meters_(torque_threshold_newton_meters)
  , consecutive_outliers_to_trigger_(consecutive_outliers_to_trigger)
  , joint_torque_filters_initialized_(false)
{
  filters_.resize(num_dof_);

  // By default, check all joints for collision
  active_joints_.resize(num_dof_);
  per_joint_outlier_count_.resize(num_dof_);
  for (size_t joint_index = 0; joint_index < num_dof_; ++joint_index)
  {
    active_joints_.at(joint_index) = joint_index;
  }
}

ReturnCode ContactDetection::registerMeasurement(std::vector<double>& current_joint_torques)
{
  if (current_joint_torques.size() != num_dof_)
  {
    return WRONG_JOINT_TORQUE_DIMENSION;
  }

  if (!joint_torque_filters_initialized_)
  {
    for (size_t joint = 0; joint < num_dof_; ++joint)
    {
      filters_.at(joint).reset(current_joint_torques.at(joint));
    }
    joint_torque_filters_initialized_ = true;
  }

  // For each joint, check if the current measurement is a deviation from the running average
  for (size_t& joint_index : active_joints_)
  {
    auto& joint_filter = filters_.at(joint_index);

    // Check for outlier
    if ((current_joint_torques.at(joint_index) >
         (joint_filter.getLastFilteredValue() + torque_threshold_newton_meters_)) ||
        (current_joint_torques.at(joint_index) <
         (joint_filter.getLastFilteredValue() - torque_threshold_newton_meters_)))
    {
      ++per_joint_outlier_count_.at(joint_index);
      if (per_joint_outlier_count_.at(joint_index) > consecutive_outliers_to_trigger_)
      {
        return CONTACT_DETECTED;
      }
    }
    else
    {
      per_joint_outlier_count_.at(joint_index) = 0;
    }

    // Filter for the next iteration
    joint_filter.filter(current_joint_torques.at(joint_index));
  }

  return NO_ERROR;
}

ReturnCode ContactDetection::selectActiveJoints(std::vector<size_t>& active_joint_indices)
{
  if ((active_joint_indices.size() > num_dof_) || (active_joint_indices.size() < 1))
  {
    return WRONG_ACTIVE_JOINT_DIMENSION;
  }

  active_joints_ = active_joint_indices;
  return NO_ERROR;
}

}  // namespace collision_sensor
