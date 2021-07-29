#pragma once

// A class for synchronous filtering and contact detection from torque signals

#include "collision_sensor/contact_detection/butterworth_low_pass_filter.hpp"
#include "collision_sensor/contact_detection/error_codes.hpp"

#include <cstddef>
#include <vector>

namespace collision_sensor
{
class ContactDetection
{
public:
  ContactDetection(size_t num_dof, double torque_threshold_newton_meters, size_t consecutive_outliers_to_trigger);

  // Receive a measurement and check for contact
  ReturnCode registerMeasurement(std::vector<double>& current_joint_torques);

  // Select which joints are checked for contact
  // active_joint_indices should start from 0. Examples of valid input for a 6-dof robot:
  // (0, 1, 2, 3, 4, 5)  (0, 2, 5)  (1,3,4)
  ReturnCode selectActiveJoints(std::vector<size_t>& active_joint_indices);

private:
  size_t num_dof_;
  std::vector<ButterworthFilter> filters_;

  // Check the joints in this filter for collision. 0-indexed
  std::vector<size_t> active_joints_;

  // Count number of torque outliers for each joint
  std::vector<size_t> per_joint_outlier_count_;

  // >0. If torque suddenly changes more than this, an outlier is counted
  double torque_threshold_newton_meters_;

  // A single outlier doesn't mean much. Check for this many consecutive outliers
  size_t consecutive_outliers_to_trigger_;
};

}  // namespace collision_sensor
