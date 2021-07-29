#pragma once

#include <string>
#include <unordered_map>

namespace collision_sensor
{
enum ReturnCode
{
  NO_ERROR = 0,
  CONTACT_DETECTED = 1,
  WRONG_JOINT_TORQUE_DIMENSION = 2,
  WRONG_ACTIVE_JOINT_DIMENSION = 3
};

/**
 * \brief Use this map to look up human-readable strings for each error code
 */
const std::unordered_map<uint, std::string>
    ERROR_CODE_MAP({ { NO_ERROR, "No error" },
                     { WRONG_JOINT_TORQUE_DIMENSION, "Joint torque input does not have the right dimension" },
                     { WRONG_ACTIVE_JOINT_DIMENSION, "Request contains more DOF than the robot has!" } });
}  // namespace collision_sensor
