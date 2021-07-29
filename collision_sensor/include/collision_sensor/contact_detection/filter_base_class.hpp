#pragma once

// Base class for filtering (e.g. lowpass or Kalman filtering)

namespace collision_sensor
{
class FilterBase
{
public:
  virtual double filter(double new_measurement) = 0;

  virtual void reset(double data) = 0;
};

}  // namespace collision_sensor
