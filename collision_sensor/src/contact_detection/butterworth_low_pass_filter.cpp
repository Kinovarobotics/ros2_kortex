#include "collision_sensor/contact_detection/butterworth_low_pass_filter.hpp"

#include <cmath>
#include <string>
#include <stdexcept>

namespace collision_sensor
{
namespace
{
constexpr double EPSILON = 1e-9;
// TODO(andyz): read from yaml
constexpr double FILTER_COEFFICIENT = 1.1;
}  // namespace

ButterworthFilter::ButterworthFilter()
  : previous_measurements_{ 0., 0. }
  , previous_filtered_measurement_(0.)
  , scale_term_(1. / (1. + FILTER_COEFFICIENT))
  , feedback_term_(1. - FILTER_COEFFICIENT)
{
  // guarantee this doesn't change because the logic below depends on this length implicity
  static_assert(ButterworthFilter::FILTER_LENGTH == 2, "ButterworthFilter::FILTER_LENGTH should be 2");

  // Make sure input values are ok
  if (std::isinf(feedback_term_))
    throw std::length_error("ButterworthFilter: infinite feedback_term_");

  if (std::isinf(scale_term_))
    throw std::length_error("ButterworthFilter: infinite scale_term_");

  if (FILTER_COEFFICIENT < 1)
    throw std::length_error("ButterworthFilter: Filter coefficient < 1. makes the lowpass filter unstable");

  if (std::abs(feedback_term_) < EPSILON)
    throw std::length_error("ButterworthFilter: Filter coefficient value resulted in feedback term of 0");
}

void ButterworthFilter::reset(double data)
{
  previous_measurements_[0] = data;
  previous_measurements_[1] = data;

  previous_filtered_measurement_ = data;
}

double ButterworthFilter::getLastFilteredValue()
{
  return previous_filtered_measurement_;
}

double ButterworthFilter::filter(double new_measurement)
{
  // Push in the new measurement
  previous_measurements_[1] = previous_measurements_[0];
  previous_measurements_[0] = new_measurement;

  double new_filtered_measurement = scale_term_ * (previous_measurements_[1] + previous_measurements_[0] -
                                                   feedback_term_ * previous_filtered_measurement_);

  // Store the new filtered measurement
  previous_filtered_measurement_ = new_filtered_measurement;

  return new_filtered_measurement;
}
}  // namespace collision_sensor
