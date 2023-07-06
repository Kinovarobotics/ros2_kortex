// Copyright (c) 2019 Kinova inc. All rights reserved.
//
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file or at
// https://developers.google.com/open-source/licenses/bsd

#ifndef KORTEX_DRIVER__KORTEX_MATH_UTIL_HPP_
#define KORTEX_DRIVER__KORTEX_MATH_UTIL_HPP_

#include <cmath>

class KortexMathUtil
{
public:
  KortexMathUtil() {}
  ~KortexMathUtil() {}

  static double toRad(double degree);
  static double toDeg(double rad);
  static int getNumberOfTurns(double rad_not_wrapped);
  static double wrapRadiansFromMinusPiToPi(double rad_not_wrapped);
  static double wrapRadiansFromMinusPiToPi(double rad_not_wrapped, int & number_of_turns);
  static double wrapDegreesFromZeroTo360(double deg_not_wrapped);
  static double wrapDegreesFromZeroTo360(double deg_not_wrapped, int & number_of_turns);
  static double relative_position_from_absolute(
    double absolute_position, double min_value, double max_value);
  static double absolute_position_from_relative(
    double relative_position, double min_value, double max_value);
};

#endif  // KORTEX_DRIVER__KORTEX_MATH_UTIL_HPP_
