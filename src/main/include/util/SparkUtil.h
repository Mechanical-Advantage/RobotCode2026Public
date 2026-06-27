// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

#pragma once

#include <rev/SparkBase.h>

#include <functional>
#include <memory>
#include <vector>

class SparkUtil {
 public:
  SparkUtil() = delete;
  static bool sparkStickyFault;

  /** Return a value from a Spark (or the default if the value is invalid). */
  static double ifOkOrDefault(std::unique_ptr<rev::spark::SparkBase>& spark,
                              const std::function<double()>& supplier,
                              double defaultValue);

  static double ifOkOrDefault(
      std::unique_ptr<rev::spark::SparkBase>& spark,
      const std::vector<std::function<double()>>& suppliers,
      std::function<double(const std::vector<double>&)> transformer,
      double defaultValue);

  static void tryUntilOk(int maxAttempts,
                         const std::function<rev::REVLibError()>& result);

  static double rotationsToRads(double rotations);

  static double radsToRotations(double rads);

  static double rpmToRadsPerSec(double rpm);

  static double radsPerSecToRpm(double radsPerSec);
};
