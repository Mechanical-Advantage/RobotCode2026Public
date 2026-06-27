// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

#include "util/SparkUtil.h"

#include <numbers>

bool SparkUtil::sparkStickyFault = false;

double SparkUtil::ifOkOrDefault(std::unique_ptr<rev::spark::SparkBase>& spark,
                                const std::function<double()>& supplier,
                                double defaultValue) {
  double value = supplier();
  if (spark->GetLastError() == rev::REVLibError::kOk) {
    return value;
  } else {
    SparkUtil::sparkStickyFault = true;
    return defaultValue;
  }
}

double SparkUtil::ifOkOrDefault(
    std::unique_ptr<rev::spark::SparkBase>& spark,
    const std::vector<std::function<double()>>& suppliers,
    std::function<double(const std::vector<double>&)> transformer,
    double defaultValue) {
  std::vector<double> values(suppliers.size());
  for (size_t i = 0; i < suppliers.size(); i++) {
    values[i] = suppliers[i]();
    if (spark->GetLastError() != rev::REVLibError::kOk) {
      SparkUtil::sparkStickyFault = true;
      return defaultValue;
    }
  }
  return transformer(values);
}

void SparkUtil::tryUntilOk(int maxAttempts,
                           const std::function<rev::REVLibError()>& result) {
  for (int i = 0; i < maxAttempts; i++) {
    auto error = result();
    if (error == rev::REVLibError::kOk) {
      break;
    } else {
      SparkUtil::sparkStickyFault = true;
    }
  }
}

double SparkUtil::rotationsToRads(double rotations) {
  return rotations * 2.0 * std::numbers::pi_v<double>;
}

double SparkUtil::radsToRotations(double rads) {
  return rads / (2.0 * std::numbers::pi_v<double>);
}

double SparkUtil::rpmToRadsPerSec(double rpm) {
  return rpm * 2.0 * std::numbers::pi_v<double> / 60.0;
}

double SparkUtil::radsPerSecToRpm(double radsPerSec) {
  return radsPerSec * 60.0 / (2.0 * std::numbers::pi_v<double>);
}
