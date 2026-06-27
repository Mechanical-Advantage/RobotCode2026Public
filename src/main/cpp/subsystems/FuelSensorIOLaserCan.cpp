// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

#include "subsystems/FuelSensorIOLaserCan.h"

FuelSensorIOLaserCan::FuelSensorIOLaserCan(const std::string_view name,
                                           const int id)
    : FuelSensorIO(name), laserCan(grpl::LaserCan(id)) {
  laserCan.set_timing_budget(grpl::LaserCanTimingBudget::TB20ms);
  laserCan.set_ranging_mode(grpl::LaserCanRangingMode::Short);
  laserCan.set_roi(grpl::LaserCanROI(8, 8, 16, 16));
};

void FuelSensorIOLaserCan::updateInputs(FuelSensorIOInputs& inputs) {
  std::optional<grpl::LaserCanMeasurement> measurement =
      laserCan.get_measurement();
  inputs.valid =
      measurement.has_value() &&
      measurement.value().status == grpl::LASERCAN_STATUS_VALID_MEASUREMENT;
  inputs.distanceMeters =
      inputs.valid ? ((double)measurement.value().distance_mm) / 1000.0 : 0;
};

void FuelSensorIOLaserCan::applyOutputs(const FuelSensorIOOutputs& outputs) {};

void FuelSensorIOLaserCan::stop() {};
