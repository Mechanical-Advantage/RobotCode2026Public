// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

#pragma once

#include <grpl/LaserCan.h>

#include "FuelSensorIO.h"

class FuelSensorIOLaserCan : public FuelSensorIO {
 public:
  FuelSensorIOLaserCan(const std::string_view name, const int id);

 private:
  void updateInputs(FuelSensorIOInputs& inputs) override;
  void applyOutputs(const FuelSensorIOOutputs& outputs) override;
  void stop() override;

  // Hardware
  grpl::LaserCan laserCan;
};
