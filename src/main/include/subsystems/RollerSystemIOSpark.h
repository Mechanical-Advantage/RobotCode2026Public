// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

#pragma once

#include <rev/SparkBase.h>
#include <rev/config/SparkBaseConfig.h>
#include <units/current.h>

#include "RollerSystemIO.h"

class RollerSystemIOSpark : public RollerSystemIO {
 public:
  RollerSystemIOSpark(const std::string_view name, const int id,
                      units::current::ampere_t currentLimitAmps,
                      const double reduction, const bool invert,
                      const bool isFlex);

 private:
  void updateInputs(RollerSystemIOInputs& inputs) override;
  void applyOutputs(const RollerSystemIOOutputs& outputs) override;
  void stop() override;
  bool lastBrakeMode = true;

  // Hardware
  std::unique_ptr<rev::spark::SparkBase> spark;
  rev::spark::SparkBaseConfig config;
};
