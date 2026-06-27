// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

#pragma once

#include <ctre/phoenix6/Pigeon2.hpp>

#include "DriveConstants.h"
#include "GyroIO.h"

class GyroIOPigeon2 : public GyroIO {
 public:
  GyroIOPigeon2(const std::string_view name, const std::string_view canBus,
                const int id);

 private:
  void updateInputs(GyroIOInputs& inputs) override;
  void applyOutputs(const GyroIOOutputs& outputs) override;
  void stop() override;

  ctre::phoenix6::hardware::Pigeon2 pigeon;
  ctre::phoenix6::StatusSignal<units::angle::degree_t> yaw = pigeon.GetYaw();
  ctre::phoenix6::StatusSignal<units::angle::degree_t> pitch =
      pigeon.GetPitch();
  ctre::phoenix6::StatusSignal<units::angle::degree_t> roll = pigeon.GetRoll();
  ctre::phoenix6::StatusSignal<units::angular_velocity::degrees_per_second_t>
      yawVelocity = pigeon.GetAngularVelocityZWorld();
  ctre::phoenix6::StatusSignal<units::angular_velocity::degrees_per_second_t>
      pitchVelocity = pigeon.GetAngularVelocityXWorld();
  ctre::phoenix6::StatusSignal<units::angular_velocity::degrees_per_second_t>
      rollVelocity = pigeon.GetAngularVelocityYWorld();
  ctre::phoenix6::StatusSignal<units::acceleration::standard_gravity_t>
      accelerationX = pigeon.GetAccelerationX();
  ctre::phoenix6::StatusSignal<units::acceleration::standard_gravity_t>
      accelerationY = pigeon.GetAccelerationY();
  ctre::phoenix6::StatusSignal<units::acceleration::standard_gravity_t>
      accelerationZ = pigeon.GetAccelerationZ();
};
