// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

#include "subsystems/GyroIOPigeon2.h"

#include <frc/geometry/Rotation2d.h>

#include "util/PhoenixUtil.h"

GyroIOPigeon2::GyroIOPigeon2(const std::string_view canBus, const int id)
    : GyroIO(), pigeon(id, ctre::phoenix6::CANBus(canBus)) {
  ctre::phoenix6::configs::Pigeon2Configuration config{};
  PhoenixUtil::tryUntilOk(5, [&]() {
    return pigeon.GetConfigurator().Apply(config, PhoenixUtil::configTimeout);
  });
  ctre::phoenix6::BaseStatusSignal::SetUpdateFrequencyForAll(
      200_Hz, yaw, pitch, roll, yawVelocity, pitchVelocity, rollVelocity);
  PhoenixUtil::tryUntilOk(5, [&]() {
    return pigeon.OptimizeBusUtilization(0_Hz, PhoenixUtil::configTimeout);
  });
  PhoenixUtil::registerSignals(ctre::phoenix6::CANBus(canBus).IsNetworkFD(),
                               yaw, pitch, roll, yawVelocity, pitchVelocity,
                               rollVelocity);
};

void GyroIOPigeon2::updateInputs(GyroIOInputs& inputs) {
  inputs.connected = ctre::phoenix6::BaseStatusSignal::IsAllGood(
      yaw, pitch, roll, yawVelocity, pitchVelocity, rollVelocity);
  inputs.yawPosition = frc::Rotation2d(yaw.GetValue());
  inputs.pitchPosition = frc::Rotation2d(pitch.GetValue());
  inputs.rollPosition = frc::Rotation2d(roll.GetValue());
  inputs.yawVelocityRadPerSec =
      yawVelocity.GetValue().convert<units::radians_per_second>().value();
  inputs.pitchVelocityRadPerSec =
      pitchVelocity.GetValue().convert<units::radians_per_second>().value();
  inputs.rollVelocityRadPerSec =
      rollVelocity.GetValue().convert<units::radians_per_second>().value();
};

void GyroIOPigeon2::applyOutputs(const GyroIOOutputs& outputs) {};

void GyroIOPigeon2::stop() {};
