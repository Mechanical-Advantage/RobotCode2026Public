// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

#pragma once

#include <ctre/phoenix6/TalonFX.hpp>

#include "FlywheelIO.h"
#include "util/TalonFXCurrentConfigurator.h"

class FlywheelIOTalonFX : public FlywheelIO {
 public:
  FlywheelIOTalonFX(const std::string_view name, const int id,
                    const std::string_view bus, const int idFollower1,
                    const int idFollower2, const int idFollower3,
                    const bool invert, const double reduction);

 private:
  void updateInputs(FlywheelIOInputs& inputs) override;
  void applyOutputs(const FlywheelIOOutputs& outputs) override;
  void stop() override;

  // Hardware
  ctre::phoenix6::hardware::TalonFX talon;
  std::array<ctre::phoenix6::hardware::TalonFX, 3> followerTalons;

  // Config
  ctre::phoenix6::configs::TalonFXConfiguration config{};

  // Control Requests
  ctre::phoenix6::controls::CoastOut coastRequest =
      ctre::phoenix6::controls::CoastOut().WithUpdateFreqHz(50_Hz);
  ctre::phoenix6::controls::VelocityVoltage velocityRequest =
      ctre::phoenix6::controls::VelocityVoltage(0_rad_per_s)
          .WithUpdateFreqHz(0_Hz);
  ctre::phoenix6::controls::VoltageOut voltageRequest =
      ctre::phoenix6::controls::VoltageOut(0_V).WithUpdateFreqHz(0_Hz);
  ctre::phoenix6::controls::Follower follower =
      ctre::phoenix6::controls::Follower(
          talon.GetDeviceID(),
          ctre::phoenix6::signals::MotorAlignmentValue::Aligned)
          .WithUpdateFreqHz(20_Hz);
  ctre::phoenix6::controls::Follower oppFollower =
      ctre::phoenix6::controls::Follower(
          talon.GetDeviceID(),
          ctre::phoenix6::signals::MotorAlignmentValue::Opposed)
          .WithUpdateFreqHz(20_Hz);

  // Inputs
  ctre::phoenix6::StatusSignal<units::angle::turn_t> position;
  ctre::phoenix6::StatusSignal<units::angular_velocity::turns_per_second_t>
      velocity;
  ctre::phoenix6::StatusSignal<units::voltage::volt_t> appliedVoltage;
  ctre::phoenix6::StatusSignal<units::voltage::volt_t> supplyVoltage;
  ctre::phoenix6::StatusSignal<units::current::ampere_t> supplyCurrentAmps;
  ctre::phoenix6::StatusSignal<units::current::ampere_t> torqueCurrentAmps;
  ctre::phoenix6::StatusSignal<units::temperature::celsius_t> tempCelsius;

  ctre::phoenix6::StatusSignal<units::current::ampere_t>
      follower1SupplyCurrentAmps;
  ctre::phoenix6::StatusSignal<units::temperature::celsius_t>
      follower1TempCelsius;
  ctre::phoenix6::StatusSignal<units::current::ampere_t>
      follower2SupplyCurrentAmps;
  ctre::phoenix6::StatusSignal<units::temperature::celsius_t>
      follower2TempCelsius;
  ctre::phoenix6::StatusSignal<units::current::ampere_t>
      follower3SupplyCurrentAmps;
  ctre::phoenix6::StatusSignal<units::temperature::celsius_t>
      follower3TempCelsius;
  ctre::phoenix6::StatusSignal<bool> bridgeBrownout;
  ctre::phoenix6::StatusSignal<bool> follower1BridgeBrownout;
  ctre::phoenix6::StatusSignal<bool> follower2BridgeBrownout;
  ctre::phoenix6::StatusSignal<bool> follower3BridgeBrownout;

  // PID tracking
  double prevkP = 0.0;
  double prevkD = 0.0;
};
