// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

#pragma once

#include <ctre/phoenix6/TalonFX.hpp>

#include "FlywheelIO.h"

class FlywheelIOTalonFX : public FlywheelIO {
 public:
  FlywheelIOTalonFX();

 private:
  void updateInputs(FlywheelIOInputs& inputs) override;
  void applyOutputs(const FlywheelIOOutputs& outputs) override;
  void stop() override;

  // Hardware
  ctre::phoenix6::hardware::TalonFX talon =
      ctre::phoenix6::hardware::TalonFX(14, "");
  ctre::phoenix6::hardware::TalonFX talonFollower =
      ctre::phoenix6::hardware::TalonFX(15, "");

  // Control Requests
  ctre::phoenix6::controls::CoastOut coastRequest =
      ctre::phoenix6::controls::CoastOut().WithUpdateFreqHz(50_Hz);
  ctre::phoenix6::controls::VelocityDutyCycle velocityDutyCycleRequest =
      ctre::phoenix6::controls::VelocityDutyCycle(0_rad_per_s)
          .WithUpdateFreqHz(0_Hz);
  ctre::phoenix6::controls::VelocityTorqueCurrentFOC
      velocityTorqueCurrentRequest =
          ctre::phoenix6::controls::VelocityTorqueCurrentFOC(0_rad_per_s)
              .WithUpdateFreqHz(0_Hz);
  ctre::phoenix6::controls::Follower follower =
      ctre::phoenix6::controls::Follower(
          talon.GetDeviceID(),
          ctre::phoenix6::signals::MotorAlignmentValue::Aligned)
          .WithUpdateFreqHz(20_Hz);

  // Inputs
  ctre::phoenix6::StatusSignal<units::angle::turn_t> position =
      talon.GetPosition();
  ctre::phoenix6::StatusSignal<units::angular_velocity::turns_per_second_t>
      velocity = talon.GetVelocity();
  ctre::phoenix6::StatusSignal<units::voltage::volt_t> appliedVoltage =
      talon.GetMotorVoltage();
  ctre::phoenix6::StatusSignal<units::current::ampere_t> supplyCurrentAmps =
      talon.GetSupplyCurrent();
  ctre::phoenix6::StatusSignal<units::current::ampere_t> torqueCurrentAmps =
      talon.GetTorqueCurrent();
  ctre::phoenix6::StatusSignal<units::temperature::celsius_t> tempCelsius =
      talon.GetDeviceTemp();

  ctre::phoenix6::StatusSignal<units::current::ampere_t>
      followerSupplyCurrentAmps = talonFollower.GetSupplyCurrent();
  ctre::phoenix6::StatusSignal<units::temperature::celsius_t>
      followerTempCelsius = talonFollower.GetDeviceTemp();
};
