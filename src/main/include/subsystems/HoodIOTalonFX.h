// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

#pragma once

#include <ctre/phoenix6/TalonFX.hpp>

#include "HoodIO.h"
#include "util/TalonFXCurrentConfigurator.h"

class HoodIOTalonFX : public HoodIO {
 public:
  HoodIOTalonFX();

 private:
  void updateInputs(HoodIOInputs& inputs) override;
  void applyOutputs(const HoodIOOutputs& outputs) override;
  void stop() override;

  // Hardware
  ctre::phoenix6::hardware::TalonFX talon{5, "*"};

  // Config
  ctre::phoenix6::configs::TalonFXConfiguration config{};

  // Control Requests
  ctre::phoenix6::controls::PositionTorqueCurrentFOC positionRequest =
      ctre::phoenix6::controls::PositionTorqueCurrentFOC(0.0_rad)
          .WithUpdateFreqHz(0_Hz);
  ctre::phoenix6::controls::VoltageOut voltageRequest =
      ctre::phoenix6::controls::VoltageOut(0_V).WithUpdateFreqHz(50_Hz);
  ctre::phoenix6::controls::CoastOut coastRequest =
      ctre::phoenix6::controls::CoastOut().WithUpdateFreqHz(50_Hz);
  ctre::phoenix6::controls::StaticBrake brakeRequest =
      ctre::phoenix6::controls::StaticBrake().WithUpdateFreqHz(50_Hz);

  // Inputs
  ctre::phoenix6::StatusSignal<units::angle::turn_t> internalPosition =
      talon.GetPosition();
  ctre::phoenix6::StatusSignal<units::angular_velocity::turns_per_second_t>
      internalVelocity = talon.GetVelocity();
  ctre::phoenix6::StatusSignal<units::voltage::volt_t> appliedVoltage =
      talon.GetMotorVoltage();
  ctre::phoenix6::StatusSignal<units::current::ampere_t> supplyCurrentAmps =
      talon.GetSupplyCurrent();
  ctre::phoenix6::StatusSignal<units::current::ampere_t> torqueCurrentAmps =
      talon.GetTorqueCurrent();
  ctre::phoenix6::StatusSignal<units::temperature::celsius_t> tempCelsius =
      talon.GetDeviceTemp();

  // PID tracking
  double prevkP = 0.0;
  double prevkD = 0.0;
};
