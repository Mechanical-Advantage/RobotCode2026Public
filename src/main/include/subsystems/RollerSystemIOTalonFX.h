// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

#pragma once

#include <ctre/phoenix6/TalonFX.hpp>

#include "RollerSystemIO.h"

class RollerSystemIOTalonFX : public RollerSystemIO {
 public:
  RollerSystemIOTalonFX(const std::string_view name, const int id,
                        const std::string_view bus,
                        const units::current::ampere_t currentLimitAmps,
                        const bool invert, const double reduction);

 private:
  void updateInputs(RollerSystemIOInputs& inputs) override;
  void applyOutputs(const RollerSystemIOOutputs& outputs) override;
  void stop() override;

  // Hardware
  ctre::phoenix6::hardware::TalonFX talon;
  double reduction;

  // Control Requests
  ctre::phoenix6::controls::VoltageOut voltageRequest =
      ctre::phoenix6::controls::VoltageOut(0_V).WithUpdateFreqHz(50_Hz);
  ctre::phoenix6::controls::TorqueCurrentFOC torqueCurrentOut =
      ctre::phoenix6::controls::TorqueCurrentFOC(0_A).WithUpdateFreqHz(50_Hz);
  ctre::phoenix6::controls::CoastOut coastRequest =
      ctre::phoenix6::controls::CoastOut().WithUpdateFreqHz(50_Hz);
  ctre::phoenix6::controls::StaticBrake brakeRequest =
      ctre::phoenix6::controls::StaticBrake().WithUpdateFreqHz(50_Hz);

  // Inputs
  ctre::phoenix6::StatusSignal<units::angle::turn_t> position;
  ctre::phoenix6::StatusSignal<units::angular_velocity::turns_per_second_t>
      velocity;
  ctre::phoenix6::StatusSignal<units::voltage::volt_t> appliedVoltage;
  ctre::phoenix6::StatusSignal<units::current::ampere_t> supplyCurrentAmps;
  ctre::phoenix6::StatusSignal<units::current::ampere_t> torqueCurrentAmps;
  ctre::phoenix6::StatusSignal<units::temperature::celsius_t> tempCelsius;
};
