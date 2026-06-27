// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

#pragma once

#include <frc/AnalogInput.h>

#include <ctre/phoenix6/CANcoder.hpp>
#include <ctre/phoenix6/TalonFX.hpp>

#include "DriveConstants.h"
#include "ModuleIO.h"

class ModuleIOTalonFXAlpha : public ModuleIO {
 public:
  ModuleIOTalonFXAlpha(const std::string_view name,
                       const std::string_view canBus, const int driveId,
                       const int turnId, const int encoderId,
                       const double encoderOffset);

 private:
  void updateInputs(ModuleIOInputs& inputs) override;
  void applyOutputs(const ModuleIOOutputs& outputs) override;
  void stop() override;

  // Hardware
  ctre::phoenix6::hardware::TalonFX driveTalon;
  ctre::phoenix6::hardware::TalonFX turnTalon;
  frc::AnalogInput encoder;
  double encoderOffset;

  // Control requests
  ctre::phoenix6::controls::TorqueCurrentFOC torqueCurrentRequest =
      ctre::phoenix6::controls::TorqueCurrentFOC(0_A).WithUpdateFreqHz(0_Hz);
  ctre::phoenix6::controls::PositionTorqueCurrentFOC
      positionTorqueCurrentRequest =
          ctre::phoenix6::controls::PositionTorqueCurrentFOC(0_rad)
              .WithUpdateFreqHz(0_Hz);
  ctre::phoenix6::controls::VelocityTorqueCurrentFOC
      velocityTorqueCurrentRequest =
          ctre::phoenix6::controls::VelocityTorqueCurrentFOC(0_rad_per_s)
              .WithUpdateFreqHz(0_Hz);
  ctre::phoenix6::controls::CoastOut coastRequest =
      ctre::phoenix6::controls::CoastOut().WithUpdateFreqHz(20_Hz);
  ctre::phoenix6::controls::StaticBrake brakeRequest =
      ctre::phoenix6::controls::StaticBrake().WithUpdateFreqHz(20_Hz);
  ctre::phoenix6::controls::StaticBrake turnBrakeRequest =
      ctre::phoenix6::controls::StaticBrake().WithUpdateFreqHz(0_Hz);

  // Drive inputs
  ctre::phoenix6::StatusSignal<units::angle::turn_t> drivePosition;
  ctre::phoenix6::StatusSignal<units::angular_velocity::turns_per_second_t>
      driveVelocity;
  ctre::phoenix6::StatusSignal<units::voltage::volt_t> driveAppliedVolts;
  ctre::phoenix6::StatusSignal<units::current::ampere_t> driveSupplyCurrent;
  ctre::phoenix6::StatusSignal<units::current::ampere_t> driveTorqueCurrent;
  ctre::phoenix6::StatusSignal<units::temperature::celsius_t> driveTempCelsius;

  // Turn inputs
  std::function<frc::Rotation2d()> turnAbsolutePosition;
  ctre::phoenix6::StatusSignal<units::angle::turn_t> turnPosition;
  ctre::phoenix6::StatusSignal<units::angular_velocity::turns_per_second_t>
      turnVelocity;
  ctre::phoenix6::StatusSignal<units::voltage::volt_t> turnAppliedVolts;
  ctre::phoenix6::StatusSignal<units::current::ampere_t> turnSupplyCurrent;
  ctre::phoenix6::StatusSignal<units::current::ampere_t> turnTorqueCurrent;
  ctre::phoenix6::StatusSignal<units::temperature::celsius_t> turnTempCelsius;
};
