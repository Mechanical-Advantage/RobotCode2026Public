// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

#include "subsystems/ModuleIOTalonFXAlpha.h"

#include <frc/RobotController.h>
#include <frc/geometry/Rotation2d.h>

#include <ctre/phoenix6/CANBus.hpp>

#include "DriveConstants.h"
#include "Robot.h"
#include "util/PhoenixUtil.h"

ModuleIOTalonFXAlpha::ModuleIOTalonFXAlpha(const std::string_view name,
                                           const std::string_view canBus,
                                           const int driveId, const int turnId,
                                           const int encoderId,
                                           const double encoderOffset,
                                           const double turnReduction)
    : ModuleIO(name),
      driveTalon(driveId, ctre::phoenix6::CANBus(canBus)),
      turnTalon(turnId, ctre::phoenix6::CANBus(canBus)),
      encoder(encoderId),
      encoderOffset(encoderOffset),
      torqueCurrentRequest(0_A),
      positionTorqueCurrentRequest(0_rad),
      drivePosition(driveTalon.GetPosition()),
      driveVelocity(driveTalon.GetVelocity()),
      driveAppliedVolts(driveTalon.GetMotorVoltage()),
      driveSupplyCurrent(driveTalon.GetSupplyCurrent()),
      driveTorqueCurrent(driveTalon.GetTorqueCurrent()),
      driveTempCelsius(driveTalon.GetDeviceTemp()),
      turnAbsolutePosition([&]() {
        return frc::Rotation2d(units::radian_t{
                   encoder.GetVoltage() / frc::RobotController::GetVoltage5V() *
                   2.0 * M_PI}) +
               frc::Rotation2d(units::radian_t{encoderOffset});
      }),
      turnPosition(turnTalon.GetPosition()),
      turnVelocity(turnTalon.GetVelocity()),
      turnAppliedVolts(turnTalon.GetMotorVoltage()),
      turnSupplyCurrent(turnTalon.GetSupplyCurrent()),
      turnTorqueCurrent(turnTalon.GetTorqueCurrent()),
      turnTempCelsius(turnTalon.GetDeviceTemp()) {
  // Configure drive motor
  ctre::phoenix6::configs::TalonFXConfiguration driveConfig{};
  driveConfig.MotorOutput.NeutralMode =
      ctre::phoenix6::signals::NeutralModeValue::Brake;
  driveConfig.Slot0 = ctre::phoenix6::configs::Slot0Configs()
                          .WithKP(DriveConstants::driveKp)
                          .WithKD(DriveConstants::driveKd);
  driveConfig.Feedback.SensorToMechanismRatio =
      Constants::robot == RobotType::ALPHABOT
          ? DriveConstants::alphabotDriveReduction
          : DriveConstants::darwinDriveReduction;
  driveConfig.TorqueCurrent.PeakForwardTorqueCurrent =
      units::ampere_t{DriveConstants::driveCurrentLimitAmps};
  driveConfig.TorqueCurrent.PeakReverseTorqueCurrent =
      -units::ampere_t{DriveConstants::driveCurrentLimitAmps};
  driveConfig.CurrentLimits.StatorCurrentLimit =
      units::ampere_t{DriveConstants::driveCurrentLimitAmps};
  driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
  driveConfig.CurrentLimits.SupplyCurrentLimit = 40.0_A;
  driveConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
  driveConfig.ClosedLoopRamps.TorqueClosedLoopRampPeriod = 0.02_s;

  PhoenixUtil::tryUntilOk(5, [&]() {
    return driveTalon.GetConfigurator().Apply(driveConfig,
                                              PhoenixUtil::configTimeout);
  });
  PhoenixUtil::tryUntilOk(5, [&]() {
    return driveTalon.SetPosition(0.0_rad, PhoenixUtil::configTimeout);
  });

  // Configure turn motor
  ctre::phoenix6::configs::TalonFXConfiguration turnConfig{};
  turnConfig.MotorOutput.NeutralMode =
      ctre::phoenix6::signals::NeutralModeValue::Brake;
  turnConfig.Slot0 = ctre::phoenix6::configs::Slot0Configs()
                         .WithKP(DriveConstants::turnKp)
                         .WithKD(DriveConstants::turnKd);
  turnConfig.Feedback.SensorToMechanismRatio = turnReduction;
  turnConfig.ClosedLoopGeneral.ContinuousWrap = true;
  turnConfig.TorqueCurrent.PeakForwardTorqueCurrent =
      units::ampere_t{DriveConstants::turnCurrentLimitAmps};
  turnConfig.TorqueCurrent.PeakReverseTorqueCurrent =
      -units::ampere_t{DriveConstants::turnCurrentLimitAmps};
  turnConfig.CurrentLimits.StatorCurrentLimit =
      units::ampere_t{DriveConstants::turnCurrentLimitAmps};
  turnConfig.CurrentLimits.StatorCurrentLimitEnable = true;
  turnConfig.MotorOutput.Inverted =
      ctre::phoenix6::signals::SensorDirectionValue::Clockwise_Positive;

  PhoenixUtil::tryUntilOk(5, [&]() {
    return turnTalon.GetConfigurator().Apply(turnConfig,
                                             PhoenixUtil::configTimeout);
  });
  PhoenixUtil::tryUntilOk(5, [&]() {
    return turnTalon.SetPosition(turnAbsolutePosition().Radians());
  });

  // Configure periodic frames
  ctre::phoenix6::BaseStatusSignal::SetUpdateFrequencyForAll(
      200_Hz, drivePosition, driveVelocity, driveAppliedVolts,
      driveSupplyCurrent, driveTorqueCurrent, driveTempCelsius, turnPosition,
      turnVelocity, turnAppliedVolts, turnSupplyCurrent, turnTorqueCurrent,
      turnTempCelsius);
  PhoenixUtil::tryUntilOk(5, [&]() {
    return ctre::phoenix6::hardware::ParentDevice::OptimizeBusUtilizationForAll(
        0_Hz, driveTalon, turnTalon);
  });

  PhoenixUtil::registerSignals(
      true, drivePosition, driveVelocity, driveAppliedVolts, driveSupplyCurrent,
      driveTorqueCurrent, turnPosition, turnVelocity, turnAppliedVolts,
      turnSupplyCurrent, turnTorqueCurrent);
};

void ModuleIOTalonFXAlpha::updateInputs(ModuleIOInputs& inputs) {
  inputs.encoderConnected = true;
  inputs.driveConnected = ctre::phoenix6::BaseStatusSignal::IsAllGood(
      drivePosition, driveVelocity, driveAppliedVolts, driveSupplyCurrent,
      driveTorqueCurrent, driveTempCelsius);
  inputs.drivePositionRads =
      drivePosition.GetValue().convert<units::radian>().value();
  inputs.driveVelocityRadsPerSec =
      driveVelocity.GetValue().convert<units::rad_per_s>().value();
  inputs.driveAppliedVolts =
      driveAppliedVolts.GetValue().convert<units::volt>().value();
  inputs.driveSupplyCurrentAmps =
      driveSupplyCurrent.GetValue().convert<units::ampere>().value();
  inputs.driveTorqueCurrentAmps =
      driveTorqueCurrent.GetValue().convert<units::ampere>().value();
  inputs.driveTempCelsius = driveTempCelsius.GetValue().value();

  inputs.turnConnected = ctre::phoenix6::BaseStatusSignal::IsAllGood(
      turnPosition, turnVelocity, turnAppliedVolts, turnSupplyCurrent,
      turnTorqueCurrent, turnTempCelsius);
  inputs.turnAbsolutePositionRads =
      turnAbsolutePosition() - frc::Rotation2d(units::radian_t{encoderOffset});
  inputs.turnPositionRads = frc::Rotation2d{turnPosition.GetValue()};
  inputs.turnVelocityRadsPerSec =
      turnVelocity.GetValue().convert<units::rad_per_s>().value();
  inputs.turnAppliedVolts =
      turnAppliedVolts.GetValue().convert<units::volt>().value();
  inputs.turnSupplyCurrentAmps =
      turnSupplyCurrent.GetValue().convert<units::ampere>().value();
  inputs.turnTorqueCurrentAmps =
      turnTorqueCurrent.GetValue().convert<units::ampere>().value();
  inputs.turnTempCelsius = turnTempCelsius.GetValue().value();
}

void ModuleIOTalonFXAlpha::applyOutputs(const ModuleIOOutputs& outputs) {
  switch (outputs.mode) {
    case ModuleIOOutputMode::COAST:
      driveTalon.SetControl(coastRequest);
      turnTalon.SetControl(coastRequest);
      break;

    case ModuleIOOutputMode::BRAKE:
      driveTalon.SetControl(brakeRequest);
      turnTalon.SetControl(brakeRequest);
      break;

    case ModuleIOOutputMode::DRIVE:
      driveTalon.SetControl(
          velocityTorqueCurrentRequest
              .WithVelocity(
                  units::radians_per_second_t{outputs.driveVelocityRadPerSec})
              .WithFeedForward(units::ampere_t{outputs.driveFeedforward}));
      if (outputs.turnNeutral) {
        turnTalon.SetControl(turnBrakeRequest);
      } else {
        turnTalon.SetControl(positionTorqueCurrentRequest.WithPosition(
            outputs.turnRotation.Radians()));
      }
      break;

    case ModuleIOOutputMode::CHARACTERIZE:
      driveTalon.SetControl(torqueCurrentRequest.WithOutput(
          units::ampere_t{outputs.driveCharacterizationOutput}));
      turnTalon.SetControl(positionTorqueCurrentRequest.WithPosition(
          outputs.turnRotation.Radians()));
      break;
  }
}

void ModuleIOTalonFXAlpha::stop() {
  // In local drive mode, the motors are already being commanded safely
  if (!Robot::isLocalDrive()) {
    driveTalon.StopMotor();
    turnTalon.StopMotor();
  }
}
