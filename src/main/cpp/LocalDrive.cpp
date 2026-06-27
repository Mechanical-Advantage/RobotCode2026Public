// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

#include "LocalDrive.h"

#include <frc/DriverStation.h>
#include <frc/MathUtil.h>

#include "CurrentLimits.h"
#include "DriveCommands.h"
#include "DriveConstants.h"
#include "Robot.h"
#include "idun/IdunClient.h"

LocalDrive::LocalDrive(GyroIO* gyro, ModuleIO* module0, ModuleIO* module1,
                       ModuleIO* module2, ModuleIO* module3)
    : gyro(gyro), module{module0, module1, module2, module3} {};

void LocalDrive::periodic() {
  if (!Robot::isLocalDrive()) {
    idun::IdunClient::updateGyroOffset(gyroOffset);
    return;
  }

  // Read gyro value
  GyroIO::GyroIOInputs gyroInputs;
  gyro->updateInputs(gyroInputs);

  // Check if alliance is flipped
  const bool isFlipped = frc::DriverStation::GetAlliance().value_or(
                             frc::DriverStation::Alliance::kBlue) ==
                         frc::DriverStation::Alliance::kRed;

  // Reset gyro offset on button press
  if (frc::DriverStation::GetStickButton(1, 7) &&
      frc::DriverStation::GetStickButton(1, 8)) {
    gyroOffset = -gyroInputs.yawPosition;
    if (isFlipped) {
      gyroOffset = gyroOffset + frc::Rotation2d{units::radian_t{M_PI}};
    }
  }

  // Get current gyro rotation
  frc::Rotation2d currentRotation = gyroInputs.yawPosition + gyroOffset;

  // Brake when disabled
  if (frc::DriverStation::IsDisabled()) {
    ModuleIO::ModuleIOOutputs moduleOutputs;
    moduleOutputs.mode = ModuleIOOutputMode::BRAKE;
    moduleOutputs.driveSupplyCurrentLimit = CurrentLimits::driveMaxLimitAmps;
    for (int i = 0; i < 4; i++) {
      module[i]->applyOutputs(moduleOutputs);
    }
    return;
  }

  // Process linear joystick inputs
  const double inputX = -frc::DriverStation::GetStickAxis(0, 1);
  const double inputY = -frc::DriverStation::GetStickAxis(0, 0);
  const double linearMagnitude =
      frc::ApplyDeadband(std::hypot(inputX, inputY), DriveCommands::deadband);
  const frc::Rotation2d linearDirection{
      units::radian_t{std::atan2(inputY, inputX)}};
  const auto linearVelocity =
      frc::Pose2d(frc::Translation2d{}, linearDirection)
          .TransformBy(frc::Transform2d(
              units::meter_t{linearMagnitude * linearMagnitude}, 0.0_m,
              frc::Rotation2d{}))
          .Translation();

  // Process angular joystick input
  const double inputOmega = -frc::DriverStation::GetStickAxis(0, 4);
  const double omegaRaw =
      frc::ApplyDeadband(inputOmega, DriveCommands::deadband);
  const double omega = std::copysign(omegaRaw * omegaRaw, omegaRaw);

  // Calculate chassis speeds
  const double maxLinearSpeed = Constants::robot == RobotType::ALPHABOT
                                    ? DriveConstants::alphabotMaxLinearSpeed
                                    : DriveConstants::darwinMaxLinearSpeed;
  const double maxAngularSpeed = Constants::robot == RobotType::ALPHABOT
                                     ? DriveConstants::alphabotMaxAngularSpeed
                                     : DriveConstants::darwinMaxAngularSpeed;
  const auto speeds = frc::ChassisSpeeds::FromFieldRelativeSpeeds(
      units::meters_per_second_t{linearVelocity.X().value() * maxLinearSpeed},
      units::meters_per_second_t{linearVelocity.Y().value() * maxLinearSpeed},
      units::radians_per_second_t{omega * maxAngularSpeed},
      isFlipped ? currentRotation + frc::Rotation2d{units::radian_t{M_PI}}
                : currentRotation);

  // Calculate setpoint states
  const auto discreteSpeeds = frc::ChassisSpeeds::Discretize(
      speeds, units::second_t{Constants::loopPeriodSecs});
  auto setpointStates = kinematics.ToSwerveModuleStates(discreteSpeeds);
  kinematics.DesaturateWheelSpeeds(&setpointStates,
                                   units::meters_per_second_t{maxLinearSpeed});

  // Apply module outputs
  const double wheelRadiusInches =
      Constants::robot == RobotType::ALPHABOT
          ? DriveConstants::alphabotWheelRadiusInches
          : DriveConstants::darwinWheelRadiusInches;
  for (int i = 0; i < 4; i++) {
    // Get latest inputs
    ModuleIO::ModuleIOInputs moduleInputs;
    module[i]->updateInputs(moduleInputs);

    // Optimize setpoint state
    setpointStates[i].Optimize(moduleInputs.turnPositionRads);
    setpointStates[i].CosineScale(moduleInputs.turnPositionRads);

    // Apply setpoint
    const double speedRadPerSec =
        setpointStates[i].speed.value() /
        units::inch_t{wheelRadiusInches}.convert<units::meter>().value();
    ModuleIO::ModuleIOOutputs moduleOutputs;
    moduleOutputs.mode = ModuleIOOutputMode::DRIVE;
    moduleOutputs.driveVelocityRadPerSec = speedRadPerSec;
    moduleOutputs.driveFeedforward =
        ffModel.Calculate(units::radians_per_second_t{speedRadPerSec}).value();
    moduleOutputs.turnRotation = setpointStates[i].angle;
    moduleOutputs.turnNeutral =
        std::abs((setpointStates[i].angle - moduleInputs.turnPositionRads)
                     .Degrees()
                     .value()) < DriveConstants::turnDeadbandDegrees;
    moduleOutputs.driveSupplyCurrentLimit = CurrentLimits::driveMaxLimitAmps;
    module[i]->applyOutputs(moduleOutputs);
  }
}
