// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

#pragma once

#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <units/angular_acceleration.h>

#include "Constants.h"
#include "DriveConstants.h"
#include "GyroIO.h"
#include "ModuleIO.h"

class LocalDrive {
 public:
  LocalDrive(GyroIO* gyro, ModuleIO* module0, ModuleIO* module1,
             ModuleIO* module2, ModuleIO* module3);

  void periodic();

 private:
  GyroIO* gyro;
  ModuleIO* module[4];

  frc::Rotation2d gyroOffset{0_rad};

  const units::meter_t moduleOffsetX =
      units::inch_t{Constants::robot == RobotType::ALPHABOT
                        ? DriveConstants::alphabotTrackWidthXInches
                        : DriveConstants::compbotTrackWidthXInches} /
      2.0;
  const units::meter_t moduleOffsetY =
      units::inch_t{Constants::robot == RobotType::ALPHABOT
                        ? DriveConstants::alphabotTrackWidthYInches
                        : DriveConstants::compbotTrackWidthYInches} /
      2.0;
  frc::SwerveDriveKinematics<4> kinematics{
      frc::Translation2d{moduleOffsetX, moduleOffsetY},
      frc::Translation2d{moduleOffsetX, -moduleOffsetY},
      frc::Translation2d{-moduleOffsetX, moduleOffsetY},
      frc::Translation2d{-moduleOffsetX, -moduleOffsetY}};
  frc::SimpleMotorFeedforward<units::radians> ffModel{
      units::volt_t{DriveConstants::driveKs},
      units::volt_t{DriveConstants::driveKv} / 1.0_rad_per_s,
      0.0_V / 1.0_rad_per_s_sq, units::second_t{Constants::loopPeriodSecs}};
};
