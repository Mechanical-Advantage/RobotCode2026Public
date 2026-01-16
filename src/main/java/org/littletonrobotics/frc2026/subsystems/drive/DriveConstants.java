// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2026.subsystems.drive;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import org.littletonrobotics.frc2026.Constants;
import org.littletonrobotics.frc2026.Constants.RobotType;

public class DriveConstants {
  // MARK: - COMPBOT Constants

  public static final double compbotTrackWidthXInches = 20.75;
  public static final double compbotTrackWidthYInches = 20.75;
  public static final double compbotMaxLinearSpeed = 4.69;
  public static final double compbotMaxAngularSpeed = 6.46; // maxLinearSpeed / driveBaseRadius
  public static final double compbotWheelRadiusInches = 2.0;

  // SDS MK5i modules, R2 reduction
  public static final double compbotDriveReduction = 6.02678571429;
  public static final double compbotTurnReduction = 26.0;

  public static final String compbotCanBus = "*";
  public static final int compbotGyroId = 1;
  public static final int compbotDriveMotorIdFL = 2;
  public static final int compbotDriveMotorIdFR = 3;
  public static final int compbotDriveMotorIdBL = 4;
  public static final int compbotDriveMotorIdBR = 5;

  public static final int compbotTurnMotorIdFL = 6;
  public static final int compbotTurnMotorIdFR = 7;
  public static final int compbotTurnMotorIdBL = 8;
  public static final int compbotTurnMotorIdBR = 9;

  public static final int compbotEncoderIdFL = 41;
  public static final int compbotEncoderIdFR = 42;
  public static final int compbotEncoderIdBL = 43;
  public static final int compbotEncoderIdBR = 44;

  public static final double compbotEncoderOffsetFL = 0.0;
  public static final double compbotEncoderOffsetFR = 0.0;
  public static final double compbotEncoderOffsetBL = 0.0;
  public static final double compbotEncoderOffsetBR = 0.0;

  // MARK: - ALPHABOT Constants

  public static final double alphabotTrackWidthXInches = 22.75;
  public static final double alphabotTrackWidthYInches = 22.75;
  public static final double alphabotMaxLinearSpeed = 4.69;
  public static final double alphabotMaxAngularSpeed = 6.46; // maxLinearSpeed / driveBaseRadius
  public static final double alphabotWheelRadiusInches = 2.0;

  // SDS MK4i modules, L3 reduction
  public static final double alphabotDriveReduction = 6.1224489796;
  public static final double alphabotTurnReduction = 21.4285714286;

  public static final String alphabotCanBus = "*";
  public static final int alphabotGyroId = 1;
  public static final int alphabotDriveMotorIdFL = 30;
  public static final int alphabotDriveMotorIdFR = 2;
  public static final int alphabotDriveMotorIdBL = 1;
  public static final int alphabotDriveMotorIdBR = 3;

  public static final int alphabotTurnMotorIdFL = 4;
  public static final int alphabotTurnMotorIdFR = 7;
  public static final int alphabotTurnMotorIdBL = 5;
  public static final int alphabotTurnMotorIdBR = 6;

  public static final int alphabotEncoderIdFL = 0;
  public static final int alphabotEncoderIdFR = 1;
  public static final int alphabotEncoderIdBL = 2;
  public static final int alphabotEncoderIdBR = 3;

  public static final double alphabotEncoderOffsetFL = 0.012886;
  public static final double alphabotEncoderOffsetFR = 0.866168;
  public static final double alphabotEncoderOffsetBL = -1.050078;
  public static final double alphabotEncoderOffsetBR = -2.801255;

  // MARK: - Shared Constants

  public static final double driveKs = 5.0;
  public static final double driveKv = 0.0;
  public static final double driveKp = 35.0;
  public static final double driveKd = 0.0;
  public static final double turnKp = 4000.0;
  public static final double turnKd = 50.0;
  public static final double turnDeadbandDegrees = 0.3;
  public static final double driveCurrentLimitAmps = 80;
  public static final double turnCurrentLimitAmps = 40;

  public static final double trackWidthX =
      Units.inchesToMeters(
          Constants.robot == RobotType.ALPHABOT
              ? alphabotTrackWidthXInches
              : compbotTrackWidthXInches);
  public static final double trackWidthY =
      Units.inchesToMeters(
          Constants.robot == RobotType.ALPHABOT
              ? alphabotTrackWidthYInches
              : compbotTrackWidthYInches);
  public static final double driveBaseRadius = Math.hypot(trackWidthX / 2, trackWidthY / 2);
  public static final double maxLinearSpeed =
      Constants.robot == RobotType.ALPHABOT ? alphabotMaxLinearSpeed : compbotMaxLinearSpeed;
  public static final double maxAngularSpeed =
      Constants.robot == RobotType.ALPHABOT ? alphabotMaxAngularSpeed : compbotMaxAngularSpeed;
  public static final double wheelRadiusInches =
      Constants.robot == RobotType.ALPHABOT ? alphabotWheelRadiusInches : compbotWheelRadiusInches;
  public static final double wheelRadius = Units.inchesToMeters(wheelRadiusInches);
  public static final Translation2d[] moduleTranslations = {
    new Translation2d(trackWidthX / 2, trackWidthY / 2),
    new Translation2d(trackWidthX / 2, -trackWidthY / 2),
    new Translation2d(-trackWidthX / 2, trackWidthY / 2),
    new Translation2d(-trackWidthX / 2, -trackWidthY / 2)
  };

  public static final double driveReduction =
      Constants.robot == RobotType.ALPHABOT ? compbotDriveReduction : alphabotDriveReduction;
  public static final double turnReduction =
      Constants.robot == RobotType.ALPHABOT ? compbotTurnReduction : alphabotTurnReduction;
}
