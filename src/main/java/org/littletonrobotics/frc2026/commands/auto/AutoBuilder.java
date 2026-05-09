// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2026.commands.auto;

import static org.littletonrobotics.frc2026.commands.auto.AutoCommands.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.List;
import java.util.Map;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import lombok.RequiredArgsConstructor;
import org.littletonrobotics.frc2026.AutoFieldConstants;
import org.littletonrobotics.frc2026.AutoFieldConstants.*;
import org.littletonrobotics.frc2026.AutoSelector.AutoQuestionResponse;
import org.littletonrobotics.frc2026.subsystems.drive.Drive;
import org.littletonrobotics.frc2026.subsystems.drive.DriveConstants;
import org.littletonrobotics.frc2026.subsystems.hopper.Hopper;
import org.littletonrobotics.frc2026.subsystems.kicker.Kicker;
import org.littletonrobotics.frc2026.subsystems.launcher.LaunchCalculator;
import org.littletonrobotics.frc2026.subsystems.launcher.flywheel.Flywheel;
import org.littletonrobotics.frc2026.subsystems.launcher.hood.Hood;
import org.littletonrobotics.frc2026.subsystems.slamtake.Slamtake;

@RequiredArgsConstructor
@SuppressWarnings("unused")
public class AutoBuilder {
  private final Drive drive;
  private final Slamtake slamtake;
  private final Hopper hopper;
  private final Kicker kicker;
  private final Hood hood;
  private final Flywheel flywheel;

  private final Supplier<List<AutoQuestionResponse>> responses;
  private final DoubleSupplier offsetTime;

  public static final double neutralZoneIntakeTimeFirst = 2.5;
  public static final double neutralZoneIntakeTimeOther = 3.0;
  public static final double launchTime = 2.5;
  public static final double depotLaunchTime = 1.5;
  public static final double autoDuration = 21.0;

  // MARK: Monopoly
  public Command monopolySalesman() {
    Supplier<AutoQuestionResponse> startPosition = () -> responses.get().get(0);
    Supplier<AutoQuestionResponse> postLaunch = () -> responses.get().get(1);

    return Commands.sequence(
        // Drive to closest intaking position
        Commands.select(
            Map.of(
                AutoQuestionResponse.LEFT_TRENCH,
                followTrajectory("trenchLeftStartThroughDepot", drive, true),
                AutoQuestionResponse.LEFT_BUMP,
                followTrajectory("bumpLeftInnerThroughDepot", drive, true)),
            startPosition),

        // Drive to and launch from launch pose
        AutoCommands.driveToPose(drive, () -> Launch.leftTower)
            .raceWith(
                Commands.sequence(
                    AutoCommands.waitUntilWithinTolerance(
                        Launch.leftTower, 0.1, Rotation2d.fromDegrees(5)),
                    index(hopper, kicker, flywheel, slamtake))),

        // Initiate chosen end behavior
        Commands.select(
            Map.of(AutoQuestionResponse.NOTHING, Commands.none()), () -> responses.get().get(1)));
  }

  // MARK: Timid
  public Command timidSalesman() {
    Supplier<Pose2d> target =
        () -> {
          Translation2d offset = new Translation2d();
          switch (responses.get().get(0)) {
            case CENTER:
            case LEFT_BUMP:
            case RIGHT_BUMP:
              offset = new Translation2d(-1, 0);
              break;
            case LEFT_TRENCH:
              offset = new Translation2d(-1, -0.5);
              break;
            case RIGHT_TRENCH:
              offset = new Translation2d(-1, 0.5);
              break;
            default:
              break;
          }

          return LaunchCalculator.getStationaryAimedPose(
              responses.get().get(0).getTranslation().plus(offset), true);
        };

    return Commands.sequence(
        // Reset pose
        Commands.select(
            Map.of(
                AutoQuestionResponse.LEFT_TRENCH,
                resetPose(
                    new Pose2d(
                        AutoFieldConstants.Trench.leftStart.minus(
                            new Translation2d(DriveConstants.fullWidthX, 0.0)),
                        Rotation2d.kPi)),
                AutoQuestionResponse.LEFT_BUMP,
                resetPose(new Pose2d(AutoFieldConstants.Bump.leftInner, Rotation2d.kPi)),
                AutoQuestionResponse.CENTER,
                resetPose(new Pose2d(AutoFieldConstants.Hub.centerStart, Rotation2d.kPi)),
                AutoQuestionResponse.RIGHT_BUMP,
                resetPose(
                    new Pose2d(
                        AutoFieldConstants.Bump.rightInner.minus(
                            new Translation2d(DriveConstants.fullWidthX, 0.0)),
                        Rotation2d.kPi)),
                AutoQuestionResponse.RIGHT_TRENCH,
                resetPose(new Pose2d(AutoFieldConstants.Trench.rightStart, Rotation2d.kPi))),
            () -> responses.get().get(0)),

        // Drive backwards a little bit and launch
        Commands.parallel(
            AutoCommands.driveToPose(drive, target),
            Commands.sequence(
                waitUntilWithinTolerance(target, 0.1, Rotation2d.fromDegrees(5)),
                index(hopper, kicker, flywheel, slamtake))));
  }

  // MARK: Drive Forward 1m
  public Command driveForward1mSalesman() {
    return followTrajectory("DriveForward1m", drive, true);
  }
}
