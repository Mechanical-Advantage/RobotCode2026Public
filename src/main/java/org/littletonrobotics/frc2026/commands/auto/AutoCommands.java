// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2026.commands.auto;

import choreo.Choreo;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.Map;
import java.util.Optional;
import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.frc2026.AutoFieldConstants;
import org.littletonrobotics.frc2026.AutoFieldConstants.Launch;
import org.littletonrobotics.frc2026.AutoSelector.AutoQuestionResponse;
import org.littletonrobotics.frc2026.Constants;
import org.littletonrobotics.frc2026.Constants.Mode;
import org.littletonrobotics.frc2026.FieldConstants;
import org.littletonrobotics.frc2026.ObjectDetection;
import org.littletonrobotics.frc2026.RobotState;
import org.littletonrobotics.frc2026.commands.CompactingCommands;
import org.littletonrobotics.frc2026.commands.DriveToPose;
import org.littletonrobotics.frc2026.commands.DriveTrajectory;
import org.littletonrobotics.frc2026.subsystems.drive.Drive;
import org.littletonrobotics.frc2026.subsystems.drive.DriveConstants;
import org.littletonrobotics.frc2026.subsystems.hopper.Hopper;
import org.littletonrobotics.frc2026.subsystems.kicker.Kicker;
import org.littletonrobotics.frc2026.subsystems.launcher.LaunchCalculator;
import org.littletonrobotics.frc2026.subsystems.launcher.flywheel.Flywheel;
import org.littletonrobotics.frc2026.subsystems.slamtake.Slamtake;
import org.littletonrobotics.frc2026.subsystems.slamtake.Slamtake.IntakeGoal;
import org.littletonrobotics.frc2026.subsystems.slamtake.Slamtake.SlamGoal;
import org.littletonrobotics.frc2026.util.LoggedTunableNumber;
import org.littletonrobotics.frc2026.util.SuppliedWaitCommand;
import org.littletonrobotics.frc2026.util.geometry.AllianceFlipUtil;
import org.littletonrobotics.frc2026.util.geometry.Bounds;
import org.littletonrobotics.frc2026.util.geometry.VerticalFlipUtil;
import org.littletonrobotics.junction.Logger;

public class AutoCommands {
  private static final LoggedTunableNumber autoDriveLaunchKp =
      new LoggedTunableNumber("AutoCommands/Launching/kP", 8.0);
  private static final LoggedTunableNumber autoDriveLaunchKd =
      new LoggedTunableNumber("AutoCommands/Launching/kD", 0.5);
  public static final double bumpCrossTime = 1.5;

  // Drives to corner of fuel pool to set up neutral zone intaking
  public static Command salesmanTurn(Drive drive, Supplier<AutoQuestionResponse> side) {
    return driveToPose(
            drive,
            () -> {
              Pose2d currentPose =
                  AllianceFlipUtil.apply(RobotState.getInstance().getEstimatedPose());
              double xTarget = FieldConstants.FuelPool.leftCenter.getX() - 0.8;
              double t =
                  MathUtil.clamp(
                      Math.abs(currentPose.getX() - xTarget)
                          / (xTarget - FieldConstants.LinesVertical.neutralZoneNear),
                      -1.0,
                      1.0);
              return new Pose2d(
                  xTarget,
                  MathUtil.interpolate(
                      isLeftSide(side).getAsBoolean()
                          ? FieldConstants.FuelPool.leftCenter.getY() - 0.0
                          : FieldConstants.FuelPool.rightCenter.getY() + 0.0,
                      isLeftSide(side).getAsBoolean()
                          ? FieldConstants.FuelPool.leftCenter.getY() + 3.5
                          : FieldConstants.FuelPool.rightCenter.getY() - 3.5,
                      t),
                  isLeftSide(side).getAsBoolean()
                      ? Rotation2d.fromDegrees(-50.0)
                      : Rotation2d.fromDegrees(50.0));
            })
        .withTimeout(1.0);
  }

  /** Returns to the alliance zone from the closest side */
  public static Command returnToClosestLaunchPose(Drive drive) {
    return returnToDeterminedLaunchPose(
        drive,
        () -> {
          ChassisSpeeds fieldVelocity = RobotState.getInstance().getFieldVelocity();
          Translation2d fieldVelocityTranslation =
              new Translation2d(fieldVelocity.vxMetersPerSecond, fieldVelocity.vyMetersPerSecond);
          Translation2d currentTranslation =
              RobotState.getInstance()
                  .getEstimatedPose()
                  .getTranslation()
                  .plus(fieldVelocityTranslation.times(0.8));
          return currentTranslation.getDistance(
                  AllianceFlipUtil.apply(AutoFieldConstants.Launch.leftBump.getTranslation()))
              < currentTranslation.getDistance(
                  AllianceFlipUtil.apply(AutoFieldConstants.Launch.rightBump.getTranslation()));
        });
  }

  /** Returns to launch pose from neutral zone given {@link AutoQuestionResponse}. */
  public static Command returnToDeterminedLaunchPose(
      Drive drive, Supplier<AutoQuestionResponse> returnResponse) {
    return returnToDeterminedLaunchPose(
        drive,
        () ->
            returnResponse.get().equals(AutoQuestionResponse.LEFT)
                || returnResponse.get().equals(AutoQuestionResponse.LEFT_BUMP)
                || returnResponse.get().equals(AutoQuestionResponse.LEFT_NO_TRENCH));
  }

  /** Returns to launch pose from neutral zone. */
  public static Command returnToDeterminedLaunchPose(Drive drive, BooleanSupplier leftBump) {
    return new DriveToPose(
        drive,
        () -> {
          Pose2d currentPose = RobotState.getInstance().getEstimatedPose();
          Pose2d target =
              leftBump.getAsBoolean()
                  ? AllianceFlipUtil.apply(AutoFieldConstants.Launch.leftBump)
                  : AllianceFlipUtil.apply(AutoFieldConstants.Launch.rightBump);

          double minYOffset = 0.1;
          double maxYOffset = 0.4;
          double t =
              MathUtil.clamp(
                  (Math.cbrt(
                      (Math.abs(currentPose.getY() - target.getY()) - minYOffset)
                          / (maxYOffset - minYOffset))),
                  0.0,
                  1.0);

          double xPosition = AllianceFlipUtil.applyX(currentPose.getX());
          if (xPosition < FieldConstants.LinesVertical.neutralZoneNear + 1.0) {
            t = 0.0;
          }

          var targetTranslation =
              new Translation2d(
                  MathUtil.interpolate(
                      target.getX(),
                      AllianceFlipUtil.applyX(FieldConstants.LinesVertical.neutralZoneNear + 1.5),
                      t),
                  target.getY());
          var targetRotation =
              xPosition
                      < (FieldConstants.LinesVertical.starting
                              + FieldConstants.LinesVertical.neutralZoneNear)
                          / 2.0
                  ? target.getRotation()
                  : xPosition
                          < FieldConstants.LinesVertical.neutralZoneNear + DriveConstants.fullWidthX
                      ? AllianceFlipUtil.apply(Rotation2d.kPi)
                      : target.getTranslation().minus(currentPose.getTranslation()).getAngle();
          return new Pose2d(targetTranslation, targetRotation);
        });
  }

  private static Double launchOnTheMoveOmega() {
    // Run PID controller
    final var parameters = LaunchCalculator.getInstance().getParameters();
    return MathUtil.clamp(
        parameters.driveVelocity()
            + (parameters.driveAngle().minus(RobotState.getInstance().getRotation()).getRadians()
                * autoDriveLaunchKp.get())
            + ((parameters.driveVelocity()
                    - RobotState.getInstance().getRobotVelocity().omegaRadiansPerSecond)
                * autoDriveLaunchKd.get()),
        -DriveConstants.maxAngularSpeed,
        DriveConstants.maxAngularSpeed);
  }

  public static Command followTrajectory(String name, Drive drive, boolean start) {
    return followTrajectory(name, drive, start, () -> false);
  }

  public static Command followTrajectory(
      String name, Drive drive, boolean start, BooleanSupplier mirror) {
    Optional<Trajectory<SwerveSample>> trajectoryOptional = Choreo.loadTrajectory(name);
    if (trajectoryOptional.isPresent()) {
      Trajectory<SwerveSample> trajectory = trajectoryOptional.get();
      return Commands.sequence(
          start
              ? AutoCommands.resetPose(
                  () ->
                      mirror.getAsBoolean()
                          ? VerticalFlipUtil.apply(
                              trajectory
                                  .getInitialSample(AllianceFlipUtil.shouldFlip())
                                  .get()
                                  .getPose())
                          : trajectory
                              .getInitialSample(AllianceFlipUtil.shouldFlip())
                              .get()
                              .getPose())
              : Commands.none(),
          new DriveTrajectory(trajectory, drive, mirror));
    } else {
      throw new RuntimeException("Choreo Trajectory Not Found: " + name);
    }
  }

  public static Command followTrajectoryWhileAiming(
      String name, Drive drive, boolean start, BooleanSupplier shouldAim) {
    return followTrajectoryWhileAiming(name, drive, start, shouldAim, () -> false);
  }

  public static Command followTrajectoryWhileAiming(
      String name, Drive drive, boolean start, BooleanSupplier shouldAim, BooleanSupplier mirror) {
    Optional<Trajectory<SwerveSample>> trajectoryOptional = Choreo.loadTrajectory(name);
    if (trajectoryOptional.isPresent()) {
      Trajectory<SwerveSample> trajectory = trajectoryOptional.get();
      return Commands.sequence(
          start
              ? AutoCommands.resetPose(
                  trajectory.getInitialSample(AllianceFlipUtil.shouldFlip()).get().getPose())
              : Commands.none(),
          new DriveTrajectory(
              trajectory,
              () ->
                  shouldAim.getAsBoolean() ? Optional.of(launchOnTheMoveOmega()) : Optional.empty(),
              drive,
              mirror));
    } else {
      throw new RuntimeException("Choreo Trajectory Not Found: " + name);
    }
  }

  public static Command driveToCoastTarget(
      Drive drive,
      Supplier<AutoQuestionResponse> coastTarget,
      BooleanSupplier isBump,
      BooleanSupplier mirror) {
    return Commands.either(
        rushToCenter(drive, bumpCrossTime, true)
            .andThen(
                driveToPose(
                    drive,
                    () ->
                        coastTarget.get().equals(AutoQuestionResponse.HUB)
                            ? new Pose2d(
                                FieldConstants.LinesVertical.neutralZoneFar
                                    - DriveConstants.fullApothemX,
                                FieldConstants.fieldWidth / 2.0,
                                Rotation2d.fromDegrees(-15))
                            : coastTarget.get().equals(AutoQuestionResponse.BUMP)
                                ? new Pose2d(
                                    FieldConstants.LinesVertical.neutralZoneFar
                                        - DriveConstants.fullApothemX,
                                    FieldConstants.LinesHorizontal.leftBumpMiddle,
                                    Rotation2d.fromDegrees(-90))
                                : new Pose2d(
                                    FieldConstants.LinesVertical.neutralZoneFar
                                        - DriveConstants.fullApothemX,
                                    FieldConstants.LinesHorizontal.leftTrenchMiddle,
                                    Rotation2d.fromDegrees(-90)))),
        Commands.select(
            Map.of(
                AutoQuestionResponse.HUB,
                followTrajectory(
                    "launchLeftTrenchThroughTrenchToFarHubKachow", drive, false, mirror),
                AutoQuestionResponse.BUMP,
                followTrajectory(
                    "launchLeftTrenchThroughTrenchToFarBumpKachow", drive, false, mirror),
                AutoQuestionResponse.TRENCH,
                followTrajectory(
                    "launchLeftTrenchThroughTrenchToFarTrenchKachow", drive, false, mirror),
                AutoQuestionResponse.NONE,
                Commands.idle()),
            coastTarget),
        isBump);
  }

  public static Command rushToCenter(Drive drive, double time, boolean slow) {
    return Commands.run(
            () ->
                drive.runVelocity(
                    ChassisSpeeds.fromFieldRelativeSpeeds(
                        AllianceFlipUtil.shouldFlip()
                            ? -DriveConstants.maxLinearSpeed * (slow ? 0.6 : 1.0)
                            : DriveConstants.maxLinearSpeed * (slow ? 0.6 : 1.0),
                        0.0,
                        0.0,
                        RobotState.getInstance().getRotation())))
        .withTimeout(Constants.getMode().equals(Mode.SIM) ? 0.7 : time);
  }

  public static Command returnLaunchAndIndexMindfully(
      Drive drive,
      Hopper hopper,
      Kicker kicker,
      Flywheel flywheel,
      Slamtake slamtake,
      boolean launchHastily,
      Supplier<AutoQuestionResponse> launchTarget,
      BooleanSupplier readyToLeave) {
    Timer launchTimer = new Timer();
    return driveToPose(
            drive,
            () ->
                launchTarget.get().equals(AutoQuestionResponse.LEFT_BUMP)
                        || launchTarget.get().equals(AutoQuestionResponse.RIGHT_BUMP)
                    ? isLeftSide(launchTarget).getAsBoolean() ? Launch.leftBump : Launch.rightBump
                    : isLeftSide(launchTarget).getAsBoolean()
                        ? Launch.leftTrench
                        : Launch.rightTrench)
        .until(
            () ->
                xCrossed(
                    FieldConstants.LinesVertical.starting - DriveConstants.fullBaseRadius, false))
        .andThen(
            new DriveToPose(
                    drive,
                    () ->
                        LaunchCalculator.getStationaryAimedPose(
                            RobotState.getInstance().getEstimatedPose().getTranslation(), false))
                .withDeadline(
                    Commands.sequence(
                        Commands.runOnce(() -> launchTimer.restart()),
                        indexMindfully(
                                hopper,
                                kicker,
                                flywheel,
                                slamtake,
                                () ->
                                    AutoCommands.withinLaunchingTolerance(
                                        Rotation2d.fromDegrees(8.0)),
                                launchHastily)
                            .withDeadline(
                                Commands.waitUntil(
                                    () ->
                                        launchTimer.hasElapsed(
                                                launchHastily
                                                    ? AutoBuilder.depotLaunchTime
                                                    : AutoBuilder.launchTime)
                                            && readyToLeave.getAsBoolean())))));
  }

  public static Command index(Hopper hopper, Kicker kicker, Flywheel flywheel, Slamtake slamtake) {
    return Commands.waitUntil(flywheel::atGoal)
        .andThen(
            new SuppliedWaitCommand(CompactingCommands.slamLaunchDelay)
                .andThen(
                    Commands.runOnce(() -> slamtake.setSlamGoal(SlamGoal.RETRACT_SLOW)),
                    Commands.runEnd(
                            () -> slamtake.setIntakeGoal(IntakeGoal.INTAKE),
                            () -> slamtake.setIntakeGoal(IntakeGoal.STOP),
                            slamtake)
                        .asProxy()))
        .deadlineFor(
            Commands.startEnd(
                () -> {
                  hopper.setGoal(Hopper.Goal.LAUNCH);
                  kicker.setGoal(Kicker.Goal.LAUNCH);
                },
                () -> {
                  hopper.setGoal(Hopper.Goal.STOP);
                  kicker.setGoal(Kicker.Goal.STOP);
                },
                hopper,
                kicker));
  }

  // Index with additional checks
  public static Command indexMindfully(
      Hopper hopper,
      Kicker kicker,
      Flywheel flywheel,
      Slamtake slamtake,
      BooleanSupplier shouldIndex) {
    return indexMindfully(hopper, kicker, flywheel, slamtake, shouldIndex, false);
  }

  public static Command indexMindfully(
      Hopper hopper,
      Kicker kicker,
      Flywheel flywheel,
      Slamtake slamtake,
      BooleanSupplier shouldIndex,
      Boolean indexHastily) {
    return Commands.waitUntil(() -> flywheel.atGoal() && shouldIndex.getAsBoolean())
        .andThen(
            new SuppliedWaitCommand(CompactingCommands.slamLaunchDelay)
                .andThen(
                    Commands.run(
                        () ->
                            slamtake.setSlamGoal(
                                indexHastily ? SlamGoal.RETRACT_FAST : SlamGoal.RETRACT_SLOW)),
                    Commands.idle())
                .deadlineFor(
                    Commands.startEnd(
                        () -> {
                          hopper.setGoal(Hopper.Goal.LAUNCH);
                          kicker.setGoal(Kicker.Goal.LAUNCH);
                        },
                        () -> {
                          hopper.setGoal(Hopper.Goal.STOP);
                          kicker.setGoal(Kicker.Goal.STOP);
                        },
                        hopper,
                        kicker)))
        .finallyDo(() -> slamtake.setSlamGoal(SlamGoal.RETRACT));
  }

  public static Translation2d keepOutX(Bounds bounds, Translation2d translation) {
    return new Translation2d(
        translation.getX() <= bounds.minX() || translation.getX() >= bounds.maxX()
            ? translation.getX()
            : Math.abs(translation.getX() - bounds.minX())
                    < Math.abs(translation.getX() - bounds.maxX())
                ? bounds.minX()
                : bounds.maxX(),
        translation.getY());
  }

  public static Translation2d keepOutY(Bounds bounds, Translation2d translation) {
    return new Translation2d(
        translation.getX(),
        translation.getY() <= bounds.minY() || translation.getY() >= bounds.maxY()
            ? translation.getY()
            : Math.abs(translation.getY() - bounds.minY())
                    < Math.abs(translation.getY() - bounds.maxY())
                ? bounds.minY()
                : bounds.maxY());
  }

  public static Command passingCommand(
      Drive drive,
      Hopper hopper,
      Kicker kicker,
      Flywheel flywheel,
      Slamtake slamtake,
      Supplier<AutoQuestionResponse> passingMode,
      double launchTime) {
    Supplier<Pose2d> targetSupplier =
        () -> {
          // Left-Right and Hub constraints
          Bounds passingZone =
              passingMode.get().equals(AutoQuestionResponse.BOTH)
                  ? new Bounds(0.0, FieldConstants.fieldLength, 0.0, FieldConstants.fieldWidth)
                  : AllianceFlipUtil.apply(
                      passingMode.get().equals(AutoQuestionResponse.LEFT_CLOSE)
                          ? new Bounds(
                              FieldConstants.LinesVertical.neutralZoneNear,
                              FieldConstants.LinesVertical.neutralZoneFar,
                              FieldConstants.LinesHorizontal.leftBumpEnd,
                              FieldConstants.fieldWidth)
                          : new Bounds(
                              FieldConstants.LinesVertical.neutralZoneNear,
                              FieldConstants.LinesVertical.neutralZoneFar,
                              0.0,
                              FieldConstants.LinesHorizontal.rightBumpStart));

          return LaunchCalculator.getStationaryAimedPose(
              // Stay on close side of neutral zone
              keepOutX(
                  AllianceFlipUtil.apply(
                      new Bounds(
                          FieldConstants.fieldLength / 2.0 + DriveConstants.fullApothemX - 0.2,
                          FieldConstants.fieldLength,
                          0.0,
                          FieldConstants.fieldWidth)),
                  passingZone.clamp(RobotState.getInstance().getEstimatedPose().getTranslation())),
              false);
        };

    return new DriveToPose(drive, targetSupplier)
        .raceWith(
            Commands.sequence(
                AutoCommands.waitUntilWithinTolerance(
                    targetSupplier, 0.5, Rotation2d.fromDegrees(15)),
                index(hopper, kicker, flywheel, slamtake).withTimeout(launchTime)));
  }

  public static Bounds expandBounds(Bounds a, Bounds b) {
    return new Bounds(
        Math.min(a.minX(), b.minX()),
        Math.max(a.maxX(), b.maxX()),
        Math.min(a.minY(), b.minY()),
        Math.max(a.maxY(), b.maxY()));
  }

  public static Bounds getDynamicBounds(
      Supplier<AutoQuestionResponse> returnSide, boolean ethical) {
    Set<Translation2d> otherRobots = ObjectDetection.getInstance().getRobotTranslations();

    // Define quadrants of the neutral zone
    double minX = FieldConstants.LinesVertical.neutralZoneNear;
    double maxX = FieldConstants.LinesVertical.neutralZoneFar;
    double midX = (minX + maxX) / 2.0;
    double minY = 0.0;
    double maxY = FieldConstants.fieldWidth;
    double midY = (minY + maxY) / 2.0;
    Bounds[] quadrants = {
      new Bounds(minX, midX, minY, midY),
      new Bounds(minX, midX, midY, maxY),
      new Bounds(midX, maxX, minY, midY),
      new Bounds(midX, maxX, midY, maxY)
    };

    // Create dynamic names for quadrants based on alliance color and return side
    int homeIndex =
        AllianceFlipUtil.shouldFlip()
            ? (returnSide.get().equals(AutoQuestionResponse.LEFT_BUMP)
                    || returnSide.get().equals(AutoQuestionResponse.LEFT_TRENCH)
                    || returnSide.get().equals(AutoQuestionResponse.LEFT_NO_TRENCH)
                ? 2
                : 3)
            : (returnSide.get().equals(AutoQuestionResponse.LEFT_BUMP)
                    || returnSide.get().equals(AutoQuestionResponse.LEFT_TRENCH)
                    || returnSide.get().equals(AutoQuestionResponse.LEFT_NO_TRENCH)
                ? 1
                : 0);
    Logger.recordOutput("AutoCommands/DynamicBounds/HomeIndex", homeIndex);

    Bounds homeBounds = quadrants[homeIndex];
    Bounds allyAdjacent = quadrants[homeIndex ^ 1];
    Bounds opponentAdjacent = quadrants[homeIndex ^ 2];
    Bounds opponentFar = quadrants[homeIndex ^ 3];

    // Expand into other side of near neutral zone if empty
    if (!otherRobots.stream().anyMatch(robot -> allyAdjacent.contains(robot))) {
      homeBounds = expandBounds(homeBounds, allyAdjacent);
      if (!otherRobots.stream().anyMatch(robot -> opponentAdjacent.contains(robot))
          && !otherRobots.stream().anyMatch(robot -> opponentFar.contains(robot))
          && !ethical) {
        homeBounds = expandBounds(homeBounds, opponentFar);
      }
    } else {
      if (!otherRobots.stream().anyMatch(robot -> opponentAdjacent.contains(robot)) && !ethical) {
        homeBounds = expandBounds(homeBounds, opponentAdjacent);
      }
    }

    Logger.recordOutput("AutoCommands/DynamicBounds/Outline", homeBounds.sides());

    // Shift bounds past centerline
    return new Bounds(
        homeBounds.minX(),
        Math.max(
            homeBounds.maxX(), FieldConstants.LinesVertical.center + DriveConstants.fullApothemX),
        homeBounds.minY(),
        homeBounds.maxY());
  }

  public static BooleanSupplier isLeftSide(AutoQuestionResponse side) {
    return () ->
        side.equals(AutoQuestionResponse.LEFT)
            || side.equals(AutoQuestionResponse.LEFT_BUMP)
            || side.equals(AutoQuestionResponse.LEFT_TRENCH)
            || side.equals(AutoQuestionResponse.LEFT_NO_TRENCH);
  }

  public static BooleanSupplier isLeftSide(Supplier<AutoQuestionResponse> side) {
    return () ->
        side.get().equals(AutoQuestionResponse.LEFT)
            || side.get().equals(AutoQuestionResponse.LEFT_BUMP)
            || side.get().equals(AutoQuestionResponse.LEFT_TRENCH)
            || side.get().equals(AutoQuestionResponse.LEFT_TRENCH_OFFSET)
            || side.get().equals(AutoQuestionResponse.LEFT_NO_TRENCH);
  }

  public static boolean xCrossed(double xPosition, boolean towardsCenter) {
    Pose2d robotPose = RobotState.getInstance().getEstimatedPose();
    if (AllianceFlipUtil.shouldFlip()) {
      if (towardsCenter) {
        return robotPose.getX() < FieldConstants.fieldLength - xPosition;
      } else {
        return robotPose.getX() > FieldConstants.fieldLength - xPosition;
      }
    } else {
      if (towardsCenter) {
        return robotPose.getX() > xPosition;
      } else {
        return robotPose.getX() < xPosition;
      }
    }
  }

  public static boolean yCrossed(double yPosition, boolean towardsLeft) {
    Pose2d robotPose = RobotState.getInstance().getEstimatedPose();
    if (AllianceFlipUtil.shouldFlip()) {
      if (towardsLeft) {
        return robotPose.getY() < FieldConstants.fieldWidth - yPosition;
      } else {
        return robotPose.getY() > FieldConstants.fieldWidth - yPosition;
      }
    } else {
      if (towardsLeft) {
        return robotPose.getY() > yPosition;
      } else {
        return robotPose.getY() < yPosition;
      }
    }
  }

  public static boolean withinTolerance(
      Pose2d target, double translationalTolerance, Rotation2d rotationalTolerance) {
    Pose2d robotPose = RobotState.getInstance().getEstimatedPose();
    return robotPose.getTranslation().getDistance(AllianceFlipUtil.apply(target.getTranslation()))
            < translationalTolerance
        && Math.abs(
                robotPose
                    .getRotation()
                    .minus(AllianceFlipUtil.apply(target.getRotation()))
                    .getRadians())
            < rotationalTolerance.getRadians();
  }

  public static boolean withinLaunchingTolerance(Rotation2d rotationalTolerance) {
    return Math.abs(
            RobotState.getInstance()
                .getRotation()
                .minus(LaunchCalculator.getInstance().getParameters().driveAngle())
                .getRadians())
        <= rotationalTolerance.getRadians();
  }

  public static Command waitUntilXCrossed(double xPosition, boolean towardsCenter) {
    return Commands.waitUntil(() -> xCrossed(xPosition, towardsCenter));
  }

  public static Command waitUntilYCrossed(double yPosition, boolean towardsLeft) {
    return Commands.waitUntil(() -> yCrossed(yPosition, towardsLeft));
  }

  public static Command waitUntilWithinTolerance(
      Pose2d target, double translationalTolerance, Rotation2d rotationalTolerance) {
    return Commands.waitUntil(
        () -> withinTolerance(target, translationalTolerance, rotationalTolerance));
  }

  public static Command waitUntilWithinTolerance(
      Supplier<Pose2d> target, double translationalTolerance, Rotation2d rotationalTolerance) {
    return Commands.waitUntil(
        () -> withinTolerance(target.get(), translationalTolerance, rotationalTolerance));
  }

  public static Command driveToPose(Drive drive, Supplier<Pose2d> target) {
    return new DriveToPose(drive, () -> AllianceFlipUtil.apply(target.get()));
  }

  public static Command driveToPoseWhileAiming(Drive drive, Supplier<Pose2d> target) {
    return new DriveToPose(
        drive,
        () -> AllianceFlipUtil.apply(target.get()),
        () -> Optional.of(launchOnTheMoveOmega()));
  }

  /**
   * Resets pose accounting for alliance color.
   *
   * @param pose Pose to reset to.
   */
  public static Command resetPose(Pose2d pose) {
    return Commands.runOnce(
        () -> {
          RobotState.getInstance().resetPose(AllianceFlipUtil.apply(pose));
        });
  }

  public static Command resetPose(Supplier<Pose2d> pose) {
    return Commands.runOnce(
        () -> {
          RobotState.getInstance().resetPose(AllianceFlipUtil.apply(pose.get()));
        });
  }

  public static Command resetStartingPose(Supplier<AutoQuestionResponse> startPosition) {
    return resetPose(
        () -> {
          if (startPosition.get().equals(AutoQuestionResponse.LEFT_TRENCH)) {
            return new Pose2d(AutoFieldConstants.Trench.leftStart, Rotation2d.fromDegrees(-90));
          } else if (startPosition.get().equals(AutoQuestionResponse.LEFT_TRENCH_OFFSET)) {
            return new Pose2d(
                AutoFieldConstants.Trench.leftStartOffset, Rotation2d.fromDegrees(-90));
          } else if (startPosition.get().equals(AutoQuestionResponse.RIGHT_TRENCH)) {
            return new Pose2d(AutoFieldConstants.Trench.rightStart, Rotation2d.fromDegrees(90));
          } else if (startPosition.get().equals(AutoQuestionResponse.RIGHT_TRENCH_OFFSET)) {
            return new Pose2d(
                AutoFieldConstants.Trench.rightStartOffset, Rotation2d.fromDegrees(90));
          } else {
            return new Pose2d(startPosition.get().getTranslation(), Rotation2d.kZero);
          }
        });
  }
}
