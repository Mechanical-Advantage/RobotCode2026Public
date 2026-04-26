// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2026.commands;

import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.Arrays;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.frc2026.Constants;
import org.littletonrobotics.frc2026.FieldConstants;
import org.littletonrobotics.frc2026.RobotState;
import org.littletonrobotics.frc2026.subsystems.drive.Drive;
import org.littletonrobotics.frc2026.util.BeachedUtil;
import org.littletonrobotics.frc2026.util.LoggedTunableNumber;
import org.littletonrobotics.frc2026.util.geometry.AllianceFlipUtil;
import org.littletonrobotics.frc2026.util.geometry.VerticalFlipUtil;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class DriveTrajectory extends Command {
  private static final LoggedTunableNumber linearkP =
      new LoggedTunableNumber("DriveTrajectory/LinearkP", 8.0);
  private static final LoggedTunableNumber linearkD =
      new LoggedTunableNumber("DriveTrajectory/LinearkD", 0.0);
  private static final LoggedTunableNumber thetakP =
      new LoggedTunableNumber("DriveTrajectory/ThetakP", 4.0);
  private static final LoggedTunableNumber thetakD =
      new LoggedTunableNumber("DriveTrajectory/ThetakD", 0.0);

  private final Timer timer = new Timer();
  private final Trajectory<SwerveSample> trajectory;
  private final Supplier<Optional<Double>> omegaOverride;
  private final Drive drive;
  private final BooleanSupplier mirror;

  private final PIDController xController;
  private final PIDController yController;
  private final PIDController thetaController;

  public DriveTrajectory(
      Trajectory<SwerveSample> trajectory,
      Supplier<Optional<Double>> omegaOverride,
      Drive drive,
      BooleanSupplier mirror) {
    this.drive = drive;
    this.trajectory = trajectory;
    this.omegaOverride = omegaOverride;
    this.mirror = mirror;
    xController = new PIDController(linearkP.get(), 0, linearkD.get(), Constants.loopPeriodSecs);
    yController = new PIDController(linearkP.get(), 0, linearkD.get(), Constants.loopPeriodSecs);
    thetaController = new PIDController(thetakP.get(), 0, thetakD.get(), Constants.loopPeriodSecs);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    addRequirements(drive);
  }

  public DriveTrajectory(
      Trajectory<SwerveSample> trajectory, Supplier<Optional<Double>> omegaOverride, Drive drive) {
    this(trajectory, omegaOverride, drive, () -> false);
  }

  public DriveTrajectory(Trajectory<SwerveSample> trajectory, Drive drive, BooleanSupplier mirror) {
    this(trajectory, () -> Optional.empty(), drive, mirror);
  }

  public DriveTrajectory(Trajectory<SwerveSample> trajectory, Drive drive) {
    this(trajectory, () -> Optional.empty(), drive, () -> false);
  }

  @Override
  public void initialize() {
    timer.restart();
    xController.reset();
    yController.reset();
    thetaController.reset();

    Logger.recordOutput(
        "DriveTrajectory/Trajectory",
        Arrays.stream(trajectory.getPoses())
            .map(AllianceFlipUtil::apply)
            .map(pose -> mirror.getAsBoolean() ? VerticalFlipUtil.apply(pose) : pose)
            .toArray(Pose2d[]::new));
    Logger.recordOutput("DriveTrajectory/Mirror", mirror.getAsBoolean());
  }

  @Override
  public void execute() {
    if (linearkP.hasChanged(hashCode())
        || linearkD.hasChanged(hashCode())
        || thetakP.hasChanged(hashCode())
        || thetakD.hasChanged(hashCode())) {
      xController.setPID(linearkP.get(), 0, linearkD.get());
      yController.setPID(linearkP.get(), 0, linearkD.get());
      thetaController.setPID(thetakP.get(), 0, thetakD.get());
    }

    Pose2d currentPose = RobotState.getInstance().getEstimatedPose();
    SwerveSample trajectoryState =
        trajectory.sampleAt(timer.get(), AllianceFlipUtil.shouldFlip()).get();

    SwerveSample desiredState =
        mirror.getAsBoolean() ? VerticalFlipUtil.apply(trajectoryState) : trajectoryState;

    double xOutput =
        xController.calculate(currentPose.getX(), desiredState.getPose().getX()) + desiredState.vx;
    double yOutput =
        yController.calculate(currentPose.getY(), desiredState.getPose().getY()) + desiredState.vy;
    double thetaOutput =
        omegaOverride
            .get()
            .orElseGet(
                () ->
                    thetaController.calculate(
                            currentPose.getRotation().getRadians(),
                            desiredState.getPose().getRotation().getRadians())
                        + desiredState.omega);
    boolean nearDepot =
        RobotState.getInstance().getEstimatedPose().getX() < 2.0
            || RobotState.getInstance().getEstimatedPose().getX()
                > FieldConstants.fieldLength - 2.0;
    Optional<ChassisSpeeds> beachedSpeeds = BeachedUtil.getInstance().getBeachedSpeeds();
    if (beachedSpeeds.isPresent() && !nearDepot) {
      drive.runVelocity(beachedSpeeds.get());
    } else {
      drive.runVelocity(
          ChassisSpeeds.fromFieldRelativeSpeeds(
              new ChassisSpeeds(xOutput, yOutput, thetaOutput), currentPose.getRotation()));
    }

    RobotState.getInstance()
        .setRobotSetpointVelocity(
            ChassisSpeeds.discretize(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                    new ChassisSpeeds(desiredState.vx, desiredState.vy, desiredState.omega),
                    currentPose.getRotation()),
                Constants.loopPeriodSecs));

    Logger.recordOutput("DriveTrajectory/Setpoint/Pose", desiredState.getPose());
    Logger.recordOutput(
        "DriveTrajectory/Setpoint/Speeds",
        new ChassisSpeeds(desiredState.vx, desiredState.vy, desiredState.omega));
    Logger.recordOutput(
        "DriveTrajectory/TranslationError",
        currentPose.getTranslation().getDistance(desiredState.getPose().getTranslation()));
    Logger.recordOutput(
        "DriveTrajectory/RotationError",
        currentPose.getRotation().minus(desiredState.getPose().getRotation()).getDegrees());
    Logger.recordOutput("DriveTrajectory/OmegaOverrideActive", omegaOverride.get().isPresent());
  }

  @Override
  @AutoLogOutput(key = "DriveTrajectory/isFinished")
  public boolean isFinished() {
    return timer.hasElapsed(trajectory.getTotalTime());
  }

  @Override
  public void end(boolean interrupted) {
    drive.stop();
  }
}
