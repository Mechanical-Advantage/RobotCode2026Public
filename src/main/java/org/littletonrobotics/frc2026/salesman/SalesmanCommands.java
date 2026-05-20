// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2026.salesman;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.Optional;
import java.util.function.Supplier;
import lombok.experimental.ExtensionMethod;
import org.littletonrobotics.frc2026.Constants;
import org.littletonrobotics.frc2026.RobotState;
import org.littletonrobotics.frc2026.subsystems.drive.Drive;
import org.littletonrobotics.frc2026.subsystems.drive.DriveConstants;
import org.littletonrobotics.frc2026.util.BeachedUtil;
import org.littletonrobotics.frc2026.util.LoggedTunableNumber;
import org.littletonrobotics.frc2026.util.geometry.AllianceFlipUtil;
import org.littletonrobotics.frc2026.util.geometry.Bounds;
import org.littletonrobotics.frc2026.util.geometry.GeomUtil;
import org.littletonrobotics.junction.Logger;

@ExtensionMethod({GeomUtil.class})
public class SalesmanCommands {
  private static final LoggedTunableNumber autoIntakeLookaheadT =
      new LoggedTunableNumber("SalesmanCommands/AutoIntakeLookaheadT", 0.25);
  private static final LoggedTunableNumber autoIntakeMaxLinearSpeed =
      new LoggedTunableNumber("SalesmanCommands/AutoIntakeMaxLinearSpeed", 3.0);
  private static final LoggedTunableNumber autoIntakeMinLinearSpeed =
      new LoggedTunableNumber("SalesmanCommands/AutoIntakeMinLinearSpeed", 1.8);
  private static final LoggedTunableNumber autoIntakeMaxReward =
      new LoggedTunableNumber("SalesmanCommands/AutoIntakeMaxReward", 70.0);
  private static final LoggedTunableNumber autoIntakeMinReward =
      new LoggedTunableNumber("SalesmanCommands/AutoIntakeMinReward", 15.0);
  private static final LoggedTunableNumber autoIntakeThetaMarginDeg =
      new LoggedTunableNumber("SalesmanCommands/AutoIntakeThetaMarginDeg", 30.0);
  private static final LoggedTunableNumber autoIntakeLinearKp =
      new LoggedTunableNumber("SalesmanCommands/AutoIntakeLinearKp", 6.0);
  private static final LoggedTunableNumber autoIntakeLinearKd =
      new LoggedTunableNumber("SalesmanCommands/AutoIntakeLinearKd", 0.0);
  private static final LoggedTunableNumber autoIntakeThetaKp =
      new LoggedTunableNumber("SalesmanCommands/AutoIntakeThetaKp", 8.0);
  private static final LoggedTunableNumber autoIntakeThetaKd =
      new LoggedTunableNumber("SalesmanCommands/AutoIntakeThetaKd", 0.0);

  private SalesmanCommands() {}

  public static Command autoIntake(Drive drive, SalesmanSolver salesmanSolver, Bounds bounds) {
    return autoIntake(drive, salesmanSolver, () -> bounds);
  }

  @SuppressWarnings("resource")
  public static Command autoIntake(
      Drive drive, SalesmanSolver salesmanSolver, Supplier<Bounds> bounds) {
    PIDController xController =
        new PIDController(
            autoIntakeLinearKp.get(), 0, autoIntakeLinearKd.get(), Constants.loopPeriodSecs);
    PIDController yController =
        new PIDController(
            autoIntakeLinearKp.get(), 0, autoIntakeLinearKd.get(), Constants.loopPeriodSecs);
    PIDController thetaController =
        new PIDController(
            autoIntakeThetaKp.get(), 0, autoIntakeThetaKd.get(), Constants.loopPeriodSecs);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    return Commands.run(
            () -> {
              // Update tunable values
              if (autoIntakeLinearKp.hasChanged(xController.hashCode())
                  || autoIntakeLinearKd.hasChanged(xController.hashCode())
                  || autoIntakeThetaKp.hasChanged(thetaController.hashCode())
                  || autoIntakeThetaKd.hasChanged(thetaController.hashCode())) {
                xController.setPID(autoIntakeLinearKp.get(), 0, autoIntakeLinearKd.get());
                yController.setPID(autoIntakeLinearKp.get(), 0, autoIntakeLinearKd.get());
                thetaController.setPID(autoIntakeThetaKp.get(), 0, autoIntakeThetaKd.get());
              }

              // Set bounds
              var boundsValue = bounds.get();
              salesmanSolver.setBounds(boundsValue);
              boundsValue = AllianceFlipUtil.apply(boundsValue);

              // Get current and target pose
              var robotCurrentPose = RobotState.getInstance().getEstimatedPose();
              var intakeCurrentPose = RobotState.getInstance().getIntakePose();
              var intakeTargetPose =
                  salesmanSolver.getTrajectoryTarget(intakeCurrentPose, autoIntakeLookaheadT.get());

              // Adjust intake theta target based on margin
              Rotation2d thetaTarget = intakeTargetPose.getRotation();
              Rotation2d thetaError =
                  intakeTargetPose.getRotation().minus(intakeCurrentPose.getRotation());
              if (Math.abs(thetaError.getDegrees()) < autoIntakeThetaMarginDeg.get()) {
                thetaTarget = intakeCurrentPose.getRotation();
              } else if (thetaError.getDegrees() >= autoIntakeThetaMarginDeg.get()) {
                thetaTarget =
                    intakeTargetPose
                        .getRotation()
                        .minus(Rotation2d.fromDegrees(autoIntakeThetaMarginDeg.get()));
              } else if (thetaError.getDegrees() <= -autoIntakeThetaMarginDeg.get()) {
                thetaTarget =
                    intakeTargetPose
                        .getRotation()
                        .plus(Rotation2d.fromDegrees(autoIntakeThetaMarginDeg.get()));
              }

              // Get robot target pose
              var robotTargetPose =
                  new Pose2d(intakeTargetPose.getTranslation(), thetaTarget)
                      .transformBy(
                          new Translation2d(-DriveConstants.intakeReferenceX, 0.0).toTransform2d());
              var robotBounds =
                  new Bounds(
                      boundsValue.minX() + DriveConstants.driveBaseRadius,
                      boundsValue.maxX() - DriveConstants.driveBaseRadius,
                      boundsValue.minY() + DriveConstants.driveBaseRadius,
                      boundsValue.maxY() - DriveConstants.driveBaseRadius);
              robotTargetPose =
                  new Pose2d(
                      robotBounds.clamp(robotTargetPose.getTranslation()),
                      robotTargetPose.getRotation());

              // Calculate linear speed
              var linearSpeed =
                  new Translation2d(
                      xController.calculate(robotCurrentPose.getX(), robotTargetPose.getX()),
                      yController.calculate(robotCurrentPose.getY(), robotTargetPose.getY()));
              double calculatedSpeed =
                  MathUtil.interpolate(
                      autoIntakeMaxLinearSpeed.get(),
                      autoIntakeMinLinearSpeed.get(),
                      (salesmanSolver.getReward() - autoIntakeMinReward.get())
                          / (autoIntakeMaxReward.get() - autoIntakeMinReward.get()));
              Logger.recordOutput("SalesmanCommands/IntakeSpeed", calculatedSpeed, "m/s");
              if (linearSpeed.getNorm() > calculatedSpeed) {
                linearSpeed = linearSpeed.times(calculatedSpeed / linearSpeed.getNorm());
              }

              // Calculate theta speed
              double thetaSpeed =
                  thetaController.calculate(
                      MathUtil.inputModulus(
                          robotCurrentPose.getRotation().getRadians(), -Math.PI, Math.PI),
                      MathUtil.inputModulus(
                          robotTargetPose.getRotation().getRadians(), -Math.PI, Math.PI));

              // Send velocity
              Optional<ChassisSpeeds> beachedSpeeds = BeachedUtil.getInstance().getBeachedSpeeds();
              if (beachedSpeeds.isPresent()) {
                drive.runVelocity(beachedSpeeds.get());
              } else {
                drive.runVelocity(
                    ChassisSpeeds.fromFieldRelativeSpeeds(
                        linearSpeed.getX(),
                        linearSpeed.getY(),
                        thetaSpeed,
                        robotCurrentPose.getRotation()));
              }
            },
            drive,
            salesmanSolver)
        .beforeStarting(
            () -> {
              salesmanSolver.setFollowingTrajectory(true);
              xController.reset();
              yController.reset();
              thetaController.reset();
            })
        .finallyDo(
            () -> {
              salesmanSolver.setFollowingTrajectory(false);
              drive.stop();
              salesmanSolver.resetBounds();
            });
  }
}
