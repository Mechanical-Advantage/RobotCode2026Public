// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2026.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.Supplier;
import lombok.Getter;
import org.littletonrobotics.frc2026.Constants;
import org.littletonrobotics.frc2026.RobotState;
import org.littletonrobotics.frc2026.subsystems.drive.Drive;
import org.littletonrobotics.frc2026.util.LoggedTunableNumber;
import org.littletonrobotics.frc2026.util.geometry.GeomUtil;
import org.littletonrobotics.junction.Logger;

public class DriveToPose extends Command {
  private static final LoggedTunableNumber drivekP = new LoggedTunableNumber("DriveToPose/DrivekP");
  private static final LoggedTunableNumber drivekD = new LoggedTunableNumber("DriveToPose/DrivekD");
  private static final LoggedTunableNumber thetakP = new LoggedTunableNumber("DriveToPose/ThetakP");
  private static final LoggedTunableNumber thetakD = new LoggedTunableNumber("DriveToPose/ThetakD");
  private static final LoggedTunableNumber driveMaxVelocity =
      new LoggedTunableNumber("DriveToPose/DriveMaxVelocity");
  private static final LoggedTunableNumber driveMaxAcceleration =
      new LoggedTunableNumber("DriveToPose/DriveMaxAcceleration");
  private static final LoggedTunableNumber thetaMaxVelocity =
      new LoggedTunableNumber("DriveToPose/ThetaMaxVelocity");
  private static final LoggedTunableNumber thetaMaxAcceleration =
      new LoggedTunableNumber("DriveToPose/ThetaMaxAcceleration");
  private static final LoggedTunableNumber driveTolerance =
      new LoggedTunableNumber("DriveToPose/DriveTolerance");
  private static final LoggedTunableNumber thetaTolerance =
      new LoggedTunableNumber("DriveToPose/ThetaTolerance");
  private static final LoggedTunableNumber thetaFFMinError =
      new LoggedTunableNumber("DriveToPose/ThetaFFMinError");
  private static final LoggedTunableNumber thetaFFMaxError =
      new LoggedTunableNumber("DriveToPose/ThetaFFMaxError");
  private static final LoggedTunableNumber setpointMinVelocity =
      new LoggedTunableNumber("DriveToPose/SetpointMinVelocity");
  private static final LoggedTunableNumber minDistanceVelocityCorrection =
      new LoggedTunableNumber("DriveToPose/MinDistanceVelocityCorrection");
  private static final LoggedTunableNumber linearFFMinRadius =
      new LoggedTunableNumber("DriveToPose/LinearFFMinRadius");
  private static final LoggedTunableNumber linearFFMaxRadius =
      new LoggedTunableNumber("DriveToPose/LinearFFMaxRadius");

  static {
    drivekP.initDefault(1.8);
    drivekD.initDefault(0.0);
    thetakP.initDefault(5.0);
    thetakD.initDefault(0.5);

    driveMaxVelocity.initDefault(4.0);
    driveMaxAcceleration.initDefault(3.5);

    thetaMaxVelocity.initDefault(Units.degreesToRadians(500.0));
    thetaMaxAcceleration.initDefault(8.0);

    driveTolerance.initDefault(0.01);
    thetaTolerance.initDefault(Units.degreesToRadians(1.0));
    linearFFMinRadius.initDefault(0.01);
    linearFFMaxRadius.initDefault(0.05);
    thetaFFMinError.initDefault(0.0);
    thetaFFMaxError.initDefault(0.0);

    setpointMinVelocity.initDefault(-0.5);
    minDistanceVelocityCorrection.initDefault(0.01);
  }

  private final Drive drive;
  private final Supplier<Pose2d> target;

  private TrapezoidProfile driveProfile;
  private final PIDController driveController =
      new PIDController(0.0, 0.0, 0.0, Constants.loopPeriodSecs);
  private final ProfiledPIDController thetaController =
      new ProfiledPIDController(
          0.0, 0.0, 0.0, new TrapezoidProfile.Constraints(0.0, 0.0), Constants.loopPeriodSecs);

  private Translation2d lastSetpointTranslation = Translation2d.kZero;
  private Translation2d lastSetpointVelocity = Translation2d.kZero;
  private Rotation2d lastGoalRotation = Rotation2d.kZero;
  private double lastTime = 0.0;
  private double driveErrorAbs = 0.0;
  private double thetaErrorAbs = 0.0;
  @Getter private boolean running = false;

  public DriveToPose(Drive drive, Supplier<Pose2d> target) {
    this.drive = drive;
    this.target = target;
    if (drive != null) addRequirements(drive);

    // Enable continuous input for theta controller
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void initialize() {
    Pose2d currentPose = RobotState.getInstance().getEstimatedPose();
    Pose2d targetPose = target.get();
    ChassisSpeeds fieldVelocity = RobotState.getInstance().getFieldVelocity();
    Translation2d linearFieldVelocity =
        new Translation2d(fieldVelocity.vxMetersPerSecond, fieldVelocity.vyMetersPerSecond);

    driveProfile =
        new TrapezoidProfile(
            new TrapezoidProfile.Constraints(driveMaxVelocity.get(), driveMaxAcceleration.get()));

    driveController.reset();
    thetaController.reset(
        currentPose.getRotation().getRadians(), fieldVelocity.omegaRadiansPerSecond);
    lastSetpointTranslation = currentPose.getTranslation();
    lastSetpointVelocity = linearFieldVelocity;
    lastGoalRotation = targetPose.getRotation();
    lastTime = Timer.getTimestamp();
  }

  @Override
  public void execute() {
    running = true;

    // Update from tunable numbers
    if (driveTolerance.hasChanged(hashCode())
        || thetaTolerance.hasChanged(hashCode())
        || drivekP.hasChanged(hashCode())
        || drivekD.hasChanged(hashCode())
        || thetakP.hasChanged(hashCode())
        || thetakD.hasChanged(hashCode())) {
      driveController.setP(drivekP.get());
      driveController.setD(drivekD.get());
      driveController.setTolerance(driveTolerance.get());
      thetaController.setP(thetakP.get());
      thetaController.setD(thetakD.get());
      thetaController.setTolerance(thetaTolerance.get());
    }

    driveProfile =
        new TrapezoidProfile(
            new TrapezoidProfile.Constraints(driveMaxVelocity.get(), driveMaxAcceleration.get()));
    thetaController.setConstraints(
        new TrapezoidProfile.Constraints(thetaMaxVelocity.get(), thetaMaxAcceleration.get()));

    // Get current pose and target pose
    Pose2d currentPose = RobotState.getInstance().getEstimatedPose();
    Pose2d targetPose = target.get();

    Pose2d poseError = currentPose.relativeTo(targetPose);
    driveErrorAbs = poseError.getTranslation().getNorm();
    thetaErrorAbs = Math.abs(poseError.getRotation().getRadians());
    double linearFFScaler =
        MathUtil.clamp(
            (driveErrorAbs - linearFFMinRadius.get())
                / (linearFFMaxRadius.get() - linearFFMinRadius.get()),
            0.0,
            1.0);
    double thetaFFScaler =
        MathUtil.clamp(
            (Units.radiansToDegrees(thetaErrorAbs) - thetaFFMinError.get())
                / (thetaFFMaxError.get() - thetaFFMinError.get()),
            0.0,
            1.0);

    // Calculate drive velocity
    // Calculate setpoint velocity towards target pose
    var direction = targetPose.getTranslation().minus(lastSetpointTranslation).toVector();
    double setpointVelocity =
        direction.norm()
                <= minDistanceVelocityCorrection
                    .get() // Don't calculate velocity in direction when really close
            ? lastSetpointVelocity.getNorm()
            : lastSetpointVelocity.toVector().dot(direction) / direction.norm();
    setpointVelocity = Math.max(setpointVelocity, setpointMinVelocity.get());
    State driveSetpoint =
        driveProfile.calculate(
            Constants.loopPeriodSecs,
            new State(
                direction.norm(), -setpointVelocity), // Use negative as profile has zero at target
            new State(0.0, 0.0));
    double driveVelocityScalar =
        driveController.calculate(driveErrorAbs, driveSetpoint.position)
            + driveSetpoint.velocity * linearFFScaler;
    if (driveErrorAbs < driveController.getErrorTolerance()) driveVelocityScalar = 0.0;
    Rotation2d targetToCurrentAngle =
        currentPose.getTranslation().minus(targetPose.getTranslation()).getAngle();

    Translation2d driveVelocity = new Translation2d(driveVelocityScalar, targetToCurrentAngle);
    lastSetpointTranslation =
        new Pose2d(targetPose.getTranslation(), targetToCurrentAngle)
            .transformBy(GeomUtil.toTransform2d(driveSetpoint.position, 0.0))
            .getTranslation();
    lastSetpointVelocity = new Translation2d(driveSetpoint.velocity, targetToCurrentAngle);

    // Calculate theta speed
    double thetaSetpointVelocity =
        Math.abs((targetPose.getRotation().minus(lastGoalRotation)).getDegrees()) < 10.0
            ? (targetPose.getRotation().minus(lastGoalRotation)).getRadians()
                / (Timer.getTimestamp() - lastTime)
            : thetaController.getSetpoint().velocity;
    double thetaVelocity =
        thetaController.calculate(
                currentPose.getRotation().getRadians(),
                new State(targetPose.getRotation().getRadians(), thetaSetpointVelocity))
            + thetaController.getSetpoint().velocity * thetaFFScaler;
    if (thetaErrorAbs < thetaController.getPositionTolerance()) thetaVelocity = 0.0;
    lastGoalRotation = targetPose.getRotation();
    lastTime = Timer.getTimestamp();

    // Command speeds
    if (drive != null) {
      drive.runVelocity(
          ChassisSpeeds.fromFieldRelativeSpeeds(
              driveVelocity.getX(),
              driveVelocity.getY(),
              thetaVelocity,
              currentPose.getRotation()));
    }

    // Log data
    Logger.recordOutput("DriveToPose/DistanceMeasured", driveErrorAbs);
    Logger.recordOutput("DriveToPose/DistanceSetpoint", driveSetpoint.position);
    Logger.recordOutput("DriveToPose/DistanceSetpointVelocity", driveSetpoint.velocity);
    Logger.recordOutput("DriveToPose/ThetaMeasured", currentPose.getRotation().getRadians());
    Logger.recordOutput("DriveToPose/ThetaSetpoint", thetaController.getSetpoint().position);
    Logger.recordOutput(
        "DriveToPose/ThetaSetpointVelocity", thetaController.getSetpoint().velocity);
    Logger.recordOutput(
        "DriveToPose/Setpoint",
        new Pose2d[] {
          new Pose2d(
              lastSetpointTranslation,
              Rotation2d.fromRadians(thetaController.getSetpoint().position))
        });
    Logger.recordOutput("DriveToPose/Goal", new Pose2d[] {targetPose});
  }

  @Override
  public void end(boolean interrupted) {
    if (drive != null) drive.stop();
    running = false;

    // Clear logs
    Logger.recordOutput("DriveToPose/Setpoint", new Pose2d[] {});
    Logger.recordOutput("DriveToPose/Goal", new Pose2d[] {});
  }

  /** Checks if the robot pose is within the allowed drive and theta tolerances. */
  public boolean withinTolerance(double driveTolerance, Rotation2d thetaTolerance) {
    return running
        && Math.abs(driveErrorAbs) < driveTolerance
        && Math.abs(thetaErrorAbs) < thetaTolerance.getRadians();
  }
}
