// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2026.salesman;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import lombok.Setter;
import org.littletonrobotics.frc2026.FieldConstants;
import org.littletonrobotics.frc2026.ObjectDetection;
import org.littletonrobotics.frc2026.RobotState;
import org.littletonrobotics.frc2026.subsystems.drive.DriveConstants;
import org.littletonrobotics.frc2026.util.FullSubsystem;
import org.littletonrobotics.frc2026.util.LoggedTracer;
import org.littletonrobotics.frc2026.util.geometry.AllianceFlipUtil;
import org.littletonrobotics.frc2026.util.geometry.Bounds;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class SalesmanSolver extends FullSubsystem {
  public static final double lookaheadTMax = 3.0;
  public static final double lookaheadTStep = 0.01;
  public static final double newTrajPositionTolerance = 1.0;
  public static final double newTrajVelocityTolerance = 1.0;
  public static final Bounds defaultBounds =
      new Bounds(
          FieldConstants.LinesVertical.neutralZoneNear,
          FieldConstants.LinesVertical.center + DriveConstants.driveBaseRadius,
          0.0,
          FieldConstants.fieldWidth);
  public static final Bounds leftBounds =
      new Bounds(
          FieldConstants.LinesVertical.neutralZoneNear,
          FieldConstants.LinesVertical.center + DriveConstants.driveBaseRadius,
          FieldConstants.fieldWidth / 2.0 - DriveConstants.fullApothemX,
          FieldConstants.fieldWidth);
  public static final Bounds rightBounds =
      new Bounds(
          FieldConstants.LinesVertical.neutralZoneNear,
          FieldConstants.LinesVertical.center + DriveConstants.driveBaseRadius,
          0.0,
          FieldConstants.fieldWidth / 2.0 + DriveConstants.fullApothemX);

  private final SalesmanSolverIO io;
  private final SalesmanSolverIOInputsAutoLogged inputs = new SalesmanSolverIOInputsAutoLogged();

  @Setter private Bounds bounds = defaultBounds;
  private long trajSeqnum = 0;
  private double[] trajXCoeffs = new double[3];
  private double[] trajYCoeffs = new double[3];
  private double trajReward = 0.0;
  @Setter() private boolean followingTrajectory = false;

  public SalesmanSolver(SalesmanSolverIO io) {
    this.io = io;
  }

  /** Reset the bounds to the default. */
  public void resetBounds() {
    bounds = defaultBounds;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("SalesmanSolver", inputs);

    // Switch to new trajectory if it's reasonably up-to-date
    if (inputs.seqnum >= 0) {
      var intakePose = RobotState.getInstance().getIntakePose();
      var intakeVelocity = RobotState.getInstance().getIntakeFieldVelocity();
      double newTrajPositionDiff =
          new Translation2d(inputs.xCoeffs[0], inputs.yCoeffs[0])
              .getDistance(intakePose.getTranslation());
      double newTrajVelocityDiff =
          new Translation2d(inputs.xCoeffs[1], inputs.yCoeffs[1])
              .getDistance(
                  new Translation2d(
                      intakeVelocity.vxMetersPerSecond, intakeVelocity.vyMetersPerSecond));
      if (newTrajPositionDiff <= newTrajPositionTolerance
          && newTrajVelocityDiff <= newTrajVelocityTolerance) {
        trajSeqnum = inputs.seqnum;
        trajXCoeffs = inputs.xCoeffs;
        trajYCoeffs = inputs.yCoeffs;
        trajReward = inputs.reward;
      }
    }

    // Log trajectory
    if (trajSeqnum >= 0) {
      Translation2d[] trajectory =
          new Translation2d[(int) (SalesAssociate.tMax / SalesAssociate.tStep)];
      for (int i = 0; i < trajectory.length; i++) {
        double t = i * SalesAssociate.tStep;
        trajectory[i] =
            new Translation2d(
                trajXCoeffs[0] + trajXCoeffs[1] * t + trajXCoeffs[2] * t * t,
                trajYCoeffs[0] + trajYCoeffs[1] * t + trajYCoeffs[2] * t * t);
      }
      Logger.recordOutput("SalesmanSolver/Trajectory", trajectory);
    }

    // Record cycle time
    LoggedTracer.record("SalesmanSolver/Periodic");
  }

  @Override
  public void periodicAfterScheduler() {
    // Update request
    var intakePose = RobotState.getInstance().getIntakePose();
    var intakeVelocitySpeeds = RobotState.getInstance().getIntakeFieldVelocity();
    var intakeVelocity =
        new Translation2d(
            intakeVelocitySpeeds.vxMetersPerSecond, intakeVelocitySpeeds.vyMetersPerSecond);

    io.setRequest(
        ObjectDetection.getInstance().getFuelTranslations().stream().toArray(Translation2d[]::new),
        intakePose.getTranslation(),
        followingTrajectory
            ? intakeVelocity
            : new Translation2d(
                intakeVelocity.rotateBy(intakePose.getRotation().unaryMinus()).getX(),
                intakePose.getRotation()),
        AllianceFlipUtil.apply(bounds));

    // Enable in auto (including disabled)
    io.setEnabled(DriverStation.isAutonomous());

    // Record cycle time
    LoggedTracer.record("SalesmanSolver/PeriodicAfterScheduler");
  }

  /**
   * Returns a target for the follower, looking forward from the closest point on the current
   * trajectory
   */
  public Pose2d getTrajectoryTarget(Pose2d currentPosition, double lookaheadT) {
    if (trajSeqnum < 0) {
      // No valid trajectory
      return currentPosition;
    }

    // Get T for closest point
    double t = 0.0;
    double lastDistance = Double.POSITIVE_INFINITY;
    for (t = 0.0; t < lookaheadTMax; t += lookaheadTStep) {
      double x = trajXCoeffs[0] + trajXCoeffs[1] * t + trajXCoeffs[2] * t * t;
      double y = trajYCoeffs[0] + trajYCoeffs[1] * t + trajYCoeffs[2] * t * t;
      double dist = Math.hypot(x - currentPosition.getX(), y - currentPosition.getY());
      if (dist >= lastDistance) break;
      lastDistance = dist;
    }

    // Get pose for lookahead point (rotation based on derivative)
    t += lookaheadT;
    Pose2d lookaheadPose =
        new Pose2d(
            trajXCoeffs[0] + trajXCoeffs[1] * t + trajXCoeffs[2] * t * t,
            trajYCoeffs[0] + trajYCoeffs[1] * t + trajYCoeffs[2] * t * t,
            new Rotation2d(
                trajXCoeffs[1] + trajXCoeffs[2] * t * 2.0,
                trajYCoeffs[1] + trajYCoeffs[2] * t * 2.0));

    // Return pose
    return lookaheadPose;
  }

  /** Returns the reward for the current trajectory. */
  @AutoLogOutput(key = "SalesmanSolver/Reward")
  public double getReward() {
    return trajReward;
  }
}
