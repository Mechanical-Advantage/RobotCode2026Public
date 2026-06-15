// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2026.commands;

import choreo.Choreo;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.Arrays;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.frc2026.subsystems.drive.DriveConstants;
import org.littletonrobotics.frc2026.util.FuelSim;
import org.littletonrobotics.frc2026.util.FuelSim.SimRobot;
import org.littletonrobotics.frc2026.util.geometry.AllianceFlipUtil;
import org.littletonrobotics.frc2026.util.geometry.VerticalFlipUtil;
import org.littletonrobotics.junction.Logger;

public class SimTrajectory extends Command {
  private final Timer timer = new Timer();
  private final Trajectory<SwerveSample> trajectory;
  private final FuelSim fuelSim;
  private int fuelIntaken = 0;
  private int instance;
  private BooleanSupplier isBlueAlliance;
  private final BooleanSupplier mirror;

  public SimTrajectory(
      String name,
      FuelSim fuelSim,
      int instance,
      BooleanSupplier isBlueAlliance,
      BooleanSupplier mirror) {
    Optional<Trajectory<SwerveSample>> trajectoryOptional = Choreo.loadTrajectory(name);
    if (!trajectoryOptional.isPresent()) {
      throw new RuntimeException("Choreo Trajectory Not Found: " + name);
    }
    this.trajectory = trajectoryOptional.get();
    this.fuelSim = fuelSim;
    this.instance = instance;
    this.isBlueAlliance = isBlueAlliance;
    this.mirror = mirror;
  }

  @Override
  public void initialize() {
    SimRobot simRobot =
        fuelSim.registerRobot(
            DriveConstants.fullWidthX,
            DriveConstants.fullWidthY,
            Units.inchesToMeters(6.0),
            this::getPose,
            fieldRelativeVelocitySupplier());

    simRobot.registerIntake(
        DriveConstants.intakeNearX,
        DriveConstants.intakeFarX,
        -DriveConstants.fullWidthY / 2,
        DriveConstants.fullWidthY / 2,
        () -> true,
        () -> fuelIntaken++);

    Logger.recordOutput(
        "SimTrajectory" + instance + "/Trajectory",
        Arrays.stream(trajectory.getPoses())
            .map((pose) -> isBlueAlliance.getAsBoolean() ? pose : AllianceFlipUtil.flip(pose))
            .map(pose -> mirror.getAsBoolean() ? VerticalFlipUtil.apply(pose) : pose)
            .toArray(Pose2d[]::new));
    Logger.recordOutput("SimTrajectory" + instance + "/FuelIntaken", 0);
    Logger.recordOutput("SimTrajectory" + instance + "/Mirror", mirror.getAsBoolean());
    fuelIntaken = 0;

    timer.restart();
  }

  @Override
  public void execute() {
    Logger.recordOutput("SimTrajectory" + instance + "/Setpoint/TrajectoryPose", getPose());
    Logger.recordOutput(
        "SimTrajectory" + instance + "/Setpoint/Speeds", fieldRelativeVelocitySupplier().get());
    Logger.recordOutput("SimTrajectory" + instance + "/FuelIntaken", fuelIntaken);
  }

  @Override
  public boolean isFinished() {
    boolean finished = timer.hasElapsed(trajectory.getTotalTime());
    Logger.recordOutput("SimTrajectory" + instance + "/IsFinished", finished);
    return finished;
  }

  private Pose2d getPose() {
    Pose2d pose = trajectory.sampleAt(timer.get(), !isBlueAlliance.getAsBoolean()).get().getPose();
    return mirror.getAsBoolean() ? VerticalFlipUtil.apply(pose) : pose;
  }

  private Supplier<ChassisSpeeds> fieldRelativeVelocitySupplier() {
    return () -> {
      var sample = trajectory.sampleAt(timer.get(), !isBlueAlliance.getAsBoolean()).get();
      return new ChassisSpeeds(sample.vx, sample.vy, sample.omega);
    };
  }
}
