// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2026.subsystems.drive;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import org.littletonrobotics.frc2026.Robot;
import org.littletonrobotics.frc2026.subsystems.drive.ModuleIO.ModuleIOOutputMode;
import org.littletonrobotics.frc2026.subsystems.drive.ModuleIO.ModuleIOOutputs;
import org.littletonrobotics.frc2026.util.LoggedTracer;
import org.littletonrobotics.junction.Logger;

public class Module {
  private final ModuleIO io;
  private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
  private final ModuleIOOutputs outputs = new ModuleIOOutputs();
  private final int index;

  private SimpleMotorFeedforward ffModel =
      new SimpleMotorFeedforward(DriveConstants.driveKs, DriveConstants.driveKv);

  private final Alert driveDisconnectedAlert;
  private final Alert turnDisconnectedAlert;

  public Module(ModuleIO io, int index) {
    this.io = io;
    this.index = index;
    driveDisconnectedAlert =
        new Alert(
            "Disconnected drive motor on module " + Integer.toString(index) + ".",
            AlertType.kError);
    turnDisconnectedAlert =
        new Alert(
            "Disconnected turn motor on module " + Integer.toString(index) + ".", AlertType.kError);
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Drive/Module" + Integer.toString(index), inputs);

    // Update alerts
    driveDisconnectedAlert.set(Robot.showHardwareAlerts() && !inputs.driveConnected);
    turnDisconnectedAlert.set(Robot.showHardwareAlerts() && !inputs.turnConnected);

    // Record cycle times
    LoggedTracer.record("Drive/Module" + index);
  }

  public void periodicAfterScheduler() {
    io.applyOutputs(outputs);
  }

  /** Runs the module with the specified setpoint state. Mutates the state to optimize it. */
  public void runSetpoint(SwerveModuleState state) {
    // Optimize velocity setpoint
    state.optimize(getAngle());
    state.cosineScale(inputs.turnPosition);

    // Apply setpoints
    double speedRadPerSec = state.speedMetersPerSecond / DriveConstants.wheelRadius;
    outputs.mode = ModuleIOOutputMode.DRIVE;
    outputs.driveVelocityRadPerSec = speedRadPerSec;
    outputs.driveFeedforward = ffModel.calculate(speedRadPerSec);
    outputs.turnRotation = state.angle;
    outputs.turnNeutral =
        Math.abs(state.angle.minus(getAngle()).getDegrees()) < DriveConstants.turnDeadbandDegrees;
  }

  /** Runs the module with the specified output while controlling to zero degrees. */
  public void runCharacterization(double output) {
    outputs.mode = ModuleIOOutputMode.CHARACTERIZE;
    outputs.driveCharacterizationOutput = output;
    outputs.turnRotation = Rotation2d.kZero;
  }

  /** Disables all motor outputs in brake mode. */
  public void brake() {
    outputs.mode = ModuleIOOutputMode.BRAKE;
  }

  /** Disables all motor outputs in coast mode. */
  public void coast() {
    outputs.mode = ModuleIOOutputMode.COAST;
  }

  /** Returns the current turn angle of the module. */
  public Rotation2d getAngle() {
    return inputs.turnPosition;
  }

  /** Returns the current drive position of the module in meters. */
  public double getPositionMeters() {
    return inputs.drivePositionRad * DriveConstants.wheelRadius;
  }

  /** Returns the current drive velocity of the module in meters per second. */
  public double getVelocityMetersPerSec() {
    return inputs.driveVelocityRadPerSec * DriveConstants.wheelRadius;
  }

  /** Returns the module position (turn angle and drive position). */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getPositionMeters(), getAngle());
  }

  /** Returns the module state (turn angle and drive velocity). */
  public SwerveModuleState getState() {
    return new SwerveModuleState(getVelocityMetersPerSec(), getAngle());
  }

  /** Returns the module position in radians. */
  public double getWheelRadiusCharacterizationPosition() {
    return inputs.drivePositionRad;
  }

  /** Returns the module velocity in rotations/sec (Phoenix native units). */
  public double getFFCharacterizationVelocity() {
    return Units.radiansToRotations(inputs.driveVelocityRadPerSec);
  }
}
