// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2026.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface ModuleIO {
  @AutoLog
  public static class ModuleIOInputs {
    public boolean driveConnected = false;
    public double drivePositionRads = 0.0;
    public double driveVelocityRadsPerSec = 0.0;
    public double driveAppliedVolts = 0.0;
    public double driveSupplyCurrentAmps = 0.0;
    public double driveTorqueCurrentAmps = 0.0;

    public boolean turnConnected = false;
    public Rotation2d turnAbsolutePositionRads = Rotation2d.kZero;
    public Rotation2d turnPositionRads = Rotation2d.kZero;
    public double turnVelocityRadsPerSec = 0.0;
    public double turnAppliedVolts = 0.0;
    public double turnSupplyCurrentAmps = 0.0;
    public double turnTorqueCurrentAmps = 0.0;
  }

  public static enum ModuleIOOutputMode {
    COAST,
    BRAKE,
    DRIVE,
    CHARACTERIZE
  }

  public static class ModuleIOOutputs {
    public ModuleIOOutputMode mode = ModuleIOOutputMode.COAST;
    public double driveVelocityRadPerSec = 0.0;
    public double driveFeedforward = 0.0;
    public double driveCharacterizationOutput = 0.0;
    public Rotation2d turnRotation = Rotation2d.kZero;
    public boolean turnNeutral = false;
  }

  public default void updateInputs(ModuleIOInputs inputs) {}

  public default void applyOutputs(ModuleIOOutputs outputs) {}
}
