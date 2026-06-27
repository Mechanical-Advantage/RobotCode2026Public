// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2026.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.idun.IdunIO;
import org.littletonrobotics.junction.AutoLog;

@IdunIO
public interface GyroIO {
  @AutoLog
  public static class GyroIOInputs {
    public boolean connected = false;
    public Rotation2d yawPosition = Rotation2d.kZero;
    public Rotation2d pitchPosition = Rotation2d.kZero;
    public Rotation2d rollPosition = Rotation2d.kZero;
    public double yawVelocityRadPerSec = 0.0;
    public double pitchVelocityRadPerSec = 0.0;
    public double rollVelocityRadPerSec = 0.0;
    public double accelerationX = 0.0; // Includes acceleration due to gravity, units "g"
    public double accelerationY = 0.0;
    public double accelerationZ = 0.0;
  }

  public static class GyroIOOutputs {}

  public default void updateInputs(GyroIOInputs inputs) {}

  public default void applyOutputs(GyroIOOutputs outputs) {}
}
