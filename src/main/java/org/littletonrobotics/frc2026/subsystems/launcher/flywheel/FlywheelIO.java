// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2026.subsystems.launcher.flywheel;

import org.littletonrobotics.idun.IdunIO;
import org.littletonrobotics.junction.AutoLog;

@IdunIO
public interface FlywheelIO {
  @AutoLog
  public static class FlywheelIOInputs {
    public boolean connected;
    public boolean follower1Connected;
    public boolean follower2Connected;
    public boolean follower3Connected;
    public boolean bridgeBrownout;
    public boolean follower1BridgeBrownout;
    public boolean follower2BridgeBrownout;
    public boolean follower3BridgeBrownout;
    public double positionRads;
    public double velocityRadsPerSec;
    public double appliedVoltage;
    public double supplyVoltage;
    public double supplyCurrentAmps;
    public double torqueCurrentAmps;
    public double tempCelsius;
    public double follower1SupplyCurrentAmps;
    public double follower1TempCelsius;
    public double follower2SupplyCurrentAmps;
    public double follower2TempCelsius;
    public double follower3SupplyCurrentAmps;
    public double follower3TempCelsius;
  }

  public static enum FlywheelIOOutputMode {
    COAST,
    VELOCITY,
    VOLTAGE,
    CHARACTERIZE
  }

  public static class FlywheelIOOutputs {
    public FlywheelIOOutputMode mode = FlywheelIOOutputMode.COAST;
    public double velocityRadsPerSec = 0.0;
    public double feedforward = 0.0;

    public double voltage = 0.0;

    public double kP;
    public double kD;
  }

  default void updateInputs(FlywheelIOInputs inputs) {}

  default void applyOutputs(FlywheelIOOutputs outputs) {}
}
