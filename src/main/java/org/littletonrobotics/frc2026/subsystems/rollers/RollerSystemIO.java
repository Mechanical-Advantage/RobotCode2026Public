// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2026.subsystems.rollers;

import org.littletonrobotics.idun.IdunIO;
import org.littletonrobotics.junction.AutoLog;

@IdunIO
public interface RollerSystemIO {
  @AutoLog
  public static class RollerSystemIOInputs {
    public boolean connected;
    public double positionRads;
    public double velocityRadsPerSec;
    public double appliedVoltage;
    public double supplyCurrentAmps;
    public double torqueCurrentAmps;
    public double tempCelsius;

    public boolean hasFollower;
    public boolean followerConnected;
    public double followerSupplyCurrentAmps;
    public double followerTempCelsius;
  }

  public enum RollerSystemIOMode {
    BRAKE,
    COAST,
    VOLTAGE_CONTROL,
    CLOSED_LOOP
  }

  public static class RollerSystemIOOutputs {
    public RollerSystemIOMode mode = RollerSystemIOMode.BRAKE;
    // Voltage control
    public double appliedVoltage = 0.0;

    // Closed loop control
    public double velocity = 0.0;
    public double kP = 0.0;
    public double kD = 0.0;
    public double feedforward = 0.0;

    public boolean brakeModeEnabled = true;
  }

  default void updateInputs(RollerSystemIOInputs inputs) {}

  default void applyOutputs(RollerSystemIOOutputs outputs) {}
}
