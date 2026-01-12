// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2026.subsystems.rollers;

import org.littletonrobotics.junction.AutoLog;

public interface RollerSystemIO {
  @AutoLog
  public static class RollerSystemIOInputs {
    public double positionRads;
    public double velocityRadsPerSec;
    public double appliedVoltage;
    public double supplyCurrentAmps;
    public double torqueCurrentAmps;
    public double tempCelsius;
    public boolean tempFault;
    public boolean connected;
  }

  public static class RollerSystemIOOutputs {
    public double appliedVoltage = 0.0;
    public boolean brakeModeEnabled = true;
  }

  default void updateInputs(RollerSystemIOInputs inputs) {}

  default void applyOutputs(RollerSystemIOOutputs outputs) {}
}
