// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2026.subsystems.slamtake;

import org.littletonrobotics.idun.IdunIO;
import org.littletonrobotics.junction.AutoLog;

@IdunIO
public interface SlamIO {
  @AutoLog
  public static class SlamIOInputs {
    public boolean connected;
    public double positionRads;
    public double velocityRadsPerSec;
    public double appliedVoltage;
    public double supplyCurrentAmps;
    public double torqueCurrentAmps;
    public double tempCelsius;
  }

  public enum SlamIOOutputMode {
    BRAKE,
    COAST,
    RUN_OPEN_LOOP,
    RUN_CLOSED_LOOP
  }

  public static class SlamIOOutputs {
    public SlamIOOutputMode mode = SlamIOOutputMode.BRAKE;
    public double appliedVolts = 0.0;

    public double position = 0.0;
    public double kP = 0.0;
    public double kD = 0.0;
  }

  public default void updateInputs(SlamIOInputs inputs) {}

  public default void applyOutputs(SlamIOOutputs outputs) {}
}
