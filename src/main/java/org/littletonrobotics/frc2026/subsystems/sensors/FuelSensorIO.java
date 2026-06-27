// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2026.subsystems.sensors;

import org.littletonrobotics.idun.IdunIO;
import org.littletonrobotics.junction.AutoLog;

@IdunIO
public interface FuelSensorIO {
  @AutoLog
  public static class FuelSensorIOInputs {
    public double distanceMeters = 0.0;
    public boolean valid = false;
  }

  public static class FuelSensorIOOutputs {}

  default void updateInputs(FuelSensorIOInputs inputs) {}

  default void applyOutputs(FuelSensorIOOutputs outputs) {}
}
