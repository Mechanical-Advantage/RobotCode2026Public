// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2026.subsystems.leds;

import org.littletonrobotics.junction.AutoLog;

public interface LedsIO {
  @AutoLog
  public static class LedsIOInputs {}

  public static class LedsIOOutputs {
    public byte[] buffer = new byte[0];
  }

  public default void updateInputs(LedsIOInputs inputs) {}

  public default void applyOutputs(LedsIOOutputs outputs) {}
}
