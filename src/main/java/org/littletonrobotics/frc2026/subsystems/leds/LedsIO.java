// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2026.subsystems.leds;

import org.littletonrobotics.idun.IdunIO;
import org.littletonrobotics.junction.AutoLog;

@IdunIO
public interface LedsIO {
  @AutoLog
  public static class LedsIOInputs {
    // FPGA time allows patterns to be synchronized across the RIO and Mac mini (unused)
    public double fpgaTime = 0.0;
  }

  public static class LedsIOOutputs {
    public byte[] buffer = new byte[0];

    public boolean ready = false;
  }

  public default void updateInputs(LedsIOInputs inputs) {}

  public default void applyOutputs(LedsIOOutputs outputs) {}
}
