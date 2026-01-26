// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2026.subsystems.leds;

import edu.wpi.first.hal.AddressableLEDJNI;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.hal.PWMJNI;

public class LedsIOHAL implements LedsIO {
  private final int handle;

  public LedsIOHAL() {
    int pwmHandle = PWMJNI.initializePWMPort(HAL.getPort((byte) LedConstants.port));
    handle = AddressableLEDJNI.initialize(pwmHandle);
    AddressableLEDJNI.setLength(handle, LedConstants.length);
    AddressableLEDJNI.start(handle);
  }

  @Override
  public void applyOutputs(LedsIOOutputs outputs) {
    AddressableLEDJNI.setData(handle, outputs.buffer);
  }
}
