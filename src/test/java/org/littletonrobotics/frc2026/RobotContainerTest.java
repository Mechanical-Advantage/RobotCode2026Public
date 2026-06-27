// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2026;

import static org.junit.jupiter.api.Assertions.fail;

import edu.wpi.first.wpilibj.DriverStation;
import org.junit.jupiter.api.Test;

public class RobotContainerTest {
  @Test
  public void createRobotContainer() {
    // Disable joystick warning
    DriverStation.silenceJoystickConnectionWarning(true);

    // Instantiate RobotContainer
    try {
      new RobotContainer();
    } catch (Exception e) {
      e.printStackTrace();
      fail("Failed to instantiate RobotContainer, see stack trace above.");
    }
  }
}
