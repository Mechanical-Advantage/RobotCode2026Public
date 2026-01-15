// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2026.subsystems.flywheel;

import org.littletonrobotics.frc2026.Constants;
import org.littletonrobotics.frc2026.util.LoggedTunableNumber;

public class FlywheelConstants {
  public static final double reduction = 1;

  public static final double ks = 0.0;
  public static final LoggedTunableNumber kp =
      switch (Constants.robot) {
        case COMPBOT -> new LoggedTunableNumber("Flywheel/kp", 0.0);
        case ALPHABOT -> new LoggedTunableNumber("Flywheel/kp", 15.0);
        default -> new LoggedTunableNumber("Flywheel/kp", 0.0);
      };

  public static final LoggedTunableNumber kd =
      switch (Constants.robot) {
        case COMPBOT -> new LoggedTunableNumber("Flywheel/kd", 0.0);
        case ALPHABOT -> new LoggedTunableNumber("Flywheel/kd", 0.0);
        default -> new LoggedTunableNumber("Flywheel/kd", 0.0);
      };
}
