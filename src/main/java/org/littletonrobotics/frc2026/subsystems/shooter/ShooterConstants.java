// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2026.subsystems.shooter;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;

public class ShooterConstants {
  public static Transform3d robotToTurret = new Transform3d(-0.19685, 0.0, 0.44, Rotation3d.kZero);
  public static Transform3d turretToCamera =
      new Transform3d(
          -0.1314196, 0.0, 0.2770674, new Rotation3d(0.0, Units.degreesToRadians(-22.5), 0.0));

  private ShooterConstants() {}
}
