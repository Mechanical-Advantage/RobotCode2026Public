// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2026.subsystems.vision;

import static org.littletonrobotics.frc2026.subsystems.shooter.ShooterConstants.turretToCamera;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import java.util.Optional;
import java.util.function.Function;
import lombok.Builder;
import lombok.experimental.ExtensionMethod;
import org.littletonrobotics.frc2026.Constants;
import org.littletonrobotics.frc2026.RobotState;
import org.littletonrobotics.frc2026.subsystems.shooter.ShooterConstants;
import org.littletonrobotics.frc2026.util.geometry.GeomUtil;

@ExtensionMethod({GeomUtil.class})
public class VisionConstants {
  public static final double ambiguityThreshold = 0.4;
  public static final double targetLogTimeSecs = 0.1;
  public static final double fieldBorderMargin = 0.5;
  public static final double xyStdDevCoefficient = 0.01;
  public static final double thetaStdDevCoefficient = 0.03;

  private static int monoExposure = 2200;
  private static double monoGain = 17.5;
  private static double monoDenoise = 1.0;

  public static CameraConfig[] cameras =
      switch (Constants.robot) {
        case COMPBOT -> new CameraConfig[] {};
        case ALPHABOT ->
            new CameraConfig[] {
              CameraConfig.builder()
                  .poseFunction(
                      (Double timestamp) -> {
                        Optional<Rotation2d> turretAngle =
                            RobotState.getInstance().getTurretAngle(timestamp);
                        if (turretAngle.isEmpty()) {
                          return Optional.empty();
                        } else {
                          return Optional.of(
                              ShooterConstants.robotToTurret
                                  .toPose3d()
                                  .transformBy(
                                      new Transform3d(
                                          Translation3d.kZero, new Rotation3d(turretAngle.get())))
                                  .transformBy(turretToCamera));
                        }
                      })
                  .id("40552081")
                  .width(1600)
                  .height(1200)
                  .exposure(monoExposure)
                  .gain(monoGain)
                  .denoise(monoDenoise)
                  .stdDevFactor(1.0)
                  .build()
            };
        default -> new CameraConfig[] {};
      };

  @Builder
  public record CameraConfig(
      Function<Double, Optional<Pose3d>> poseFunction,
      String id,
      int width,
      int height,
      int autoExposure,
      int exposure,
      double gain,
      double denoise,
      double stdDevFactor) {}

  private VisionConstants() {}
}
