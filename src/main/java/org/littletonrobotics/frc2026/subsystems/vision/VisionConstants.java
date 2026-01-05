// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2026.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import lombok.Builder;
import org.littletonrobotics.frc2026.Constants;

public class VisionConstants {
  public static final double ambiguityThreshold = 0.4;
  public static final double targetLogTimeSecs = 0.1;
  public static final double fieldBorderMargin = 0.5;
  public static final double xyStdDevCoefficient = 0.01;
  public static final double thetaStdDevCoefficient = 0.03;

  public static CameraConfig[] cameras =
      switch (Constants.robot) {
        case COMPBOT -> new CameraConfig[] {};
        case ALPHABOT -> new CameraConfig[] {};
        default -> new CameraConfig[] {};
      };

  @Builder
  public record CameraConfig(
      Pose3d pose,
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
