// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2026.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import java.util.Optional;
import java.util.function.Function;
import lombok.Builder;
import lombok.experimental.ExtensionMethod;
import org.littletonrobotics.frc2026.Constants;
import org.littletonrobotics.frc2026.RobotState;
import org.littletonrobotics.frc2026.util.LoggedTunableNumber;
import org.littletonrobotics.frc2026.util.geometry.GeomUtil;

@ExtensionMethod({GeomUtil.class})
public class VisionConstants {
  public static final boolean alwaysRecord = false;

  public static final double ambiguityThreshold = 0.4;
  public static final double targetLogTimeSecs = 0.1;
  public static final double autoIgnoreTimeSecs = 2.0;
  public static final double fieldBorderMargin = 0.5;
  public static final double xyStdDevCoefficient = 0.01;
  public static final double thetaStdDevCoefficient = 0.03;
  public static final double fuelDetectConfidenceThreshold = 0.0; // Enforced by Northstar
  public static final double robotDetectConfidenceThreshold = 0.0; // Enforced by Northstar
  public static final double zMin = -0.5;
  public static final double zMax = 1.0;

  // Moving camera constants
  public static final double robotToPivotX = Units.inchesToMeters(12.480);
  public static final double pivotToSlotYOffset = Units.inchesToMeters(2.188);
  public static final double pivotToWasherNorm =
      Units.inchesToMeters(7.359); // Measured center to center
  public static final double slotStartToHopperX =
      Units.inchesToMeters(6.345); // Measured from bottom "center"
  public static final double hopperWallToCameraX = Units.inchesToMeters(-0.885);
  public static final double slotAngle = Units.degreesToRadians(70.0);

  private static LoggedTunableNumber[] cameraPitchFudgeDegrees =
      new LoggedTunableNumber[] {
        new LoggedTunableNumber("Vision/Camera0PitchFudgeDeg", 0.0),
        new LoggedTunableNumber("Vision/Camera1PitchFudgeDeg", -4.5),
        new LoggedTunableNumber("Vision/Camera2PitchFudgeDeg", -1.0),
        new LoggedTunableNumber("Vision/Camera3PitchFudgeDeg", 0.0)
      };

  private static int monoExposure = 1800;
  private static double monoGain = 15.0;
  private static double monoDenoise = 1.0;
  private static int colorExposure = 4500;
  private static double colorGain = 5.0;
  private static double cameraBalanceRed = 1.2;
  private static double cameraBalanceBlue = 1.2;

  public static CameraConfig[] cameras =
      switch (Constants.getRobot()) {
        case DARWIN ->
            new CameraConfig[] {
              CameraConfig.builder()
                  .poseFunction(
                      (Double timestamp) -> {
                        return Optional.of(
                            new Pose3d(
                                -0.324607,
                                0.000279,
                                0.408737,
                                new Rotation3d(
                                    0.0,
                                    Units.degreesToRadians(
                                        -22.5 + cameraPitchFudgeDegrees[0].get()),
                                    Units.degreesToRadians(180.0))));
                      })
                  .id("40552081")
                  .width(1800)
                  .height(1200)
                  .exposure(monoExposure)
                  .gain(monoGain)
                  .denoise(monoDenoise)
                  .stdDevFactor(1.0)
                  .fovRads(Units.degreesToRadians(75.0))
                  .build(),
              CameraConfig.builder()
                  .poseFunction(
                      (Double timestamp) -> {
                        if (RobotState.getInstance().getSlamAngle(timestamp).isPresent()) {
                          var pivotToWasherX =
                              pivotToWasherNorm
                                  * RobotState.getInstance().getSlamAngle(timestamp).get().getCos();
                          var slotToWasherY =
                              pivotToWasherNorm
                                      * RobotState.getInstance()
                                          .getSlamAngle(timestamp)
                                          .get()
                                          .getSin()
                                  - pivotToSlotYOffset; // equivalent to height on slot
                          var washerToSlotX = slotToWasherY / Math.tan(slotAngle);
                          var cameraPosition =
                              robotToPivotX
                                  + pivotToWasherX
                                  + washerToSlotX
                                  + slotStartToHopperX
                                  + hopperWallToCameraX;
                          return Optional.of(
                              new Pose3d(
                                  cameraPosition,
                                  Units.inchesToMeters(8.476),
                                  Units.inchesToMeters(19.096),
                                  new Rotation3d(
                                      0.0,
                                      Units.degreesToRadians(
                                          12.0 + cameraPitchFudgeDegrees[1].get()),
                                      Units.degreesToRadians(-5.0))));
                        } else {
                          return Optional.empty();
                        }
                      })
                  .id("24737133")
                  .width(1280)
                  .height(960)
                  .exposure(colorExposure)
                  .gain(colorGain)
                  .stdDevFactor(1.0)
                  .fovRads(Units.degreesToRadians(90.0))
                  .cameraBalanceRed(cameraBalanceRed)
                  .cameraBalanceBlue(cameraBalanceBlue)
                  .build(),
              CameraConfig.builder()
                  .poseFunction(
                      (Double timestamp) -> {
                        return Optional.of(
                            new Pose3d(
                                -0.170833,
                                0.316631,
                                0.435463,
                                new Rotation3d(
                                    0.0,
                                    Units.degreesToRadians(12.0 + cameraPitchFudgeDegrees[2].get()),
                                    Units.degreesToRadians(70.0))));
                      })
                  .id("24736167")
                  .width(1280)
                  .height(960)
                  .exposure(colorExposure)
                  .gain(colorGain)
                  .stdDevFactor(1.0)
                  .fovRads(Units.degreesToRadians(90.0))
                  .cameraBalanceRed(cameraBalanceRed)
                  .cameraBalanceBlue(cameraBalanceBlue)
                  .build(),
              CameraConfig.builder()
                  .poseFunction(
                      (Double timestamp) -> {
                        return Optional.of(
                            new Pose3d(
                                -0.158133,
                                -0.316631,
                                0.435463,
                                new Rotation3d(
                                    0.0,
                                    Units.degreesToRadians(12.0 + cameraPitchFudgeDegrees[3].get()),
                                    Units.degreesToRadians(-70.0))));
                      })
                  .id("24736221")
                  .width(1280)
                  .height(960)
                  .exposure(colorExposure)
                  .gain(colorGain)
                  .stdDevFactor(1.0)
                  .fovRads(Units.degreesToRadians(90.0))
                  .cameraBalanceRed(cameraBalanceRed)
                  .cameraBalanceBlue(cameraBalanceBlue)
                  .build()
            };
        case ALPHABOT ->
            new CameraConfig[] {
              CameraConfig.builder()
                  .poseFunction(
                      (Double timestamp) -> {
                        return Optional.of(
                            new Pose3d(
                                Units.inchesToMeters(-28.0 / 2.0 + 2.5),
                                Units.inchesToMeters(-28.0 / 2.0 + 2.75),
                                Units.inchesToMeters(18.75),
                                new Rotation3d(
                                    0.0,
                                    Units.degreesToRadians(-27.0),
                                    Units.degreesToRadians(-152.5))));
                      })
                  .id("40530395")
                  .width(1800)
                  .height(1200)
                  .exposure(monoExposure)
                  .gain(monoGain)
                  .denoise(monoDenoise)
                  .stdDevFactor(1.0)
                  .fovRads(Units.degreesToRadians(75.0))
                  .build(),
              CameraConfig.builder()
                  .poseFunction(
                      (Double timestamp) -> {
                        return Optional.of(
                            new Pose3d(
                                Units.inchesToMeters(-28.0 / 2.0 + 2.25),
                                Units.inchesToMeters(-28.0 / 2.0 + 2.75),
                                Units.inchesToMeters(18.0),
                                new Rotation3d(
                                    Units.degreesToRadians(11.0),
                                    Units.degreesToRadians(15.0),
                                    Units.degreesToRadians(-167.0))));
                      })
                  .id("24737133")
                  .width(1280)
                  .height(960)
                  .exposure(colorExposure)
                  .gain(colorGain)
                  .denoise(monoDenoise)
                  .stdDevFactor(1.0)
                  .fovRads(Units.degreesToRadians(90.0))
                  .cameraBalanceRed(cameraBalanceRed)
                  .cameraBalanceBlue(cameraBalanceBlue)
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
      double stdDevFactor,
      double fovRads,
      double cameraBalanceRed,
      double cameraBalanceBlue) {}

  private VisionConstants() {}
}
