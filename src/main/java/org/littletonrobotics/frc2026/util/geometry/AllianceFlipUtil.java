// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2026.util.geometry;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.DriverStation;
import org.littletonrobotics.frc2026.Constants;
import org.littletonrobotics.frc2026.FieldConstants;

public class AllianceFlipUtil {
  public static double flipX(double x) {
    return FieldConstants.fieldLength - x;
  }

  public static double flipY(double y) {
    return FieldConstants.fieldWidth - y;
  }

  public static Translation2d flip(Translation2d translation) {
    return new Translation2d(flipX(translation.getX()), flipY(translation.getY()));
  }

  public static Rotation2d flip(Rotation2d rotation) {
    return rotation.rotateBy(Rotation2d.kPi);
  }

  public static Pose2d flip(Pose2d pose) {
    return new Pose2d(flip(pose.getTranslation()), flip(pose.getRotation()));
  }

  public static double applyX(double x) {
    return shouldFlip() ? flipX(x) : x;
  }

  public static double applyY(double y) {
    return shouldFlip() ? flipY(y) : y;
  }

  public static Translation2d apply(Translation2d translation) {
    return new Translation2d(applyX(translation.getX()), applyY(translation.getY()));
  }

  public static Rotation2d apply(Rotation2d rotation) {
    return shouldFlip() ? flip(rotation) : rotation;
  }

  public static Pose2d apply(Pose2d pose) {
    return shouldFlip()
        ? new Pose2d(apply(pose.getTranslation()), apply(pose.getRotation()))
        : pose;
  }

  public static Translation3d apply(Translation3d translation) {
    return new Translation3d(
        applyX(translation.getX()), applyY(translation.getY()), translation.getZ());
  }

  public static Rotation3d apply(Rotation3d rotation) {
    return shouldFlip() ? rotation.rotateBy(new Rotation3d(0.0, 0.0, Math.PI)) : rotation;
  }

  public static Pose3d apply(Pose3d pose) {
    return new Pose3d(apply(pose.getTranslation()), apply(pose.getRotation()));
  }

  public static Bounds apply(Bounds bounds) {
    if (shouldFlip()) {
      return new Bounds(
          applyX(bounds.maxX()),
          applyX(bounds.minX()),
          applyY(bounds.maxY()),
          applyY(bounds.minY()));
    } else {
      return bounds;
    }
  }

  public static boolean shouldFlip() {
    return !Constants.disableHAL
        && DriverStation.getAlliance().isPresent()
        && DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
  }
}
