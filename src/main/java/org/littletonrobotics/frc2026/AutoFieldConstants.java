// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2026;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import org.littletonrobotics.frc2026.subsystems.drive.DriveConstants;
import org.littletonrobotics.frc2026.subsystems.launcher.LaunchCalculator;
import org.littletonrobotics.frc2026.util.geometry.VerticalFlipUtil;

public class AutoFieldConstants {
  public static class Trench {
    // Positioned so the robot is right in front of the starting line
    public static final Translation2d leftStart =
        new Translation2d(
            FieldConstants.LinesVertical.starting + DriveConstants.fullApothemX,
            (FieldConstants.LinesHorizontal.leftTrenchOpenStart
                    + FieldConstants.LinesHorizontal.leftTrenchOpenEnd)
                / 2.0);
    public static final Translation2d rightStart = VerticalFlipUtil.apply(leftStart);

    // Positioned so the robot is right behind the starting line
    public static final Translation2d leftStartOffset =
        new Translation2d(
            FieldConstants.LinesVertical.starting - DriveConstants.fullApothemX,
            (FieldConstants.LinesHorizontal.leftTrenchOpenStart
                    + FieldConstants.LinesHorizontal.leftTrenchOpenEnd)
                / 2.0);
    public static final Translation2d rightStartOffset = VerticalFlipUtil.apply(leftStartOffset);

    // Inside from the trench so that the robot clears the trapezoid extension
    public static final Translation2d leftEntry =
        new Translation2d(
            FieldConstants.LinesVertical.starting - DriveConstants.fullApothemX,
            (FieldConstants.LinesHorizontal.leftTrenchOpenStart
                    + FieldConstants.LinesHorizontal.leftTrenchOpenEnd)
                / 2.0);
    public static final Translation2d rightEntry = VerticalFlipUtil.apply(leftEntry);

    // Inside from the trench so that the launcher just clears the bar
    public static final Translation2d leftBeforeBar =
        new Translation2d(
            (FieldConstants.LinesVertical.allianceZone
                        + FieldConstants.LinesVertical.neutralZoneNear)
                    / 2.0
                - FieldConstants.trenchBarWidth / 2.0
                - DriveConstants.fullApothemX,
            (FieldConstants.LinesHorizontal.leftTrenchOpenStart
                    + FieldConstants.LinesHorizontal.leftTrenchOpenEnd)
                / 2.0);
    public static final Translation2d rightBeforeBar = VerticalFlipUtil.apply(leftBeforeBar);

    // Outside from the trench so that the robot clears the outward trapezoid extension
    public static final Translation2d leftClear =
        new Translation2d(
            FieldConstants.LinesVertical.starting
                + FieldConstants.LeftTrench.depth
                + DriveConstants.fullApothemX
                + 0.2,
            (FieldConstants.LinesHorizontal.leftTrenchOpenStart
                    + FieldConstants.LinesHorizontal.leftTrenchOpenEnd)
                / 2.0);
    public static final Translation2d rightClear = VerticalFlipUtil.apply(leftClear);
  }

  public static class Bump {
    // Positioned so whole drivebase is on the floor right inside the bump
    public static final Translation2d leftInner =
        new Translation2d(
            FieldConstants.LinesVertical.starting - DriveConstants.fullApothemX,
            FieldConstants.LinesHorizontal.leftBumpMiddle);
    public static final Translation2d rightInner = VerticalFlipUtil.apply(leftInner);

    // Positioned so whole drivebase is on the floor right outside the bump
    public static final Translation2d leftOuter =
        new Translation2d(
            FieldConstants.LinesVertical.neutralZoneNear + DriveConstants.fullApothemX,
            FieldConstants.LinesHorizontal.leftBumpMiddle);
    public static final Translation2d rightOuter = VerticalFlipUtil.apply(leftOuter);
  }

  public static class Hub {
    public static final Translation2d centerStart =
        new Translation2d(
            FieldConstants.LinesVertical.starting - DriveConstants.fullApothemX,
            FieldConstants.LinesHorizontal.center);
  }

  public static class Depot {
    public static final Translation2d leftThrough =
        new Translation2d(
            FieldConstants.Depot.depth / 2.0 + 0.2,
            FieldConstants.Depot.leftCorner.getY() + DriveConstants.fullApothemX + 0.1);

    public static final Translation2d rightThrough =
        new Translation2d(
            FieldConstants.Depot.depth / 2.0 + 0.2,
            FieldConstants.Depot.rightCorner.getY() - DriveConstants.fullApothemX - 0.1);
  }

  public static class Tower {
    public static final Translation2d leftThrough =
        new Translation2d(
            FieldConstants.Tower.frontFaceX / 2.0,
            FieldConstants.Tower.leftUpright.getY() + DriveConstants.fullApothemX + 0.25);

    public static final Translation2d rightThrough =
        new Translation2d(
            FieldConstants.Tower.frontFaceX / 2.0,
            FieldConstants.Tower.rightUpright.getY() - DriveConstants.fullApothemX - 0.25);

    public static final Translation2d leftOutside =
        FieldConstants.Tower.leftUpright.plus(
            new Translation2d(
                DriveConstants.fullApothemX + 0.3, DriveConstants.fullApothemX + 0.3));

    public static final Translation2d rightOutside =
        FieldConstants.Tower.rightUpright.plus(
            new Translation2d(
                DriveConstants.fullApothemX + 0.3, -DriveConstants.fullApothemX - 0.3));
  }

  public static class Climb {
    public static final Pose2d right =
        new Pose2d(
            FieldConstants.Tower.rightUpright.plus(
                new Translation2d(0.0, -DriveConstants.fullApothemX)),
            Rotation2d.kPi);

    public static final Pose2d rightOffset =
        new Pose2d(
            FieldConstants.Tower.rightUpright.plus(
                new Translation2d(0.0, -DriveConstants.fullApothemX - 0.6)),
            Rotation2d.kPi);

    public static final Pose2d left =
        new Pose2d(
            FieldConstants.Tower.leftUpright.plus(
                new Translation2d(0.0, +DriveConstants.fullApothemX)),
            Rotation2d.kZero);

    public static final Pose2d leftOffset =
        new Pose2d(
            FieldConstants.Tower.leftUpright.plus(
                new Translation2d(0.0, +DriveConstants.fullApothemX + 0.6)),
            Rotation2d.kZero);
  }

  public static class Launch {
    public static final Pose2d leftTower =
        LaunchCalculator.getStationaryAimedPose(
            Climb.left.getTranslation().plus(new Translation2d(1.5, 0.5)), true);

    public static final Pose2d rightTower =
        LaunchCalculator.getStationaryAimedPose(
            Climb.right.getTranslation().plus(new Translation2d(1.5, -0.5)), true);

    public static Pose2d leftBump =
        LaunchCalculator.getStationaryAimedPose(
            new Translation2d(
                FieldConstants.LinesVertical.starting - 0.7,
                FieldConstants.LinesHorizontal.leftBumpMiddle),
            true);

    public static Pose2d rightBump =
        LaunchCalculator.getStationaryAimedPose(
            new Translation2d(
                FieldConstants.LinesVertical.starting - 0.7,
                FieldConstants.LinesHorizontal.rightBumpMiddle),
            true);
  }
}
