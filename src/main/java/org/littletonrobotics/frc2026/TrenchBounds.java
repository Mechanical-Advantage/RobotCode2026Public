// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2026;

import edu.wpi.first.math.geometry.Translation2d;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.frc2026.subsystems.drive.DriveConstants;
import org.littletonrobotics.frc2026.util.geometry.Bounds;
import org.littletonrobotics.frc2026.util.geometry.VerticalFlipUtil;

// Bounds when the hood should be forced down
public class TrenchBounds {
  public static class Hood {
    // Distance from hugging the trench bar
    private static double generosity = 0.4;

    public static final Bounds blueLeftTrench =
        new Bounds(
            (FieldConstants.LinesVertical.allianceZone
                        + FieldConstants.LinesVertical.neutralZoneNear)
                    / 2.0
                - FieldConstants.trenchBarWidth / 2.0
                - DriveConstants.fullBaseRadius
                - generosity,
            (FieldConstants.LinesVertical.allianceZone
                        + FieldConstants.LinesVertical.neutralZoneNear)
                    / 2.0
                + FieldConstants.trenchBarWidth / 2.0
                + DriveConstants.fullBaseRadius
                + generosity,
            FieldConstants.LinesHorizontal.leftTrenchOpenEnd,
            FieldConstants.LinesHorizontal.leftTrenchOpenStart);

    public static final Bounds blueRightTrench = VerticalFlipUtil.apply(blueLeftTrench);

    public static final Bounds redLeftTrench =
        new Bounds(
            FieldConstants.fieldLength - blueLeftTrench.maxX(),
            FieldConstants.fieldLength - blueLeftTrench.minX(),
            FieldConstants.fieldWidth - blueLeftTrench.maxY(),
            FieldConstants.fieldWidth - blueLeftTrench.minY());

    public static final Bounds redRightTrench =
        new Bounds(
            FieldConstants.fieldLength - blueLeftTrench.maxX(),
            FieldConstants.fieldLength - blueLeftTrench.minX(),
            blueLeftTrench.minY(),
            blueLeftTrench.maxY());

    public static BooleanSupplier contains(Supplier<Translation2d> translation) {
      return () ->
          blueLeftTrench.contains(translation.get())
              || blueRightTrench.contains(translation.get())
              || redLeftTrench.contains(translation.get())
              || redRightTrench.contains(translation.get());
    }
  }
}
