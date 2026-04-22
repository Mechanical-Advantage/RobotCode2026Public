// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2026;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import lombok.Getter;
import lombok.Setter;
import lombok.experimental.ExtensionMethod;
import org.littletonrobotics.frc2026.Constants.RobotType;
import org.littletonrobotics.frc2026.subsystems.launcher.LauncherConstants;
import org.littletonrobotics.frc2026.subsystems.vision.VisionConstants;
import org.littletonrobotics.frc2026.util.geometry.GeomUtil;
import org.littletonrobotics.junction.Logger;

@ExtensionMethod({GeomUtil.class})
public class DarwinMechanism3d {
  private static DarwinMechanism3d measured;

  public static DarwinMechanism3d getMeasured() {
    if (measured == null) {
      measured = new DarwinMechanism3d();
    }
    return measured;
  }

  @Getter @Setter private Rotation2d hoodAngle = Rotation2d.kZero;
  @Getter @Setter private Rotation2d intakeAngle = Rotation2d.kZero;
  @Getter @Setter private double trashCompactorHeight = 0.554692;

  // These values may differ from VisionConstants as they are calibrated for the version of
  // the CAD exported for use in AdvantageScope, which may differ slightly from the actual robot.
  public static final double robotToPivotX = Units.inchesToMeters(12.480);
  public static final double pivotToSlotYOffset = Units.inchesToMeters(2.393);
  public static final double pivotToWasherNorm = Units.inchesToMeters(7.204);
  public static final double slotStartToHopperX = Units.inchesToMeters(6.789);
  public static final double slotAngle = Units.degreesToRadians(70.0);

  /** Log the component poses and camera pose. */
  public void log(String key) {
    var pivotToWasherX = pivotToWasherNorm * intakeAngle.getCos();
    var slotToWasherY = pivotToWasherNorm * intakeAngle.getSin() - pivotToSlotYOffset;
    var washerToSlotX = slotToWasherY / Math.tan(slotAngle);
    var robotToHopperX = robotToPivotX + pivotToWasherX + washerToSlotX + slotStartToHopperX;

    var hoodPose =
        LauncherConstants.robotToLauncher
            .toPose3d()
            .transformBy(
                new Transform3d(
                    0.122, 0.0, 0.0, new Rotation3d(0.0, -hoodAngle.getRadians(), Math.PI)));
    var intakePose =
        new Pose3d(
            VisionConstants.robotToPivotX,
            0.0,
            0.186,
            new Rotation3d(0.0, -intakeAngle.getRadians(), 0.0));
    var frontHopperPose = new Pose3d(robotToHopperX, 0.0, 0.37, Rotation3d.kZero);
    var trashCompactorPose = new Pose3d(0.01, 0.0, trashCompactorHeight, Rotation3d.kZero);
    Logger.recordOutput(
        key + "/Components", hoodPose, intakePose, frontHopperPose, trashCompactorPose);

    if (Constants.getRobot() == RobotType.DARWIN) {
      var cameraPose = VisionConstants.cameras[1].poseFunction().apply(Timer.getTimestamp());
      if (cameraPose.isPresent()) {
        Logger.recordOutput(
            key + "/Camera",
            new Pose3d(RobotState.getInstance().getEstimatedPose())
                .transformBy(cameraPose.get().toTransform3d()));
      }
    }
  }
}
