// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2026.subsystems.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.HashMap;
import java.util.Map;
import lombok.experimental.ExtensionMethod;
import org.littletonrobotics.frc2026.AutoFieldConstants.*;
import org.littletonrobotics.frc2026.FieldConstants;
import org.littletonrobotics.frc2026.util.geometry.GeomUtil;
import org.littletonrobotics.frc2026.util.vts.request.PathRequest;
import org.littletonrobotics.frc2026.util.vts.request.PathRequest.PathRequestBuilder;
import org.littletonrobotics.frc2026.util.vts.request.PathRequestHelpers;
import org.littletonrobotics.frc2026.util.vts.request.PathRequestSegment;
import org.littletonrobotics.frc2026.util.vts.request.PathRequestSegment.PointTarget;
import org.littletonrobotics.frc2026.util.vts.request.PathWaypoint;

@ExtensionMethod({PathRequestHelpers.class, GeomUtil.class})
public class DriveTrajectories {
  public static final Map<String, PathRequest> paths = new HashMap<>();
  public static final PointTarget hubTarget =
      new PointTarget(
          FieldConstants.Hub.innerCenterPoint.toTranslation2d(), Rotation2d.fromDegrees(10), true);
  public static final double intakeVelocityLimit = 1.8;

  public static PathRequestSegment bumpReturn =
      PathRequestSegment.builder()
          .waypoints(
              // Just after bump
              PathWaypoint.from(
                      new Pose2d(Bump.leftOuter.plus(new Translation2d(0.4, 0)), Rotation2d.kZero))
                  .build())
          .maxVelocity(3.2)
          .build();

  static {
    // MARK: Examples
    paths.put(
        "DriveForward1m",
        PathRequest.builder()
            .segments(
                PathRequestSegment.builder()
                    .waypoints(
                        PathWaypoint.from(Pose2d.kZero).build(),
                        PathWaypoint.from(new Pose2d(1.0, 0.0, Rotation2d.kZero))
                            .stopped(true)
                            .build())
                    .build())
            .build());

    paths.put(
        "NonPath",
        PathRequest.builder()
            .segments(
                PathRequestSegment.builder()
                    .waypoints(
                        PathWaypoint.from(Pose2d.kZero).build(),
                        PathWaypoint.from(new Pose2d(0.1, 0.0, Rotation2d.kZero)).build())
                    .build())
            .stopAtStart(false)
            .stopAtEnd(false)
            .build());

    // MARK: Trench Start -> NZ
    PathRequestBuilder trenchLeftStart =
        PathRequest.builder()
            .segments(
                PathRequestSegment.builder()
                    .waypoints(
                        // Starting in trench
                        PathWaypoint.from(new Pose2d(Trench.leftStart, Rotation2d.fromDegrees(-90)))
                            .build(),
                        PathWaypoint.from(Trench.leftStart).build())
                    .build());

    PathRequestBuilder trenchLeftStartOffsetToNeutralZone =
        PathRequest.builder()
            .segments(
                PathRequestSegment.builder()
                    .waypoints(
                        // Starting in trench
                        PathWaypoint.from(
                                new Pose2d(Trench.leftStartOffset, Rotation2d.fromDegrees(-90)))
                            .build(),
                        PathWaypoint.from(Trench.leftClear.plus(new Translation2d(-0.1, 0.0)))
                            .build())
                    .keepInLaneWidth(0.1)
                    .maxAngularVelocity(0.1)
                    .build());

    paths.put(
        "trenchLeftStart",
        PathRequest.builder().segments(trenchLeftStart.build().segments).stopAtEnd(false).build());

    paths.put(
        "trenchLeftStartOffsetToNeutralZone",
        PathRequest.builder()
            .segments(trenchLeftStartOffsetToNeutralZone.build().segments)
            .stopAtEnd(false)
            .build());

    /* MARK: First Sweep
     *
     *
     *
     *
     *
     * MARK: > Conservative
     */
    PathRequestBuilder tushPush =
        PathRequest.builder()
            .segments(
                PathRequestSegment.builder()
                    .waypoints(
                        // Intake into the hub
                        PathWaypoint.from(
                                new Pose2d(
                                    FieldConstants.LeftBump.farRightCorner.plus(
                                        new Translation2d(
                                            DriveConstants.fullBaseRadius + 0.40, -0.44)),
                                    Rotation2d.kPi))
                            .build())
                    .keepInLaneWidth(0.15)
                    .build(),
                PathRequestSegment.builder()
                    .waypoints(
                        // Back up to drive over bump
                        PathWaypoint.from(
                                new Pose2d(
                                    FieldConstants.LeftBump.farRightCorner.plus(
                                        new Translation2d(
                                            DriveConstants.fullBaseRadius + 0.5, 0.65)),
                                    Rotation2d.kPi))
                            .build())
                    .build());

    PathRequestBuilder conservativeSweep =
        PathRequest.builder()
            .segments(
                PathRequestSegment.builder()
                    .waypoints(
                        // Right before fuel pool
                        PathWaypoint.from(
                                new Pose2d(
                                    FieldConstants.FuelPool.leftCenter.plus(
                                        new Translation2d(
                                            -DriveConstants.fullApothemX,
                                            DriveConstants.fullApothemX)),
                                    Rotation2d.fromDegrees(-90.0)))
                            .build())
                    .keepInLaneWidth(0.35)
                    .build(),
                PathRequestSegment.builder()
                    .waypoints(
                        // Into fuel pool
                        PathWaypoint.from(
                                new Pose2d(
                                    FieldConstants.fieldLength / 2.0 - DriveConstants.fullApothemX,
                                    FieldConstants.FuelPool.leftCenter.getY()
                                        - FieldConstants.FuelPool.width / 4.0
                                        + 0.2,
                                    Rotation2d.fromDegrees(-90)))
                            .build())
                    .keepInLaneWidth(0.05)
                    .maxVelocity(intakeVelocityLimit)
                    .maxAngularVelocity(0.1)
                    .build(),
                PathRequestSegment.builder()
                    .waypoints(
                        // Start turn back to bump
                        PathWaypoint.from(
                                new Pose2d(
                                        FieldConstants.fieldLength / 2.0
                                            - DriveConstants.fullApothemX,
                                        FieldConstants.FuelPool.leftCenter.getY()
                                            - FieldConstants.FuelPool.width / 4.0
                                            + 0.2,
                                        Rotation2d.fromDegrees(-90))
                                    .plus(new Transform2d(0.3, 0.0, Rotation2d.fromDegrees(-20))))
                            .build())
                    .build());

    paths.put(
        "leftTrenchStartConservativeSweep",
        PathRequest.builder()
            .segments(trenchLeftStart.build().segments)
            .segments(conservativeSweep.build().segments)
            .segments(tushPush.build().segments)
            .stopAtEnd(false)
            .build());

    paths.put(
        "leftTrenchStartOffsetConservativeSweep",
        PathRequest.builder()
            .segments(trenchLeftStartOffsetToNeutralZone.build().segments)
            .segments(conservativeSweep.build().segments)
            .segments(tushPush.build().segments)
            .stopAtEnd(false)
            .build());

    paths.put(
        "leftBumpConservativeSweep",
        PathRequest.builder()
            .segments(bumpReturn)
            .segments(conservativeSweep.build().segments)
            .segments(tushPush.build().segments)
            .stopAtEnd(false)
            .build());

    // MARK: > Neutral
    PathRequestBuilder neutralSweep =
        PathRequest.builder()
            .segments(
                PathRequestSegment.builder()
                    .waypoints(
                        // Robot width away from middle of fuel pool
                        PathWaypoint.from(
                                new Pose2d(
                                    FieldConstants.FuelPool.leftCenter.plus(
                                        new Translation2d(-0.2, DriveConstants.fullApothemX)),
                                    Rotation2d.fromDegrees(-90.0)))
                            .build())
                    .build(),
                PathRequestSegment.builder()
                    .waypoints(
                        // Intermediate point in fuel pool
                        PathWaypoint.from(
                                new Pose2d(
                                    FieldConstants.fieldCenter.plus(
                                        new Translation2d(
                                            0.0, FieldConstants.FuelPool.width * 3.0 / 8.0)),
                                    Rotation2d.fromDegrees(-106)))
                            .build())
                    .maxVelocity(intakeVelocityLimit)
                    .build(),
                PathRequestSegment.builder()
                    .waypoints(
                        // Through fuel pool
                        PathWaypoint.from(
                                new Pose2d(
                                    FieldConstants.fieldCenter.plus(
                                        new Translation2d(
                                            -0.2, FieldConstants.FuelPool.width / 4.0)),
                                    Rotation2d.fromDegrees(-125)))
                            .build())
                    .keepInLaneWidth(0.03)
                    .maxAngularVelocity(0.8)
                    .maxVelocity(intakeVelocityLimit)
                    .build());

    paths.put(
        "leftTrenchStartNeutralSweep",
        PathRequest.builder()
            .segments(trenchLeftStart.build().segments)
            .segments(neutralSweep.build().segments)
            .segments(tushPush.build().segments)
            .stopAtEnd(false)
            .build());

    paths.put(
        "leftTrenchStartOffsetNeutralSweep",
        PathRequest.builder()
            .segments(trenchLeftStartOffsetToNeutralZone.build().segments)
            .segments(neutralSweep.build().segments)
            .segments(tushPush.build().segments)
            .stopAtEnd(false)
            .build());

    paths.put(
        "leftBumpNeutralSweep",
        PathRequest.builder()
            .segments(
                bumpReturn,
                PathRequestSegment.builder()
                    .waypoints(
                        // Just before fuel pool corner
                        PathWaypoint.from(
                                new Pose2d(
                                        FieldConstants.FuelPool.nearLeftCorner,
                                        Rotation2d.fromDegrees(-66.0))
                                    .transformBy(new Transform2d(-0.65, 0.0, Rotation2d.kZero))
                                    .getTranslation())
                            .build())
                    .build())
            .segments(neutralSweep.build().segments)
            .segments(tushPush.build().segments)
            .stopAtEnd(false)
            .build());

    // MARK: > Flightless
    PathRequestBuilder flightlessSweep =
        PathRequest.builder()
            .segments(
                PathRequestSegment.builder()
                    .waypoints(
                        PathWaypoint.from(
                                new Pose2d(
                                    (FieldConstants.LinesVertical.neutralZoneNear
                                            + FieldConstants.fieldLength / 2.0)
                                        / 2.0,
                                    FieldConstants.LinesHorizontal.leftBumpMiddle,
                                    Rotation2d.fromDegrees(-90)))
                            .build())
                    .build(),
                PathRequestSegment.builder()
                    .waypoints(
                        PathWaypoint.from(
                                new Pose2d(
                                    (FieldConstants.LinesVertical.neutralZoneNear
                                            + FieldConstants.fieldLength / 2.0)
                                        / 2.0,
                                    FieldConstants.fieldWidth / 2.0 - DriveConstants.fullApothemX,
                                    Rotation2d.fromDegrees(-90)))
                            .build())
                    .keepInLaneWidth(0.05)
                    .maxAngularVelocity(0.1)
                    .build(),
                PathRequestSegment.builder()
                    .waypoints(
                        PathWaypoint.from(
                                new Pose2d(
                                    (FieldConstants.LinesVertical.neutralZoneNear
                                                + FieldConstants.fieldLength / 2.0)
                                            / 2.0
                                        - DriveConstants.fullApothemX,
                                    FieldConstants.fieldWidth / 2.0 - DriveConstants.fullWidthX,
                                    Rotation2d.kPi))
                            .build(),
                        PathWaypoint.from(
                                new Pose2d(
                                    FieldConstants.LinesVertical.neutralZoneNear
                                        + DriveConstants.fullApothemX
                                        + 0.27,
                                    FieldConstants.fieldWidth / 2.0 - DriveConstants.fullApothemX,
                                    Rotation2d.fromDegrees(90)))
                            .build())
                    .build(),
                PathRequestSegment.builder()
                    .waypoints(
                        PathWaypoint.from(
                                new Pose2d(
                                    FieldConstants.LinesVertical.neutralZoneNear
                                        + DriveConstants.fullApothemX
                                        + 0.27,
                                    FieldConstants.fieldWidth / 2.0
                                        + DriveConstants.fullApothemX
                                        + 0.7,
                                    Rotation2d.fromDegrees(90)))
                            .build())
                    .keepInLaneWidth(0.05)
                    .maxAngularVelocity(0.1)
                    .build());

    paths.put(
        "leftTrenchStartOffsetFlightlessSweep",
        PathRequest.builder()
            .segments(
                PathRequestSegment.builder()
                    .waypoints(
                        // Starting in trench
                        PathWaypoint.from(
                                new Pose2d(Trench.leftStartOffset, Rotation2d.fromDegrees(-90)))
                            .build(),
                        PathWaypoint.from(Trench.leftClear).build())
                    .keepInLaneWidth(0.05)
                    .build())
            .segments(flightlessSweep.build().segments)
            .stopAtEnd(false)
            .build());

    paths.put(
        "leftBumpFlightlessSweep",
        PathRequest.builder()
            .segments(bumpReturn)
            .segments(flightlessSweep.build().segments)
            .stopAtEnd(false)
            .build());

    /* MARK: Alliance Zone
     *
     *
     *
     *
     *
     * MARK: > Launch to NZ
     */
    PathRequestBuilder launchLeftBumpThroughTrench =
        PathRequest.builder()
            .segments(
                PathRequestSegment.builder()
                    .waypoints(
                        // Return from bump
                        PathWaypoint.from(Launch.leftBump).build(),

                        // Back up
                        PathWaypoint.from(
                                Launch.leftBump.getTranslation().plus(new Translation2d(-0.3, 0.5)))
                            .build())
                    .maxVelocity(0.8)
                    .build(),
                PathRequestSegment.builder()
                    .waypoints(
                        // Start curve into trench
                        PathWaypoint.from(
                                Launch.leftBump.getTranslation().plus(new Translation2d(-0.3, 1.2)))
                            .build(),
                        // Prepare to enter trench
                        PathWaypoint.from(Trench.leftEntry).build())
                    .pointAt(hubTarget)
                    .maxVelocity(0.8)
                    .maxAcceleration(0.6)
                    .build(),
                PathRequestSegment.builder()
                    .waypoints(
                        // Approximate end of LOTM, start turning under trench
                        PathWaypoint.from(
                                new Pose2d(Trench.leftBeforeBar, Rotation2d.fromDegrees(68)))
                            .build(),
                        // Force turn before through trench
                        PathWaypoint.from(
                                new Pose2d(
                                    FieldConstants.LeftTrench.center.plus(
                                        new Translation2d(0.15, 0.0)),
                                    Rotation2d.kZero))
                            .build(),
                        PathWaypoint.from(
                                new Pose2d(
                                    Trench.leftClear.plus(new Translation2d(-0.6, 0.0)),
                                    Rotation2d.kZero))
                            .build())
                    .keepInLaneWidth(0.03)
                    .build());

    paths.put(
        "launchLeftBumpThroughTrench",
        PathRequest.builder()
            .segments(launchLeftBumpThroughTrench.build().segments)
            .stopAtEnd(false)
            .build());

    PathRequestBuilder launchLeftBumpThroughTrenchKachow =
        PathRequest.builder()
            .segments(
                PathRequestSegment.builder()
                    .waypoints(
                        // Continue after launching
                        PathWaypoint.from(Launch.leftBump).build(),

                        // Prepare to enter trench
                        PathWaypoint.from(new Pose2d(Trench.leftEntry, Rotation2d.kZero)).build())
                    .build(),
                PathRequestSegment.builder()
                    .waypoints(
                        // Drive through and clear trench
                        PathWaypoint.from(
                                new Pose2d(
                                    Trench.leftClear.plus(new Translation2d(-0.6, 0.0)),
                                    Rotation2d.kZero))
                            .build())
                    .keepInLaneWidth(0.03)
                    .maxAngularVelocity(0.05)
                    .build());

    paths.put(
        "launchLeftBumpThroughTrenchKachow",
        PathRequest.builder()
            .segments(launchLeftBumpThroughTrenchKachow.build().segments)
            .stopAtEnd(false)
            .build());

    PathRequestBuilder launchLeftTrenchThroughTrenchKachow =
        PathRequest.builder()
            .segments(
                PathRequestSegment.builder()
                    .waypoints(
                        // Continue after launching
                        PathWaypoint.from(Launch.leftTrench).build(),

                        // Prepare to enter trench
                        PathWaypoint.from(new Pose2d(Trench.leftEntry, Rotation2d.kZero)).build())
                    .build(),
                PathRequestSegment.builder()
                    .waypoints(
                        // Drive through and clear trench
                        PathWaypoint.from(
                                new Pose2d(
                                    Trench.leftClear.plus(new Translation2d(-0.6, 0.0)),
                                    Rotation2d.kZero))
                            .build())
                    .keepInLaneWidth(0.03)
                    .maxAngularVelocity(0.05)
                    .build());

    paths.put(
        "launchLeftTrenchThroughTrenchKachow",
        PathRequest.builder()
            .segments(launchLeftTrenchThroughTrenchKachow.build().segments)
            .stopAtEnd(false)
            .build());

    /* MARK: Second Sweep
     *
     *
     *
     *
     *
     * MARK: > Coast
     *
     *
     *
     *
     *
     * MARK: >> Regular End
     */
    PathRequestBuilder trenchLeftEntryToFarHub =
        PathRequest.builder()
            .segments(
                PathRequestSegment.builder()
                    .waypoints(
                        // Prepare to enter trench
                        PathWaypoint.from(new Pose2d(Trench.leftEntry, Rotation2d.kZero)).build())
                    .build(),
                PathRequestSegment.builder()
                    .waypoints(
                        // Drive through and clear trench
                        PathWaypoint.from(
                                new Pose2d(
                                    Trench.leftClear.plus(new Translation2d(-0.6, 0.0)),
                                    Rotation2d.kZero))
                            .build())
                    .keepInLaneWidth(0.03)
                    .maxAngularVelocity(0.1)
                    .build(),
                PathRequestSegment.builder()
                    .waypoints(
                        // Turn towards hub
                        PathWaypoint.from(
                                new Pose2d(
                                    Trench.leftClear.plus(new Translation2d(0.0, -0.1)),
                                    Rotation2d.fromDegrees(-40)))
                            .build(),

                        // Drive towards hub
                        PathWaypoint.from(
                                new Pose2d(
                                    FieldConstants.FuelPool.leftCenter.plus(
                                        new Translation2d(0.0, -0.25)),
                                    Rotation2d.fromDegrees(-40)))
                            .build())
                    .build());

    paths.put(
        "launchLeftBumpToFarHubKachow",
        PathRequest.builder()
            .segments(
                PathRequestSegment.builder()
                    .waypoints(PathWaypoint.from(Launch.leftBump).build())
                    .build())
            .segments(trenchLeftEntryToFarHub.build().segments)
            .stopAtEnd(false)
            .build());

    paths.put(
        "launchLeftTrenchToFarHubKachow",
        PathRequest.builder()
            .segments(
                PathRequestSegment.builder()
                    .waypoints(PathWaypoint.from(Launch.leftTrench).build())
                    .build())
            .segments(trenchLeftEntryToFarHub.build().segments)
            .stopAtEnd(false)
            .build());

    PathRequestBuilder trenchLeftEntryToFarBump =
        PathRequest.builder()
            .segments(
                PathRequestSegment.builder()
                    .waypoints(
                        // Prepare to enter trench
                        PathWaypoint.from(new Pose2d(Trench.leftEntry, Rotation2d.kZero)).build())
                    .build(),
                PathRequestSegment.builder()
                    .waypoints(
                        // Drive through and clear trench
                        PathWaypoint.from(
                                new Pose2d(
                                    Trench.leftClear.plus(new Translation2d(-0.6, 0.0)),
                                    Rotation2d.kZero))
                            .build())
                    .keepInLaneWidth(0.03)
                    .maxAngularVelocity(0.1)
                    .build(),
                PathRequestSegment.builder()
                    .waypoints(
                        // Rotate
                        PathWaypoint.from(
                                new Pose2d(
                                    Trench.leftClear.plus(new Translation2d(0.5, -0.2)),
                                    Rotation2d.fromDegrees(-90)))
                            .build())
                    .build(),
                PathRequestSegment.builder()
                    .waypoints(
                        // Towards bump
                        PathWaypoint.from(
                                new Pose2d(
                                    FieldConstants.FuelPool.leftCenter.plus(
                                        new Translation2d(0.0, -0.1)),
                                    Rotation2d.fromDegrees(-90)))
                            .build())
                    .build());

    paths.put(
        "launchLeftBumpToFarBumpKachow",
        PathRequest.builder()
            .segments(
                PathRequestSegment.builder()
                    .waypoints(PathWaypoint.from(Launch.leftBump).build())
                    .build())
            .segments(trenchLeftEntryToFarBump.build().segments)
            .stopAtEnd(false)
            .build());

    paths.put(
        "launchLeftTrenchToFarBumpKachow",
        PathRequest.builder()
            .segments(
                PathRequestSegment.builder()
                    .waypoints(PathWaypoint.from(Launch.leftTrench).build())
                    .build())
            .segments(trenchLeftEntryToFarBump.build().segments)
            .stopAtEnd(false)
            .build());

    PathRequestBuilder trenchLeftEntryToFarTrenchToFarTrench =
        PathRequest.builder()
            .segments(
                PathRequestSegment.builder()
                    .waypoints(
                        // Prepare to enter trench
                        PathWaypoint.from(new Pose2d(Trench.leftEntry, Rotation2d.kZero)).build())
                    .build(),
                PathRequestSegment.builder()
                    .waypoints(
                        // Drive through and clear trench
                        PathWaypoint.from(
                                new Pose2d(
                                    Trench.leftClear.plus(new Translation2d(-0.6, 0.0)),
                                    Rotation2d.kZero))
                            .build())
                    .keepInLaneWidth(0.03)
                    .maxAngularVelocity(0.1)
                    .build(),
                PathRequestSegment.builder()
                    .waypoints(
                        // Orient towards trench
                        PathWaypoint.from(
                                new Pose2d(
                                    FieldConstants.fieldLength / 2.0
                                        - DriveConstants.fullApothemX
                                        + 0.01,
                                    Trench.leftEntry.getY(),
                                    Rotation2d.fromDegrees(-90)))
                            .build())
                    .keepInLaneWidth(0.05)
                    .build());

    paths.put(
        "launchLeftBumpToFarTrenchKachow",
        PathRequest.builder()
            .segments(
                PathRequestSegment.builder()
                    .waypoints(PathWaypoint.from(Launch.leftBump).build())
                    .build())
            .segments(trenchLeftEntryToFarTrenchToFarTrench.build().segments)
            .stopAtEnd(false)
            .build());

    paths.put(
        "launchLeftTrenchToFarTrenchKachow",
        PathRequest.builder()
            .segments(
                PathRequestSegment.builder()
                    .waypoints(PathWaypoint.from(Launch.leftTrench).build())
                    .build())
            .segments(trenchLeftEntryToFarTrenchToFarTrench.build().segments)
            .stopAtEnd(false)
            .build());

    // MARK: >> Tilde End
    PathRequestBuilder tildeToFarHub =
        PathRequest.builder()
            .segments(
                PathRequestSegment.builder()
                    .waypoints(
                        PathWaypoint.from(
                                new Pose2d(
                                    FieldConstants.FuelPool.rightCenter,
                                    Rotation2d.fromDegrees(40)))
                            .build())
                    .build());

    PathRequestBuilder tildeToFarBump =
        PathRequest.builder()
            .segments(
                PathRequestSegment.builder()
                    .waypoints(
                        PathWaypoint.from(
                                new Pose2d(
                                    FieldConstants.FuelPool.rightCenter.plus(
                                        new Translation2d(
                                            -DriveConstants.fullApothemX,
                                            -DriveConstants.fullApothemX)),
                                    Rotation2d.fromDegrees(90)))
                            .build())
                    .build(),
                PathRequestSegment.builder()
                    .waypoints(
                        PathWaypoint.from(
                                new Pose2d(
                                    FieldConstants.FuelPool.rightCenter
                                        .plus(
                                            new Translation2d(
                                                -DriveConstants.fullApothemX,
                                                -DriveConstants.fullApothemX))
                                        .plus(new Translation2d(0.35, 0.1)),
                                    Rotation2d.fromDegrees(90)))
                            .build())
                    .keepInLaneWidth(0.05)
                    .build());

    PathRequestBuilder tildeToFarTrench =
        PathRequest.builder()
            .segments(
                PathRequestSegment.builder()
                    .waypoints(
                        PathWaypoint.from(
                                new Pose2d(
                                    FieldConstants.fieldLength / 2.0
                                        - DriveConstants.fullApothemX
                                        - 0.3,
                                    DriveConstants.fullApothemX + 0.25,
                                    Rotation2d.fromDegrees(90)))
                            .build())
                    .build(),
                PathRequestSegment.builder()
                    .waypoints(
                        PathWaypoint.from(
                                new Pose2d(
                                    FieldConstants.fieldLength / 2.0 - DriveConstants.fullApothemX,
                                    DriveConstants.fullApothemX + 0.25,
                                    Rotation2d.fromDegrees(90)))
                            .build())
                    .keepInLaneWidth(0.05)
                    .build());

    PathRequestBuilder tildeToNone =
        PathRequest.builder()
            .segments(
                PathRequestSegment.builder()
                    .waypoints(
                        PathWaypoint.from(
                                new Pose2d(
                                    FieldConstants.FuelPool.rightCenter.plus(
                                        new Translation2d(0.0, -DriveConstants.fullApothemX)),
                                    Rotation2d.fromDegrees(90)))
                            .build())
                    .build());

    // MARK: > Coriolis
    PathRequestBuilder coriolis =
        PathRequest.builder()
            .segments(
                PathRequestSegment.builder()
                    // Turn into fuel pool
                    .waypoints(
                        PathWaypoint.from(
                                new Pose2d(
                                    FieldConstants.FuelPool.nearLeftCorner.plus(
                                        new Translation2d(-0.2, DriveConstants.fullApothemX + 0.1)),
                                    Rotation2d.fromDegrees(-60)))
                            .build())
                    .build(),
                PathRequestSegment.builder()
                    // Conservatively enter fuel pool
                    .waypoints(
                        PathWaypoint.from(
                                new Pose2d(
                                    FieldConstants.FuelPool.nearLeftCorner.plus(
                                        new Translation2d(0.1, -0.1)),
                                    Rotation2d.fromDegrees(-80)))
                            .build())
                    .build(),
                PathRequestSegment.builder()
                    // Conservatively drive through fuel pool
                    .waypoints(
                        PathWaypoint.from(
                                new Pose2d(
                                    FieldConstants.fieldCenter
                                        .plus(
                                            new Translation2d(
                                                -DriveConstants.fullApothemX,
                                                DriveConstants.fullApothemX))
                                        .plus(new Translation2d(-0.1, 0.4)),
                                    Rotation2d.fromDegrees(-90)))
                            .build())
                    .keepInLaneWidth(0.05)
                    .build(),
                PathRequestSegment.builder()
                    .waypoints(
                        // Drive behind hub
                        PathWaypoint.from(
                                new Pose2d(
                                    FieldConstants.LinesVertical.neutralZoneNear
                                        + DriveConstants.fullWidthX * 3.0 / 2.0
                                        + 0.5,
                                    FieldConstants.fieldWidth / 2.0 + 0.4,
                                    Rotation2d.kPi))
                            .build())
                    .build(),
                PathRequestSegment.builder()
                    .waypoints(
                        // Intake into the hub
                        PathWaypoint.from(
                                new Pose2d(
                                    FieldConstants.LeftBump.farRightCorner.plus(
                                        new Translation2d(
                                            DriveConstants.fullBaseRadius + 0.40, -0.44)),
                                    Rotation2d.kPi))
                            .build())
                    .maxAngularVelocity(0.1)
                    .build(),
                PathRequestSegment.builder()
                    .waypoints(
                        // Back up to drive over bump
                        PathWaypoint.from(
                                new Pose2d(
                                    FieldConstants.LeftBump.farRightCorner.plus(
                                        new Translation2d(
                                            DriveConstants.fullBaseRadius + 0.5, 0.65)),
                                    Rotation2d.kPi))
                            .build())
                    .build());

    paths.put(
        "leftTrenchStartOffsetCoriolisSweep",
        PathRequest.builder()
            .segments(trenchLeftStartOffsetToNeutralZone.build().segments)
            .segments(coriolis.build().segments)
            .stopAtEnd(false)
            .build());

    paths.put(
        "launchLeftBumpCoriolisSweep",
        PathRequest.builder()
            .segments(launchLeftBumpThroughTrench.build().segments)
            .segments(coriolis.build().segments)
            .stopAtEnd(false)
            .build());

    paths.put(
        "launchLeftBumpCoriolisSweepKachow",
        PathRequest.builder()
            .segments(launchLeftBumpThroughTrenchKachow.build().segments)
            .segments(coriolis.build().segments)
            .stopAtEnd(false)
            .build());

    // MARK: > Behind Hub
    PathRequestBuilder behindHub =
        PathRequest.builder()
            .segments(
                PathRequestSegment.builder()
                    .waypoints(
                        // Fix rotation before robot has crossed bump
                        PathWaypoint.from(
                                new Pose2d(
                                    FieldConstants.LeftBump.farLeftCorner.plus(
                                        new Translation2d(DriveConstants.fullApothemX + 0.4, 0)),
                                    Rotation2d.fromDegrees(-105)))
                            .build())
                    .build(),
                PathRequestSegment.builder()
                    .waypoints(
                        // Behind the hub
                        PathWaypoint.from(
                                new Pose2d(
                                    FieldConstants.LeftBump.farRightCorner.plus(
                                        new Translation2d(
                                            DriveConstants.fullApothemX + 0.4,
                                            -DriveConstants.fullApothemX + 0.2)),
                                    Rotation2d.fromDegrees(-105)))
                            .build())
                    .keepInLaneWidth(0.05)
                    .maxAngularVelocity(0.1)
                    .build());

    PathRequestBuilder behindHubFriendship =
        PathRequest.builder()
            .segments(
                PathRequestSegment.builder()
                    .waypoints(
                        // Fix rotation before robot has crossed bump
                        PathWaypoint.from(
                                new Pose2d(
                                    FieldConstants.LeftBump.farLeftCorner.plus(
                                        new Translation2d(DriveConstants.fullApothemX + 0.4, 0)),
                                    Rotation2d.fromDegrees(-105)))
                            .build())
                    .build(),
                PathRequestSegment.builder()
                    .waypoints(
                        // Behind the hub
                        PathWaypoint.from(
                                new Pose2d(
                                    FieldConstants.LeftBump.farRightCorner.plus(
                                        new Translation2d(
                                            DriveConstants.fullApothemX + 0.4,
                                            DriveConstants.fullApothemX + 0.2)),
                                    Rotation2d.fromDegrees(-105)))
                            .build())
                    .keepInLaneWidth(0.05)
                    .maxAngularVelocity(0.1)
                    .build());

    PathRequestBuilder launchLeftBumpToBehindHub =
        PathRequest.builder()
            .segments(launchLeftBumpThroughTrench.build().segments)
            .segments(behindHub.build().segments);

    PathRequestBuilder launchLeftBumpToBehindHubKachow =
        PathRequest.builder()
            .segments(launchLeftBumpThroughTrenchKachow.build().segments)
            .segments(behindHub.build().segments);

    PathRequestBuilder launchLeftBumpToBehindHubFriendship =
        PathRequest.builder()
            .segments(launchLeftBumpThroughTrench.build().segments)
            .segments(behindHubFriendship.build().segments);

    PathRequestBuilder launchLeftBumpToBehindHubFriendshipKachow =
        PathRequest.builder()
            .segments(launchLeftBumpThroughTrenchKachow.build().segments)
            .segments(behindHubFriendship.build().segments);

    PathRequestBuilder bumpLeftOuterToBehindHub =
        PathRequest.builder()
            .segments(
                bumpReturn,
                PathRequestSegment.builder()
                    .waypoints(
                        // Behind the hub
                        PathWaypoint.from(
                                new Pose2d(
                                    FieldConstants.LeftBump.farRightCorner.plus(
                                        new Translation2d(
                                            DriveConstants.fullApothemX + 0.35,
                                            -DriveConstants.fullApothemX + 0.2)),
                                    Rotation2d.fromDegrees(-90)))
                            .build())
                    .build());

    PathRequestBuilder bumpLeftOuterToBehindHubFriendship =
        PathRequest.builder()
            .segments(
                bumpReturn,
                PathRequestSegment.builder()
                    .waypoints(
                        // Point behind hub
                        PathWaypoint.from(
                                new Pose2d(
                                    FieldConstants.LeftBump.farRightCorner.plus(
                                        new Translation2d(
                                            DriveConstants.fullApothemX + 0.3,
                                            DriveConstants.fullApothemX)),
                                    Rotation2d.fromDegrees(-90)))
                            .build())
                    .build());

    paths.put(
        "launchLeftBumpToBehindHub",
        PathRequest.builder()
            .segments(launchLeftBumpToBehindHub.build().segments)
            .stopAtEnd(false)
            .build());

    paths.put(
        "launchLeftBumpToBehindHubKachow",
        PathRequest.builder()
            .segments(launchLeftBumpToBehindHubKachow.build().segments)
            .stopAtEnd(false)
            .build());

    paths.put(
        "launchLeftBumpToBehindHubFriendship",
        PathRequest.builder()
            .segments(launchLeftBumpToBehindHubFriendship.build().segments)
            .stopAtEnd(false)
            .build());

    paths.put(
        "launchLeftBumpToBehindHubFriendshipKachow",
        PathRequest.builder()
            .segments(launchLeftBumpToBehindHubFriendshipKachow.build().segments)
            .stopAtEnd(false)
            .build());

    paths.put(
        "bumpLeftOuterToBehindHub",
        PathRequest.builder()
            .segments(bumpLeftOuterToBehindHub.build().segments)
            .stopAtEnd(false)
            .build());

    paths.put(
        "bumpLeftOuterToBehindHubFriendship",
        PathRequest.builder()
            .segments(bumpLeftOuterToBehindHubFriendship.build().segments)
            .stopAtEnd(false)
            .build());

    // MARK: > Tilde
    PathRequestBuilder tildeSweepTrench =
        PathRequest.builder()
            .segments(behindHub.build().segments)
            .segments(
                PathRequestSegment.builder()
                    .waypoints(
                        PathWaypoint.from(
                                new Pose2d(
                                    FieldConstants.RightBump.farRightCorner.plus(
                                        new Translation2d(DriveConstants.fullApothemX + 0.5, 0)),
                                    Rotation2d.fromDegrees(-90)))
                            .build())
                    .keepInLaneWidth(0.1)
                    .maxAngularVelocity(0.1)
                    .build());

    PathRequestBuilder tildeSweepBump =
        PathRequest.builder()
            .segments(bumpLeftOuterToBehindHub.build().segments)
            .segments(
                PathRequestSegment.builder()
                    .waypoints(
                        PathWaypoint.from(
                                new Pose2d(
                                    FieldConstants.RightBump.farRightCorner.plus(
                                        new Translation2d(DriveConstants.fullApothemX + 0.5, 0)),
                                    Rotation2d.fromDegrees(-90)))
                            .build())
                    .keepInLaneWidth(0.1)
                    .maxAngularVelocity(0.1)
                    .build());

    paths.put(
        "launchLeftBumpTildeSweepToFarHub",
        PathRequest.builder()
            .segments(launchLeftBumpThroughTrench.build().segments)
            .segments(tildeSweepTrench.build().segments)
            .segments(tildeToFarHub.build().segments)
            .stopAtEnd(false)
            .build());

    paths.put(
        "launchLeftBumpTildeSweepToFarBump",
        PathRequest.builder()
            .segments(launchLeftBumpThroughTrench.build().segments)
            .segments(tildeSweepTrench.build().segments)
            .segments(tildeToFarBump.build().segments)
            .stopAtEnd(false)
            .build());

    paths.put(
        "launchLeftBumpTildeSweepToFarTrench",
        PathRequest.builder()
            .segments(launchLeftBumpThroughTrench.build().segments)
            .segments(tildeSweepTrench.build().segments)
            .segments(tildeToFarTrench.build().segments)
            .stopAtEnd(false)
            .build());

    paths.put(
        "launchLeftBumpTildeSweepToNone",
        PathRequest.builder()
            .segments(launchLeftBumpThroughTrench.build().segments)
            .segments(tildeSweepTrench.build().segments)
            .segments(tildeToNone.build().segments)
            .build());

    paths.put(
        "launchLeftBumpTildeSweepToFarHubKachow",
        PathRequest.builder()
            .segments(launchLeftBumpThroughTrenchKachow.build().segments)
            .segments(tildeSweepTrench.build().segments)
            .segments(tildeToFarHub.build().segments)
            .stopAtEnd(false)
            .build());

    paths.put(
        "launchLeftBumpTildeSweepToFarBumpKachow",
        PathRequest.builder()
            .segments(launchLeftBumpThroughTrenchKachow.build().segments)
            .segments(tildeSweepTrench.build().segments)
            .segments(tildeToFarBump.build().segments)
            .stopAtEnd(false)
            .build());

    paths.put(
        "launchLeftBumpTildeSweepToFarTrenchKachow",
        PathRequest.builder()
            .segments(launchLeftBumpThroughTrenchKachow.build().segments)
            .segments(tildeSweepTrench.build().segments)
            .segments(tildeToFarTrench.build().segments)
            .stopAtEnd(false)
            .build());

    paths.put(
        "launchLeftBumpTildeSweepToNoneKachow",
        PathRequest.builder()
            .segments(launchLeftBumpThroughTrenchKachow.build().segments)
            .segments(tildeSweepTrench.build().segments)
            .segments(tildeToNone.build().segments)
            .build());

    paths.put(
        "bumpLeftOuterTildeSweepToFarHub",
        PathRequest.builder()
            .segments(bumpReturn)
            .segments(tildeSweepBump.build().segments)
            .segments(tildeToFarHub.build().segments)
            .stopAtEnd(false)
            .build());

    paths.put(
        "bumpLeftOuterTildeSweepToFarBump",
        PathRequest.builder()
            .segments(bumpReturn)
            .segments(tildeSweepBump.build().segments)
            .segments(tildeToFarBump.build().segments)
            .stopAtEnd(false)
            .build());

    paths.put(
        "bumpLeftOuterTildeSweepToFarTrench",
        PathRequest.builder()
            .segments(bumpReturn)
            .segments(tildeSweepBump.build().segments)
            .segments(tildeToFarTrench.build().segments)
            .stopAtEnd(false)
            .build());

    paths.put(
        "bumpLeftOuterTildeSweepToNone",
        PathRequest.builder()
            .segments(bumpReturn)
            .segments(tildeSweepBump.build().segments)
            .segments(tildeToNone.build().segments)
            .build());

    // MARK: > Davis
    PathRequestBuilder behindHubThroughDavis =
        PathRequest.builder()
            .segments(
                // Curve into a sweep of the centerline
                PathRequestSegment.builder()
                    .waypoints(
                        // Start turning towards the center
                        PathWaypoint.from(
                                new Pose2d(
                                    FieldConstants.LinesVertical.neutralZoneNear
                                        + DriveConstants.fullWidthX
                                        + 0.2,
                                    FieldConstants.fieldWidth / 2.0
                                        - DriveConstants.fullBaseRadius / 2.0
                                        + 0.1,
                                    Rotation2d.fromDegrees(-30)))
                            .build(),

                        // Turn into center
                        PathWaypoint.from(
                                new Pose2d(
                                    FieldConstants.fieldCenter.plus(
                                        new Translation2d(-0.5 - 0.7, -0.5)),
                                    Rotation2d.fromDegrees(27)))
                            .build(),

                        // Intake through center
                        PathWaypoint.from(
                                new Pose2d(
                                    FieldConstants.fieldCenter.plus(new Translation2d(-0.7, 0.0)),
                                    Rotation2d.fromDegrees(90)))
                            .build(),
                        PathWaypoint.from(
                                new Pose2d(
                                    FieldConstants.fieldCenter.plus(new Translation2d(-0.7, 1.2)),
                                    Rotation2d.fromDegrees(120)))
                            .build())
                    .build());

    PathRequestBuilder behindHubThroughDavisFriendship =
        PathRequest.builder()
            .segments(
                // Curve into a sweep of the centerline
                PathRequestSegment.builder()
                    .waypoints(
                        // Start turning towards the center
                        PathWaypoint.from(
                                new Pose2d(
                                    FieldConstants.LeftBump.farRightCorner.getX() + 1.0,
                                    FieldConstants.LeftBump.farRightCorner.getY(),
                                    Rotation2d.fromDegrees(-37)))
                            .build(),
                        // Intermediate to follow through curve
                        PathWaypoint.from(
                                new Pose2d(
                                    FieldConstants.fieldCenter.plus(
                                        new Translation2d(
                                            -DriveConstants.fullApothemX - 0.7,
                                            DriveConstants.fullApothemX - 0.15)),
                                    Rotation2d.fromDegrees(30)))
                            .build(),
                        PathWaypoint.from(
                                new Pose2d(
                                    FieldConstants.fieldCenter.plus(new Translation2d(-0.7, 1.0)),
                                    Rotation2d.fromDegrees(90)))
                            .build(),
                        PathWaypoint.from(
                                new Pose2d(
                                    FieldConstants.fieldCenter.plus(new Translation2d(-0.7, 1.5)),
                                    Rotation2d.fromDegrees(120)))
                            .build())
                    .build());

    paths.put(
        "leftTrenchStartOffsetDavisSweep",
        PathRequest.builder()
            .segments(trenchLeftStartOffsetToNeutralZone.build().segments)
            .segments(behindHub.build().segments)
            .segments(behindHubThroughDavis.build().segments)
            .stopAtEnd(false)
            .build());

    paths.put(
        "launchLeftBumpDavisSweep",
        PathRequest.builder()
            .segments(launchLeftBumpToBehindHub.build().segments)
            .segments(behindHubThroughDavis.build().segments)
            .stopAtEnd(false)
            .build());

    paths.put(
        "launchLeftBumpDavisSweepKachow",
        PathRequest.builder()
            .segments(launchLeftBumpToBehindHubKachow.build().segments)
            .segments(behindHubThroughDavis.build().segments)
            .stopAtEnd(false)
            .build());

    paths.put(
        "leftTrenchStartOffsetDavisSweepFriendship",
        PathRequest.builder()
            .segments(trenchLeftStartOffsetToNeutralZone.build().segments)
            .segments(behindHubFriendship.build().segments)
            .segments(behindHubThroughDavisFriendship.build().segments)
            .stopAtEnd(false)
            .build());

    paths.put(
        "launchLeftBumpDavisSweepFriendship",
        PathRequest.builder()
            .segments(launchLeftBumpToBehindHubFriendship.build().segments)
            .segments(behindHubThroughDavisFriendship.build().segments)
            .stopAtEnd(false)
            .build());

    paths.put(
        "launchLeftBumpDavisSweepFriendshipKachow",
        PathRequest.builder()
            .segments(launchLeftBumpToBehindHubFriendshipKachow.build().segments)
            .segments(behindHubThroughDavisFriendship.build().segments)
            .stopAtEnd(false)
            .build());

    paths.put(
        "bumpLeftOuterDavisSweep",
        PathRequest.builder()
            .segments(bumpLeftOuterToBehindHub.build().segments)
            .segments(behindHubThroughDavis.build().segments)
            .stopAtEnd(false)
            .build());

    paths.put(
        "bumpLeftOuterDavisSweepFriendship",
        PathRequest.builder()
            .segments(bumpLeftOuterToBehindHubFriendship.build().segments)
            .segments(behindHubThroughDavisFriendship.build().segments)
            .stopAtEnd(false)
            .build());

    // MARK: Monopoly Salesman
    PathRequestBuilder depotLeftToRight =
        PathRequest.builder()
            .segments(
                PathRequestSegment.builder()
                    .waypoints(
                        // Edge of depot
                        PathWaypoint.from(
                                new Pose2d(Depot.leftThrough, Rotation2d.fromDegrees(-105)))
                            .build())
                    .keepInLaneWidth(0.3)
                    .build(),
                PathRequestSegment.builder()
                    .waypoints(
                        // Through Depot
                        PathWaypoint.from(
                                new Pose2d(Depot.rightThrough, Rotation2d.fromDegrees(-105)))
                            .build())
                    .maxVelocity(1.5)
                    .maxAngularVelocity(0.1)
                    .keepInLaneWidth(0.05)
                    .build());

    PathRequestBuilder trenchLeftStartThroughDepot =
        PathRequest.builder()
            .segments(
                PathRequestSegment.builder()
                    .waypoints(
                        // Starting line
                        PathWaypoint.from(
                                new Pose2d(
                                    Trench.leftStart.minus(
                                        new Translation2d(DriveConstants.fullWidthX, 0.0)),
                                    Rotation2d.fromDegrees(-90)))
                            .build())
                    .build())
            .segments(depotLeftToRight.build().segments);

    PathRequestBuilder bumpLeftInnerThroughDepot =
        PathRequest.builder()
            .segments(
                PathRequestSegment.builder()
                    .waypoints(
                        // Starting line
                        PathWaypoint.from(new Pose2d(Bump.leftInner, Rotation2d.fromDegrees(-90)))
                            .build())
                    .build())
            .segments(depotLeftToRight.build().segments);

    paths.put("trenchLeftStartThroughDepot", trenchLeftStartThroughDepot.build());

    paths.put("bumpLeftInnerThroughDepot", bumpLeftInnerThroughDepot.build());

    // MARK: Substantial Salesman
    PathRequestBuilder depotRightToLeft =
        PathRequest.builder()
            .segments(
                PathRequestSegment.builder()
                    .waypoints(
                        // Edge of depot
                        PathWaypoint.from(
                                new Pose2d(Depot.rightThrough, Rotation2d.fromDegrees(105)))
                            .build())
                    .build(),
                PathRequestSegment.builder()
                    .waypoints(
                        // Through Depot
                        PathWaypoint.from(
                                new Pose2d(Depot.leftThrough, Rotation2d.fromDegrees(105)))
                            .build())
                    .maxVelocity(1.5)
                    .maxAngularVelocity(0.1)
                    .keepInLaneWidth(0.05)
                    .build());

    PathRequestBuilder launchLeftBumpThroughDepotLeftToRight =
        PathRequest.builder()
            .segments(
                PathRequestSegment.builder()
                    .waypoints(
                        // Starting launch pose
                        PathWaypoint.from(Launch.leftBump).build())
                    .build(),
                PathRequestSegment.builder()
                    .waypoints(
                        // Edge of depot offset translation
                        PathWaypoint.from(
                                new Translation2d(
                                    FieldConstants.Depot.rightCorner.getX()
                                        + DriveConstants.fullApothemX
                                        + 0.15,
                                    FieldConstants.Depot.rightCorner.getY()
                                        - DriveConstants.fullApothemX))
                            .build())
                    .build(),
                PathRequestSegment.builder()
                    .waypoints(
                        // Edge of depot
                        PathWaypoint.from(
                                new Pose2d(Depot.rightThrough, Rotation2d.fromDegrees(105)))
                            .build())
                    .build())
            // Intake through depot
            .segments(depotRightToLeft.build().segments)
            .segments(
                PathRequestSegment.builder()
                    .waypoints(
                        // Curve around end of depot
                        PathWaypoint.from(Depot.leftThrough.plus(new Translation2d(0.2, 0.25)))
                            .build())
                    .build());

    paths.put(
        "launchLeftBumpThroughDepotToLaunchLeftBump",
        PathRequest.builder()
            .segments(launchLeftBumpThroughDepotLeftToRight.build().segments)
            .segments(
                PathRequestSegment.builder()
                    .waypoints(
                        // Launch left bump translation
                        PathWaypoint.from(Launch.leftBump).build())
                    .build())
            .build());

    paths.put(
        "launchLeftBumpThroughDepotToLaunchLeftTrench",
        PathRequest.builder()
            .segments(launchLeftBumpThroughDepotLeftToRight.build().segments)
            .segments(
                PathRequestSegment.builder()
                    .waypoints(
                        // Launch left bump translation
                        PathWaypoint.from(Launch.leftTrench).build())
                    .keepInLaneWidth(0.1)
                    .build())
            .build());
  }
}
