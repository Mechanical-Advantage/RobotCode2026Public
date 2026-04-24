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
  public static final double substantialAimUntilXDepot =
      FieldConstants.Depot.rightCorner.getX() + DriveConstants.fullWidthX / 2.0 + 0.15;
  public static final double intakeVelocityLimit = 1.8;

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

    // MARK: Trench Start -> NZ
    PathRequestBuilder trenchLeftStartToNeutralZone =
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
                        PathWaypoint.from(Trench.leftClear).build())
                    .build());

    paths.put(
        "trenchLeftStartToNeutralZone",
        PathRequest.builder()
            .segments(trenchLeftStartToNeutralZone.build().segments)
            .stopAtEnd(false)
            .build());

    paths.put(
        "trenchLeftStartOffsetToNeutralZone",
        PathRequest.builder()
            .segments(trenchLeftStartOffsetToNeutralZone.build().segments)
            .stopAtEnd(false)
            .build());

    /* MARK: NZ Sweeps
     *
     *
     *
     *
     *
     * MARK: > Conservative
     */
    PathRequestBuilder neutralZoneSweepConservative =
        PathRequest.builder()
            .segments(
                PathRequestSegment.builder()
                    .waypoints(
                        // Right before fuel pool
                        PathWaypoint.from(
                                new Pose2d(
                                    FieldConstants.FuelPool.leftCenter.plus(
                                        new Translation2d(
                                            -DriveConstants.fullWidthX / 2.0,
                                            DriveConstants.fullWidthX / 2.0)),
                                    Rotation2d.fromDegrees(-90.0)))
                            .build())
                    .build(),
                PathRequestSegment.builder()
                    .waypoints(
                        // Into fuel pool
                        PathWaypoint.from(
                                new Pose2d(
                                    FieldConstants.fieldLength / 2.0
                                        - DriveConstants.fullWidthX / 2.0,
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
                                            - DriveConstants.fullWidthX / 2.0,
                                        FieldConstants.FuelPool.leftCenter.getY()
                                            - FieldConstants.FuelPool.width / 4.0
                                            + 0.2,
                                        Rotation2d.fromDegrees(-90))
                                    .plus(new Transform2d(0.3, 0.0, Rotation2d.fromDegrees(-20))))
                            .build())
                    .build(),
                PathRequestSegment.builder()
                    .waypoints(
                        // Turn fully around
                        PathWaypoint.from(
                                new Pose2d(
                                        FieldConstants.fieldLength / 2.0
                                            - DriveConstants.fullWidthX,
                                        FieldConstants.FuelPool.leftCenter.getY()
                                            - FieldConstants.FuelPool.width / 4.0
                                            - 0.2,
                                        Rotation2d.fromDegrees(-90))
                                    .plus(new Transform2d(0.3, 0.0, Rotation2d.fromDegrees(-90))))
                            .build())
                    .build());

    paths.put(
        "leftTrenchStartNeutralZoneSweepConservative",
        PathRequest.builder()
            .segments(trenchLeftStartToNeutralZone.build().segments)
            .segments(neutralZoneSweepConservative.build().segments)
            .stopAtEnd(false)
            .build());

    paths.put(
        "leftTrenchStartOffsetNeutralZoneSweepConservative",
        PathRequest.builder()
            .segments(trenchLeftStartOffsetToNeutralZone.build().segments)
            .segments(neutralZoneSweepConservative.build().segments)
            .stopAtEnd(false)
            .build());

    paths.put(
        "leftBumpNeutralZoneSweepConservative",
        PathRequest.builder()
            .segments(
                PathRequestSegment.builder()
                    .waypoints(
                        // Just after bump
                        PathWaypoint.from(
                                new Pose2d(
                                    Bump.leftOuter.plus(new Translation2d(0.4, 0)),
                                    Rotation2d.kZero))
                            .build())
                    .maxVelocity(3.2)
                    .build())
            .segments(neutralZoneSweepConservative.build().segments)
            .stopAtEnd(false)
            .build());

    // MARK: > Neutral
    PathRequestBuilder neutralZoneSweepNeutral =
        PathRequest.builder()
            .segments(
                PathRequestSegment.builder()
                    .waypoints(
                        // Robot width away from middle of fuel pool
                        PathWaypoint.from(
                                new Pose2d(
                                    FieldConstants.FuelPool.leftCenter.plus(
                                        new Translation2d(-0.2, DriveConstants.fullWidthX / 2.0)),
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
                    .build(),
                PathRequestSegment.builder()
                    .waypoints(
                        // Turn around fully
                        PathWaypoint.from(
                                new Pose2d(
                                    FieldConstants.fieldCenter.plus(
                                        new Translation2d(
                                            -0.7, FieldConstants.FuelPool.width / 4.0 - 0.27)),
                                    Rotation2d.kPi))
                            .build())
                    .maxVelocity(intakeVelocityLimit)
                    .build());

    paths.put(
        "leftTrenchStartNeutralZoneSweepNeutral",
        PathRequest.builder()
            .segments(trenchLeftStartToNeutralZone.build().segments)
            .segments(neutralZoneSweepNeutral.build().segments)
            .stopAtEnd(false)
            .build());

    paths.put(
        "leftTrenchStartOffsetNeutralZoneSweepNeutral",
        PathRequest.builder()
            .segments(trenchLeftStartOffsetToNeutralZone.build().segments)
            .segments(neutralZoneSweepNeutral.build().segments)
            .stopAtEnd(false)
            .build());

    paths.put(
        "leftBumpNeutralZoneSweepNeutral",
        PathRequest.builder()
            .segments(
                PathRequestSegment.builder()
                    .waypoints(
                        // Just after bump
                        PathWaypoint.from(
                                new Pose2d(
                                    Bump.leftOuter.plus(new Translation2d(0.4, 0)),
                                    Rotation2d.kZero))
                            .build())
                    .maxVelocity(3.2)
                    .build(),
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
            .segments(neutralZoneSweepNeutral.build().segments)
            .stopAtEnd(false)
            .build());

    // MARK: Alliance Zone
    PathRequestBuilder launchLeftBumpThroughLeftTrench =
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
        "launchLeftBumpThroughLeftTrench",
        PathRequest.builder()
            .segments(launchLeftBumpThroughLeftTrench.build().segments)
            .stopAtEnd(false)
            .build());

    PathRequestBuilder launchLeftBumpThroughLeftTrenchKachow =
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
        "launchLeftBumpThroughLeftTrenchKachow",
        PathRequest.builder()
            .segments(launchLeftBumpThroughLeftTrenchKachow.build().segments)
            .stopAtEnd(false)
            .build());

    paths.put(
        "launchLeftBumpThroughLeftTrenchToFarHubKachow",
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
                    .build(),
                PathRequestSegment.builder()
                    .waypoints(
                        PathWaypoint.from(
                                new Pose2d(
                                    Trench.leftClear.plus(new Translation2d(0.0, -0.1)),
                                    Rotation2d.fromDegrees(-27)))
                            .build(),
                        PathWaypoint.from(
                                new Pose2d(
                                    FieldConstants.LinesVertical.neutralZoneFar,
                                    FieldConstants.fieldWidth / 2.0,
                                    Rotation2d.fromDegrees(-40)))
                            .build())
                    .build())
            .stopAtEnd(false)
            .build());

    paths.put(
        "launchLeftBumpThroughLeftTrenchToFarBumpKachow",
        PathRequest.builder()
            .segments(
                PathRequestSegment.builder()
                    .waypoints(
                        // Continue after launching
                        PathWaypoint.from(Launch.leftBump).build(),

                        // Prepare to enter trench
                        PathWaypoint.from(new Pose2d(Trench.leftEntry, Rotation2d.fromDegrees(-90)))
                            .build())
                    .build(),
                PathRequestSegment.builder()
                    .waypoints(
                        // Through trench
                        PathWaypoint.from(new Pose2d(Trench.leftClear, Rotation2d.fromDegrees(-90)))
                            .build())
                    .keepInLaneWidth(0.05)
                    .build(),
                PathRequestSegment.builder()
                    .waypoints(
                        // Far bump
                        PathWaypoint.from(
                                new Pose2d(
                                    FieldConstants.LinesVertical.neutralZoneFar,
                                    FieldConstants.LinesHorizontal.leftBumpMiddle,
                                    Rotation2d.fromDegrees(-90)))
                            .build())
                    .build())
            .stopAtEnd(false)
            .build());

    paths.put(
        "launchLeftBumpThroughLeftTrenchToFarTrenchKachow",
        PathRequest.builder()
            .segments(
                PathRequestSegment.builder()
                    .waypoints(
                        // Continue after launching
                        PathWaypoint.from(Launch.leftBump).build(),

                        // Prepare to enter trench
                        PathWaypoint.from(new Pose2d(Trench.leftEntry, Rotation2d.fromDegrees(-90)))
                            .build())
                    .build(),
                PathRequestSegment.builder()
                    .waypoints(
                        // Orient rotation
                        PathWaypoint.from(
                                new Pose2d(
                                    FieldConstants.fieldLength / 2.0
                                        - DriveConstants.fullWidthX / 2.0,
                                    Trench.leftEntry.getY(),
                                    Rotation2d.fromDegrees(-90)))
                            .build())
                    .keepInLaneWidth(0.05)
                    .build(),
                PathRequestSegment.builder()
                    .waypoints(
                        // Far trench
                        PathWaypoint.from(
                                new Pose2d(
                                    FieldConstants.LinesVertical.neutralZoneFar,
                                    FieldConstants.LinesHorizontal.leftTrenchMiddle,
                                    Rotation2d.fromDegrees(-90)))
                            .build())
                    .build())
            .stopAtEnd(false)
            .build());

    PathRequestBuilder leftTrenchThroughCoriolis =
        PathRequest.builder()
            .segments(
                PathRequestSegment.builder()
                    // Turn into fuel pool
                    .waypoints(
                        PathWaypoint.from(
                                new Pose2d(
                                    FieldConstants.FuelPool.nearLeftCorner.plus(
                                        new Translation2d(
                                            -0.2, DriveConstants.fullWidthX / 2.0 + 0.1)),
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
                                                -DriveConstants.fullWidthX / 2.0,
                                                DriveConstants.fullWidthX / 2.0))
                                        .plus(new Translation2d(-0.1, 0.4)),
                                    Rotation2d.fromDegrees(-90)))
                            .build())
                    .keepInLaneWidth(0.05)
                    .maxAngularVelocity(0.2)
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
                        // Drive behind hub
                        PathWaypoint.from(
                                new Pose2d(
                                    FieldConstants.LinesVertical.neutralZoneNear
                                        + DriveConstants.fullWidthX / 2.0
                                        + 0.75,
                                    FieldConstants.fieldWidth / 2.0 + 0.4,
                                    Rotation2d.kPi))
                            .build())
                    .maxAngularVelocity(0.1)
                    .build(),
                PathRequestSegment.builder()
                    .waypoints(
                        // Drive in front of bump
                        PathWaypoint.from(
                                new Pose2d(
                                    FieldConstants.LeftBump.farRightCorner.plus(
                                        new Translation2d(
                                            DriveConstants.fullWidthX / 2.0 + 0.2,
                                            DriveConstants.fullWidthX / 2.0)),
                                    Rotation2d.fromDegrees(90)))
                            .build())
                    .maxAngularVelocity(2.5)
                    .build());

    paths.put(
        "launchLeftBumpThroughLeftTrenchToCoriolis",
        PathRequest.builder()
            .segments(launchLeftBumpThroughLeftTrench.build().segments)
            .segments(leftTrenchThroughCoriolis.build().segments)
            .stopAtEnd(false)
            .build());

    paths.put(
        "launchLeftBumpThroughLeftTrenchToCoriolisKachow",
        PathRequest.builder()
            .segments(launchLeftBumpThroughLeftTrenchKachow.build().segments)
            .segments(leftTrenchThroughCoriolis.build().segments)
            .stopAtEnd(false)
            .build());

    PathRequestBuilder launchLeftBumpThroughTrenchToBehindHub =
        PathRequest.builder()
            .segments(launchLeftBumpThroughLeftTrench.build().segments)
            .segments(
                PathRequestSegment.builder()
                    .waypoints(
                        // Fix rotation before robot has crossed bump
                        PathWaypoint.from(
                                new Pose2d(
                                    FieldConstants.LeftBump.farLeftCorner.plus(
                                        new Translation2d(
                                            DriveConstants.fullWidthX / 2.0 + 0.5, 0)),
                                    Rotation2d.fromDegrees(-90)))
                            .build())
                    .build(),
                PathRequestSegment.builder()
                    .waypoints(
                        // Behind the hub
                        PathWaypoint.from(
                                new Pose2d(
                                    FieldConstants.LeftBump.farRightCorner.plus(
                                        new Translation2d(
                                            DriveConstants.fullWidthX / 2.0 + 0.5,
                                            -DriveConstants.fullWidthX / 2.0 + 0.2)),
                                    Rotation2d.fromDegrees(-90)))
                            .build())
                    .keepInLaneWidth(0.08)
                    .maxAngularVelocity(0.1)
                    .build());

    PathRequestBuilder launchLeftBumpThroughTrenchToBehindHubKachow =
        PathRequest.builder()
            .segments(launchLeftBumpThroughLeftTrenchKachow.build().segments)
            .segments(
                PathRequestSegment.builder()
                    .waypoints(
                        // Fix rotation before robot has crossed bump
                        PathWaypoint.from(
                                new Pose2d(
                                    FieldConstants.LeftBump.farLeftCorner.plus(
                                        new Translation2d(
                                            DriveConstants.fullWidthX / 2.0 + 0.5, 0)),
                                    Rotation2d.fromDegrees(-90)))
                            .build())
                    .build(),
                PathRequestSegment.builder()
                    .waypoints(
                        // Behind the hub
                        PathWaypoint.from(
                                new Pose2d(
                                    FieldConstants.LeftBump.farRightCorner.plus(
                                        new Translation2d(
                                            DriveConstants.fullWidthX / 2.0 + 0.5,
                                            -DriveConstants.fullWidthX / 2.0 + 0.2)),
                                    Rotation2d.fromDegrees(-90)))
                            .build())
                    .keepInLaneWidth(0.08)
                    .maxAngularVelocity(0.1)
                    .build());

    PathRequestBuilder launchLeftBumpThroughTrenchToBehindHubFriendship =
        PathRequest.builder()
            .segments(launchLeftBumpThroughLeftTrench.build().segments)
            .segments(
                PathRequestSegment.builder()
                    .waypoints(
                        // Fix rotation before robot has crossed bump
                        PathWaypoint.from(
                                new Pose2d(
                                    FieldConstants.LeftBump.farLeftCorner.plus(
                                        new Translation2d(
                                            DriveConstants.fullWidthX / 2.0 + 0.5, 0)),
                                    Rotation2d.fromDegrees(-90)))
                            .build())
                    .build(),
                PathRequestSegment.builder()
                    .waypoints(
                        // Behind the hub
                        PathWaypoint.from(
                                new Pose2d(
                                    FieldConstants.LeftBump.farRightCorner.plus(
                                        new Translation2d(
                                            DriveConstants.fullWidthX / 2.0 + 0.5,
                                            DriveConstants.fullWidthX / 2.0 + 0.2)),
                                    Rotation2d.fromDegrees(-90)))
                            .build())
                    .keepInLaneWidth(0.08)
                    .maxAngularVelocity(0.1)
                    .build());

    PathRequestBuilder launchLeftBumpThroughTrenchToBehindHubFriendshipKachow =
        PathRequest.builder()
            .segments(launchLeftBumpThroughLeftTrenchKachow.build().segments)
            .segments(
                PathRequestSegment.builder()
                    .waypoints(
                        // Fix rotation before robot has crossed bump
                        PathWaypoint.from(
                                new Pose2d(
                                    FieldConstants.LeftBump.farLeftCorner.plus(
                                        new Translation2d(
                                            DriveConstants.fullWidthX / 2.0 + 0.5, 0)),
                                    Rotation2d.fromDegrees(-90)))
                            .build())
                    .build(),
                PathRequestSegment.builder()
                    .waypoints(
                        // Behind the hub
                        PathWaypoint.from(
                                new Pose2d(
                                    FieldConstants.LeftBump.farRightCorner.plus(
                                        new Translation2d(
                                            DriveConstants.fullWidthX / 2.0 + 0.5,
                                            DriveConstants.fullWidthX / 2.0 + 0.2)),
                                    Rotation2d.fromDegrees(-90)))
                            .build())
                    .keepInLaneWidth(0.08)
                    .maxAngularVelocity(0.1)
                    .build());

    PathRequestBuilder bumpLeftOuterToBehindHub =
        PathRequest.builder()
            .segments(
                PathRequestSegment.builder()
                    .waypoints(
                        // Just after bump
                        PathWaypoint.from(
                                new Pose2d(
                                    Bump.leftOuter.plus(new Translation2d(0.4, 0)),
                                    Rotation2d.kZero))
                            .build())
                    .maxVelocity(3.2)
                    .build(),
                PathRequestSegment.builder()
                    .waypoints(
                        // Fix rotation right before behind hub sweep
                        PathWaypoint.from(
                                new Pose2d(
                                    FieldConstants.LeftBump.farRightCorner.plus(
                                        new Translation2d(
                                            DriveConstants.fullWidthX / 2.0 + 0.3,
                                            DriveConstants.fullWidthX / 2.0)),
                                    Rotation2d.fromDegrees(-90)))
                            .build())
                    .build(),
                PathRequestSegment.builder()
                    .waypoints(
                        // Behind the hub
                        PathWaypoint.from(
                                new Pose2d(
                                    FieldConstants.LeftBump.farRightCorner.plus(
                                        new Translation2d(
                                            DriveConstants.fullWidthX / 2.0 + 0.3,
                                            -DriveConstants.fullWidthX / 2.0)),
                                    Rotation2d.fromDegrees(-90)))
                            .build())
                    .keepInLaneWidth(0.08)
                    .maxAngularVelocity(0.1)
                    .build());

    PathRequestBuilder bumpLeftOuterToBehindHubFriendship =
        PathRequest.builder()
            .segments(
                PathRequestSegment.builder()
                    .waypoints(
                        // Just after bump
                        PathWaypoint.from(
                                new Pose2d(
                                    Bump.leftOuter.plus(new Translation2d(0.4, 0)),
                                    Rotation2d.kZero))
                            .build())
                    .maxVelocity(3.2)
                    .build(),
                PathRequestSegment.builder()
                    .waypoints(
                        // Point behind hub
                        PathWaypoint.from(
                                new Pose2d(
                                    FieldConstants.LeftBump.farRightCorner.plus(
                                        new Translation2d(
                                            DriveConstants.fullWidthX / 2.0 + 0.3,
                                            DriveConstants.fullWidthX / 2.0)),
                                    Rotation2d.fromDegrees(-90)))
                            .build())
                    .build());

    paths.put(
        "launchLeftBumpThroughTrenchToBehindHub",
        PathRequest.builder()
            .segments(launchLeftBumpThroughTrenchToBehindHub.build().segments)
            .stopAtEnd(false)
            .build());

    paths.put(
        "launchLeftBumpThroughTrenchToBehindHubKachow",
        PathRequest.builder()
            .segments(launchLeftBumpThroughTrenchToBehindHubKachow.build().segments)
            .stopAtEnd(false)
            .build());

    paths.put(
        "launchLeftBumpThroughTrenchToBehindHubFriendship",
        PathRequest.builder()
            .segments(launchLeftBumpThroughTrenchToBehindHubFriendship.build().segments)
            .stopAtEnd(false)
            .build());

    paths.put(
        "launchLeftBumpThroughTrenchToBehindHubFriendshipKachow",
        PathRequest.builder()
            .segments(launchLeftBumpThroughTrenchToBehindHubFriendshipKachow.build().segments)
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
                                            -DriveConstants.fullWidthX / 2.0 - 0.7,
                                            DriveConstants.fullWidthX / 2.0 - 0.15)),
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
        "launchLeftBumpThroughTrenchToDavis",
        PathRequest.builder()
            .segments(launchLeftBumpThroughTrenchToBehindHub.build().segments)
            .segments(behindHubThroughDavis.build().segments)
            .stopAtEnd(false)
            .build());

    paths.put(
        "launchLeftBumpThroughTrenchToDavisKachow",
        PathRequest.builder()
            .segments(launchLeftBumpThroughTrenchToBehindHubKachow.build().segments)
            .segments(behindHubThroughDavis.build().segments)
            .stopAtEnd(false)
            .build());

    paths.put(
        "launchLeftBumpThroughTrenchToDavisFriendship",
        PathRequest.builder()
            .segments(launchLeftBumpThroughTrenchToBehindHubFriendship.build().segments)
            .segments(behindHubThroughDavisFriendship.build().segments)
            .stopAtEnd(false)
            .build());

    paths.put(
        "launchLeftBumpThroughTrenchToDavisFriendshipKachow",
        PathRequest.builder()
            .segments(launchLeftBumpThroughTrenchToBehindHubFriendshipKachow.build().segments)
            .segments(behindHubThroughDavisFriendship.build().segments)
            .stopAtEnd(false)
            .build());

    paths.put(
        "bumpLeftOuterToDavis",
        PathRequest.builder()
            .segments(bumpLeftOuterToBehindHub.build().segments)
            .segments(behindHubThroughDavis.build().segments)
            .stopAtEnd(false)
            .build());

    paths.put(
        "bumpLeftOuterToDavisFriendship",
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

    paths.put(
        "launchLeftBumpThroughDepotAndBack",
        PathRequest.builder()
            .segments(
                PathRequestSegment.builder()
                    .waypoints(
                        // Starting launch pose
                        PathWaypoint.from(Launch.leftBump).build())
                    .build(),
                PathRequestSegment.builder()
                    .waypoints(
                        // Starting launch translation
                        PathWaypoint.from(Launch.leftBump.getTranslation()).build())
                    .build(),
                PathRequestSegment.builder()
                    .waypoints(
                        // Edge of depot offset translation
                        PathWaypoint.from(
                                new Translation2d(
                                    substantialAimUntilXDepot,
                                    FieldConstants.Depot.rightCorner.getY()
                                        - DriveConstants.fullWidthX / 2.0))
                            .build())
                    .pointAt(hubTarget)
                    .maxVelocity(0.6)
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
                        // Through Depot translation
                        PathWaypoint.from(Depot.leftThrough).build())
                    .build(),
                PathRequestSegment.builder()
                    .waypoints(
                        // Curve around end of depot
                        PathWaypoint.from(Depot.leftThrough.plus(new Translation2d(0.2, 0.25)))
                            .build())
                    .build(),
                PathRequestSegment.builder()
                    .waypoints(
                        // Launch left bump translation
                        PathWaypoint.from(Launch.leftBump.getTranslation()).build())
                    .pointAt(hubTarget)
                    .maxVelocity(0.8)
                    .build())
            .stopAtEnd(false)
            .build());
  }
}
