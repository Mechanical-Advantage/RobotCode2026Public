// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2026.subsystems.launcher;

import static org.littletonrobotics.frc2026.subsystems.launcher.LauncherConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import lombok.Getter;
import lombok.experimental.ExtensionMethod;
import org.littletonrobotics.frc2026.Constants;
import org.littletonrobotics.frc2026.FieldConstants;
import org.littletonrobotics.frc2026.RobotState;
import org.littletonrobotics.frc2026.subsystems.launcher.hood.Hood;
import org.littletonrobotics.frc2026.util.LoggedTunableNumber;
import org.littletonrobotics.frc2026.util.geometry.AllianceFlipUtil;
import org.littletonrobotics.frc2026.util.geometry.Bounds;
import org.littletonrobotics.frc2026.util.geometry.GeomUtil;
import org.littletonrobotics.junction.Logger;

@ExtensionMethod({GeomUtil.class})
public class LaunchCalculator {
  private static LaunchCalculator instance;

  @Getter private double flywheelSpeedOffsetRadsPerSec = 0.0;

  private final LinearFilter hoodAngleFilter =
      LinearFilter.movingAverage((int) (0.1 / Constants.loopPeriodSecs));
  private final LinearFilter driveAngleFilter =
      LinearFilter.movingAverage((int) (0.1 / Constants.loopPeriodSecs));

  private double lastHoodAngle;
  private Rotation2d lastDriveAngle;

  public static LaunchCalculator getInstance() {
    if (instance == null) instance = new LaunchCalculator();
    return instance;
  }

  public record LaunchingParameters(
      boolean isValid,
      Rotation2d driveAngle,
      double driveVelocity,
      double hoodAngle,
      double hoodVelocity,
      double flywheelSpeed,
      double kickerSurfaceSpeed,
      double distance,
      double distanceNoLookahead,
      double timeOfFlight,
      boolean passing) {}

  // Cache parameters
  private LaunchingParameters latestParameters = null;

  private static final double minDistance;
  private static final double maxDistance;
  private static final double passingMinDistance;
  private static final double passingMaxDistance;
  private static final double phaseDelay;

  // Launching Maps
  private static final InterpolatingTreeMap<Double, Rotation2d> hoodAngleMap =
      new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Rotation2d::interpolate);
  private static final InterpolatingDoubleTreeMap flywheelSpeedMap =
      new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap timeOfFlightMap =
      new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap kickerSurfaceSpeedMap =
      new InterpolatingDoubleTreeMap();

  // Passing Maps
  private static final InterpolatingTreeMap<Double, Rotation2d> passingHoodAngleMap =
      new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Rotation2d::interpolate);
  private static final InterpolatingDoubleTreeMap passingFlywheelSpeedMap =
      new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap passingTimeOfFlightMap =
      new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap passingKickerSurfaceSpeedMap =
      new InterpolatingDoubleTreeMap();

  // Presets
  public static final double towerPresetDistance = 2.5;
  public static final double trenchPresetDistance = 3.03;
  public static final double outpostPresetDistance = 4.84;
  public static final double passingPresetDistance = 5.5;
  public static final LaunchPreset passingPreset;
  public static final LaunchPreset towerPreset;
  public static final LaunchPreset trenchPreset;
  public static final LaunchPreset outpostPreset;
  public static final LaunchPreset hoodMinPreset =
      new LaunchPreset(
          new LoggedTunableNumber(
              "LaunchCalculator/Presets/HoodMin/HoodAngle", Units.radiansToDegrees(Hood.minAngle)),
          new LoggedTunableNumber("LaunchCalculator/Presets/HoodMin/FlywheelSpeed", 50));
  public static final LaunchPreset hoodMaxPreset =
      new LaunchPreset(
          new LoggedTunableNumber(
              "LaunchCalculator/Presets/HoodMax/HoodAngle", Units.radiansToDegrees(Hood.maxAngle)),
          new LoggedTunableNumber("LaunchCalculator/Presets/HoodMax/FlywheelSpeed", 50));

  public static final LoggedTunableNumber idleSpeed =
      new LoggedTunableNumber("LaunchCalculator/IdleSpeed", 80);
  public static final LoggedTunableNumber linearDragTimeConstant =
      new LoggedTunableNumber("LaunchCalculator/LinearDragTimeConstant", 0.375);

  public static record LaunchPreset(
      LoggedTunableNumber hoodAngleDeg, LoggedTunableNumber flywheelSpeed) {}

  // Passing targets
  private static final double xPassTarget = Units.inchesToMeters(37);
  private static final double yPassTarget = Units.inchesToMeters(65);
  // Boxes of bad
  // Under tower
  private static final Bounds towerBound =
      new Bounds(0, Units.inchesToMeters(46), Units.inchesToMeters(129), Units.inchesToMeters(168));

  // Behind the hubs
  public static final Bounds nearHubBound =
      new Bounds(
          FieldConstants.LinesVertical.neutralZoneNear,
          FieldConstants.LinesVertical.neutralZoneNear + Units.inchesToMeters(120),
          FieldConstants.LinesHorizontal.rightBumpStart,
          FieldConstants.LinesHorizontal.leftBumpEnd);
  private static final Bounds farHubBound =
      new Bounds(
          FieldConstants.LinesVertical.oppAllianceZone,
          FieldConstants.fieldLength,
          FieldConstants.LinesHorizontal.rightBumpStart,
          FieldConstants.LinesHorizontal.leftBumpEnd);

  static {
    minDistance = 0.6;
    maxDistance = 5.3;
    passingMinDistance = 3.0;
    passingMaxDistance = 17.16;
    phaseDelay = 0.03;

    hoodAngleMap.put(0.75, Rotation2d.fromDegrees(10.0));
    hoodAngleMap.put(1.58, Rotation2d.fromDegrees(20.0));
    hoodAngleMap.put(1.98, Rotation2d.fromDegrees(23.0));
    hoodAngleMap.put(2.53, Rotation2d.fromDegrees(26.0));
    hoodAngleMap.put(3.23, Rotation2d.fromDegrees(29.0));
    hoodAngleMap.put(3.67, Rotation2d.fromDegrees(31.0));
    hoodAngleMap.put(3.95, Rotation2d.fromDegrees(33.0));
    hoodAngleMap.put(4.65, Rotation2d.fromDegrees(34.0));
    hoodAngleMap.put(5.00, Rotation2d.fromDegrees(37.0));

    flywheelSpeedMap.put(0.75, 145.0);
    flywheelSpeedMap.put(1.58, 145.0);
    flywheelSpeedMap.put(1.98, 155.0);
    flywheelSpeedMap.put(2.53, 160.0);
    flywheelSpeedMap.put(3.23, 175.0);
    flywheelSpeedMap.put(3.67, 180.0);
    flywheelSpeedMap.put(3.95, 190.0);
    flywheelSpeedMap.put(4.65, 195.0);
    flywheelSpeedMap.put(5.00, 200.0);

    flywheelSpeedMap.put(1.15, 170.0);
    flywheelSpeedMap.put(1.41, 160.0);
    flywheelSpeedMap.put(1.81, 165.0);
    flywheelSpeedMap.put(2.32, 165.0);
    flywheelSpeedMap.put(2.71, 175.0);
    flywheelSpeedMap.put(3.38, 175.0);
    flywheelSpeedMap.put(4.04, 185.0);
    flywheelSpeedMap.put(4.83, 195.0);

    kickerSurfaceSpeedMap.put(0.0, 6.0);
    kickerSurfaceSpeedMap.put(3.0, 6.0);
    kickerSurfaceSpeedMap.put(4.0, 3.0);
    kickerSurfaceSpeedMap.put(5.0, 3.0);

    timeOfFlightMap.put(1.63, 1.017);
    timeOfFlightMap.put(2.40, 0.967);
    timeOfFlightMap.put(3.25, 1.19);
    timeOfFlightMap.put(4.15, 1.18);
    timeOfFlightMap.put(4.875, 1.25);

    passingHoodAngleMap.put(5.46, Rotation2d.fromDegrees(38.0));
    passingHoodAngleMap.put(6.62, Rotation2d.fromDegrees(38.0));
    passingHoodAngleMap.put(7.80, Rotation2d.fromDegrees(38.0));
    passingHoodAngleMap.put(17.16, Rotation2d.fromDegrees(38.0));

    passingFlywheelSpeedMap.put(5.46, 160.0);
    passingFlywheelSpeedMap.put(6.62, 180.0);
    passingFlywheelSpeedMap.put(7.80, 200.0);
    passingFlywheelSpeedMap.put(17.16, 400.0);

    passingKickerSurfaceSpeedMap.put(0.0, 6.0);

    passingTimeOfFlightMap.put(5.46, 1.27);
    passingTimeOfFlightMap.put(6.62, 1.39);
    passingTimeOfFlightMap.put(7.8, 1.49);
    passingTimeOfFlightMap.put(11.0, 1.75);
    passingTimeOfFlightMap.put(13.0, 1.76);
    passingTimeOfFlightMap.put(17.16, 2.5);

    passingPreset =
        new LaunchPreset(
            new LoggedTunableNumber(
                "LaunchCalculator/Presets/Passing/HoodAngle",
                hoodAngleMap.get(passingPresetDistance).getDegrees()),
            new LoggedTunableNumber(
                "LaunchCalculator/Presets/Passing/FlywheelSpeed",
                flywheelSpeedMap.get(passingPresetDistance)));
    towerPreset =
        new LaunchPreset(
            new LoggedTunableNumber(
                "LaunchCalculator/Presets/Tower/HoodAngle",
                hoodAngleMap.get(towerPresetDistance).getDegrees()),
            new LoggedTunableNumber(
                "LaunchCalculator/Presets/Tower/FlywheelSpeed",
                flywheelSpeedMap.get(towerPresetDistance)));
    trenchPreset =
        new LaunchPreset(
            new LoggedTunableNumber(
                "LaunchCalculator/Presets/Trench/HoodAngle",
                hoodAngleMap.get(trenchPresetDistance).getDegrees()),
            new LoggedTunableNumber(
                "LaunchCalculator/Presets/Trench/FlywheelSpeed",
                flywheelSpeedMap.get(trenchPresetDistance)));
    outpostPreset =
        new LaunchPreset(
            new LoggedTunableNumber(
                "LaunchCalculator/Presets/Outpost/HoodAngle",
                hoodAngleMap.get(outpostPresetDistance).getDegrees()),
            new LoggedTunableNumber(
                "LaunchCalculator/Presets/Outpost/FlywheelSpeed",
                flywheelSpeedMap.get(outpostPresetDistance)));
  }

  public static double getMinTimeOfFlight() {
    return timeOfFlightMap.get(minDistance);
  }

  public static double getMaxTimeOfFlight() {
    return timeOfFlightMap.get(maxDistance);
  }

  public LaunchingParameters getParameters() {
    boolean passing =
        AllianceFlipUtil.applyX(RobotState.getInstance().getEstimatedPose().getX())
            > FieldConstants.LinesVertical.hubCenter;
    if (latestParameters != null) {
      return latestParameters;
    }

    // Calculate estimated pose while accounting for phase delay
    Pose2d estimatedPose = RobotState.getInstance().getEstimatedPose();
    ChassisSpeeds robotRelativeVelocity = RobotState.getInstance().getRobotVelocity();
    estimatedPose =
        estimatedPose.exp(
            new Twist2d(
                robotRelativeVelocity.vxMetersPerSecond * phaseDelay,
                robotRelativeVelocity.vyMetersPerSecond * phaseDelay,
                robotRelativeVelocity.omegaRadiansPerSecond * phaseDelay));

    // Calculate target
    Translation2d target =
        passing
            ? AllianceFlipUtil.apply(getPassingTarget())
            : AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint.toTranslation2d());
    Pose2d launcherPosition = estimatedPose.transformBy(robotToLauncher.toTransform2d());
    double launcherToTargetDistance = target.getDistance(launcherPosition.getTranslation());

    // Calculate field relative launcher velocity
    var robotVelocity = RobotState.getInstance().getFieldSetpointVelocity();
    var robotAngle = RobotState.getInstance().getRotation();
    ChassisSpeeds launcherVelocity =
        DriverStation.isAutonomous()
            ? robotVelocity
            : GeomUtil.transformVelocity(
                robotVelocity, robotToLauncher.getTranslation().toTranslation2d(), robotAngle);

    // Account for imparted velocity by robot (launcher) to offset
    double timeOfFlight =
        passing
            ? passingTimeOfFlightMap.get(launcherToTargetDistance)
            : timeOfFlightMap.get(launcherToTargetDistance);
    Pose2d lookaheadPose = launcherPosition;
    double lookaheadLauncherToTargetDistance = launcherToTargetDistance;

    for (int i = 0; i < 20; i++) {
      timeOfFlight =
          passing
              ? passingTimeOfFlightMap.get(lookaheadLauncherToTargetDistance)
              : timeOfFlightMap.get(lookaheadLauncherToTargetDistance);
      double effectiveTOF =
          (1 - Math.exp(-timeOfFlight * linearDragTimeConstant.get()))
              / linearDragTimeConstant.get();
      double offsetX = launcherVelocity.vxMetersPerSecond * effectiveTOF;
      double offsetY = launcherVelocity.vyMetersPerSecond * effectiveTOF;
      lookaheadPose =
          new Pose2d(
              launcherPosition.getTranslation().plus(new Translation2d(offsetX, offsetY)),
              launcherPosition.getRotation());
      lookaheadLauncherToTargetDistance = target.getDistance(lookaheadPose.getTranslation());
    }

    // Account for launcher being off center
    Pose2d lookaheadRobotPose =
        lookaheadPose.transformBy(robotToLauncher.toTransform2d().inverse());
    Rotation2d driveAngle = getDriveAngleWithLauncherOffset(lookaheadRobotPose, target);

    // Calculate remaining parameters
    double hoodAngle =
        passing
            ? passingHoodAngleMap.get(lookaheadLauncherToTargetDistance).getRadians()
            : hoodAngleMap.get(lookaheadLauncherToTargetDistance).getRadians();
    if (lastDriveAngle == null) lastDriveAngle = driveAngle;
    if (Double.isNaN(lastHoodAngle)) lastHoodAngle = hoodAngle;
    double hoodVelocity =
        hoodAngleFilter.calculate((hoodAngle - lastHoodAngle) / Constants.loopPeriodSecs);
    lastHoodAngle = hoodAngle;
    double driveVelocity =
        driveAngleFilter.calculate(
            driveAngle.minus(lastDriveAngle).getRadians() / Constants.loopPeriodSecs);
    lastDriveAngle = driveAngle;

    // Check if inside a box of bad
    var flippedPose = AllianceFlipUtil.apply(estimatedPose);
    boolean insideTowerBadBox = towerBound.contains(flippedPose.getTranslation());
    boolean behindNearHub = nearHubBound.contains(flippedPose.getTranslation());
    boolean behindFarHub = farHubBound.contains(flippedPose.getTranslation());
    boolean outsideOfBadBoxes = !(insideTowerBadBox || behindNearHub || behindFarHub);

    double flywheelVelocity =
        passing
            ? passingFlywheelSpeedMap.get(lookaheadLauncherToTargetDistance)
            : flywheelSpeedMap.get(lookaheadLauncherToTargetDistance);

    double kickerSurfaceSpeed =
        passing
            ? passingKickerSurfaceSpeedMap.get(lookaheadLauncherToTargetDistance)
            : kickerSurfaceSpeedMap.get(lookaheadLauncherToTargetDistance);

    // Constructor parameters
    latestParameters =
        new LaunchingParameters(
            outsideOfBadBoxes
                && lookaheadLauncherToTargetDistance >= (passing ? passingMinDistance : minDistance)
                && lookaheadLauncherToTargetDistance
                    <= (passing ? passingMaxDistance : maxDistance),
            driveAngle,
            driveVelocity,
            hoodAngle,
            hoodVelocity,
            flywheelVelocity + flywheelSpeedOffsetRadsPerSec,
            kickerSurfaceSpeed,
            lookaheadLauncherToTargetDistance,
            launcherToTargetDistance,
            timeOfFlight,
            passing);

    // Log calculated values
    Logger.recordOutput("LaunchCalculator/TargetPose", new Pose2d(target, Rotation2d.kZero));
    Logger.recordOutput("LaunchCalculator/LookaheadPose", lookaheadRobotPose);
    Logger.recordOutput(
        "LaunchCalculator/LauncherToTargetDistance", lookaheadLauncherToTargetDistance);

    return latestParameters;
  }

  private static Rotation2d getDriveAngleWithLauncherOffset(
      Pose2d robotPose, Translation2d target) {
    Rotation2d fieldToHubAngle = target.minus(robotPose.getTranslation()).getAngle();
    Rotation2d hubAngle =
        new Rotation2d(
            Math.asin(
                MathUtil.clamp(
                    robotToLauncher.getTranslation().getY()
                        / target.getDistance(robotPose.getTranslation()),
                    -1.0,
                    1.0)));
    Rotation2d driveAngle =
        fieldToHubAngle.plus(hubAngle).plus(robotToLauncher.getRotation().toRotation2d());
    return driveAngle;
  }

  public double getNaiveTOF(double distance) {
    return timeOfFlightMap.get(distance);
  }

  public void clearLaunchingParameters() {
    latestParameters = null;
  }

  public static Translation2d getPassingTarget() {
    double flippedY = AllianceFlipUtil.apply(RobotState.getInstance().getEstimatedPose()).getY();
    boolean mirror = flippedY > FieldConstants.LinesHorizontal.center;

    // Fixed passing target
    Translation2d flippedGoalTranslation =
        new Translation2d(
            xPassTarget, mirror ? FieldConstants.fieldWidth - yPassTarget : yPassTarget);

    return flippedGoalTranslation;
  }

  /**
   * Returns the Pose2d that correctly aims the robot at the goal for a given robot translation.
   *
   * @param robotTranslation The translation of the center of the robot.
   * @param forceBlue Always use the blue hub target
   * @return The target pose for the aimed robot.
   */
  public static Pose2d getStationaryAimedPose(Translation2d robotTranslation, boolean forceBlue) {
    // Calculate target
    boolean passing =
        !Constants.disableHAL && LaunchCalculator.getInstance().getParameters().passing();

    Translation2d target =
        passing ? getPassingTarget() : FieldConstants.Hub.topCenterPoint.toTranslation2d();
    if (!forceBlue) {
      target = AllianceFlipUtil.apply(target);
    }

    return new Pose2d(
        robotTranslation, getDriveAngleWithLauncherOffset(robotTranslation.toPose2d(), target));
  }

  /** Adjusts the flywheel speed up or down the specified amount. */
  public void incrementFlywheelSpeedOffset(double incrementRadsPerSec) {
    flywheelSpeedOffsetRadsPerSec += incrementRadsPerSec;
  }
}
