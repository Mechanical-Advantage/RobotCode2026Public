// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2026;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import java.lang.reflect.Field;
import java.util.Optional;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class RobotStateTest {

  // Disable HAL before any class referencing FieldConstants is loaded so that
  // AprilTagLayoutType reads from the local src/main/deploy path rather than
  // the RoboRIO deploy directory (which doesn't exist in tests).
  static {
    Constants.disableHAL();
  }

  @BeforeAll
  static void setup() {
    Constants.disableHAL();
  }

  /** Reset the singleton before each test to avoid state bleed. */
  @BeforeEach
  void resetSingleton() throws Exception {
    Field instance = RobotState.class.getDeclaredField("instance");
    instance.setAccessible(true);
    instance.set(null, null);
  }

  private static SwerveModulePosition[] zeroPositions() {
    return new SwerveModulePosition[] {
      new SwerveModulePosition(0, Rotation2d.kZero),
      new SwerveModulePosition(0, Rotation2d.kZero),
      new SwerveModulePosition(0, Rotation2d.kZero),
      new SwerveModulePosition(0, Rotation2d.kZero)
    };
  }

  private static RobotState.OdometryObservation gyroObs(
      double t, SwerveModulePosition[] positions, double yawRad) {
    return new RobotState.OdometryObservation(
        t,
        positions,
        Optional.empty(),
        Optional.empty(),
        Optional.of(Rotation2d.fromRadians(yawRad)));
  }

  private static RobotState.OdometryObservation wheelObs(
      double t, SwerveModulePosition[] positions) {
    return new RobotState.OdometryObservation(
        t, positions, Optional.empty(), Optional.empty(), Optional.empty());
  }

  // ── resetPose ────────────────────────────────────────────────────────────

  @Test
  void resetPose_setsOdometryAndEstimatedPose() {
    RobotState rs = RobotState.getInstance();
    Pose2d target = new Pose2d(3.0, 1.5, Rotation2d.fromDegrees(90));
    rs.resetPose(target);
    assertEquals(target, rs.getOdometryPose());
    assertEquals(target, rs.getEstimatedPose());
  }

  @Test
  void resetPose_twice_retainsSecondPose() {
    RobotState rs = RobotState.getInstance();
    rs.resetPose(new Pose2d(1.0, 0, Rotation2d.kZero));
    Pose2d second = new Pose2d(5.0, 2.0, Rotation2d.fromDegrees(45));
    rs.resetPose(second);
    assertEquals(second, rs.getOdometryPose());
    assertEquals(second, rs.getEstimatedPose());
  }

  // ── addOdometryObservation ───────────────────────────────────────────────

  @Test
  void odometry_zeroMovement_doesNotChangePose() {
    RobotState rs = RobotState.getInstance();
    // Seed with one zero observation so lastWheelPositions is populated
    rs.addOdometryObservation(wheelObs(0.0, zeroPositions()));
    Pose2d before = rs.getOdometryPose();
    rs.addOdometryObservation(wheelObs(0.02, zeroPositions()));
    assertEquals(before.getX(), rs.getOdometryPose().getX(), 1e-9);
    assertEquals(before.getY(), rs.getOdometryPose().getY(), 1e-9);
  }

  @Test
  void odometry_forwardMovement_increasesX() {
    RobotState rs = RobotState.getInstance();
    rs.addOdometryObservation(wheelObs(0.0, zeroPositions()));

    // Move all 4 modules forward 1 metre
    SwerveModulePosition[] fwd =
        new SwerveModulePosition[] {
          new SwerveModulePosition(1.0, Rotation2d.kZero),
          new SwerveModulePosition(1.0, Rotation2d.kZero),
          new SwerveModulePosition(1.0, Rotation2d.kZero),
          new SwerveModulePosition(1.0, Rotation2d.kZero)
        };
    rs.addOdometryObservation(wheelObs(0.02, fwd));

    assertTrue(
        rs.getOdometryPose().getX() > 0.5,
        "Expected X to increase after forward movement, got " + rs.getOdometryPose().getX());
  }

  @Test
  void odometry_gyroOverridesHeading() {
    RobotState rs = RobotState.getInstance();
    rs.addOdometryObservation(wheelObs(0.0, zeroPositions()));
    rs.addOdometryObservation(gyroObs(0.02, zeroPositions(), Math.PI / 2));
    assertEquals(
        Math.PI / 2, rs.getOdometryPose().getRotation().getRadians(), 1e-6, "Gyro heading applied");
  }

  @Test
  void odometry_buildsPoseBuffer() {
    RobotState rs = RobotState.getInstance();
    rs.addOdometryObservation(wheelObs(1.0, zeroPositions()));
    rs.addOdometryObservation(wheelObs(1.02, zeroPositions()));
    // Should be able to retrieve a sample at the recorded timestamp
    Optional<Pose2d> sample = rs.getEstimatedPoseAtTimestamp(1.0);
    assertTrue(sample.isPresent(), "Pose buffer should have sample at t=1.0");
  }

  // ── addVisionObservation ─────────────────────────────────────────────────

  @Test
  void vision_nudgesEstimateTowardObservation() {
    RobotState rs = RobotState.getInstance();
    // Seed odometry buffer
    rs.addOdometryObservation(wheelObs(1.0, zeroPositions()));
    rs.addOdometryObservation(wheelObs(1.02, zeroPositions()));

    Pose2d before = rs.getEstimatedPose();

    // Send a vision measurement 2 m to the right of current estimate
    double targetX = before.getX() + 2.0;
    rs.addVisionObservation(
        new RobotState.VisionObservation(
            1.01,
            new Pose3d(targetX, before.getY(), 0, new Rotation3d()),
            VecBuilder.fill(0.1, 0.1, 0.1)));

    // Estimate should have moved toward vision (increased X)
    assertTrue(
        rs.getEstimatedPose().getX() > before.getX(),
        "Vision should nudge estimate toward observation");
  }

  @Test
  void vision_oldObservationOutsideBufferIsIgnored() {
    RobotState rs = RobotState.getInstance();
    // Only seed recent samples
    rs.addOdometryObservation(wheelObs(10.0, zeroPositions()));
    Pose2d before = rs.getEstimatedPose();

    // Vision timestamp is more than 2 s before last odometry sample → ignored
    rs.addVisionObservation(
        new RobotState.VisionObservation(
            7.9, // 10.0 - 2.0 - epsilon
            new Pose3d(99, 99, 0, new Rotation3d()),
            VecBuilder.fill(0.01, 0.01, 0.01)));

    assertEquals(
        before.getX(), rs.getEstimatedPose().getX(), 1e-9, "Stale vision should be ignored");
    assertEquals(
        before.getY(), rs.getEstimatedPose().getY(), 1e-9, "Stale vision should be ignored");
  }

  // ── getEstimatedPoseAtTimestamp ──────────────────────────────────────────

  @Test
  void poseAtTimestamp_returnsEmptyBeforeAnyOdometry() {
    RobotState rs = RobotState.getInstance();
    assertTrue(rs.getEstimatedPoseAtTimestamp(0.5).isEmpty());
  }

  @Test
  void poseAtTimestamp_returnsPresentForKnownTimestamp() {
    RobotState rs = RobotState.getInstance();
    rs.addOdometryObservation(wheelObs(2.0, zeroPositions()));
    rs.addOdometryObservation(wheelObs(2.02, zeroPositions()));
    assertTrue(rs.getEstimatedPoseAtTimestamp(2.0).isPresent());
  }

  // ── getIntakePose ────────────────────────────────────────────────────────

  @Test
  void intakePose_isAheadOfRobotCenter_facingForward() {
    RobotState rs = RobotState.getInstance();
    rs.resetPose(new Pose2d(0, 0, Rotation2d.kZero));
    // Intake should be offset in +X direction when heading is 0
    assertTrue(rs.getIntakePose().getX() > 0, "Intake should be ahead of robot centre");
  }
}
