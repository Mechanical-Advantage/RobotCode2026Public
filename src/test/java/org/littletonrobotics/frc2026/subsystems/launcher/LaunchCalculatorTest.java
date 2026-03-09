// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2026.subsystems.launcher;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import org.littletonrobotics.frc2026.Constants;

/**
 * Tests the LaunchCalculator static lookup maps and preset values.
 *
 * <p>getParameters() is not tested here because it depends on RobotState, DriverStation, and
 * Logger. The lookup-table math is the highest-value testable surface.
 */
class LaunchCalculatorTest {

  static {
    Constants.disableHAL();
  }

  @BeforeAll
  static void setup() {
    Constants.disableHAL();
    // Force LaunchCalculator's static initializer by touching a known field
    double ignored = LaunchCalculator.hubPresetDistance;
  }

  // ── Preset distances are positive ────────────────────────────────────────

  @Test
  void presetDistances_arePositive() {
    assertTrue(LaunchCalculator.hubPresetDistance > 0);
    assertTrue(LaunchCalculator.towerPresetDistance > 0);
    assertTrue(LaunchCalculator.trenchPresetDistance > 0);
    assertTrue(LaunchCalculator.outpostPresetDistance > 0);
    assertTrue(LaunchCalculator.passingPresetDistance > 0);
  }

  // ── Presets are ordered by distance ──────────────────────────────────────

  @Test
  void presetDistances_areInAscendingOrder() {
    assertTrue(
        LaunchCalculator.hubPresetDistance < LaunchCalculator.towerPresetDistance, "Hub < Tower");
    assertTrue(
        LaunchCalculator.towerPresetDistance < LaunchCalculator.trenchPresetDistance,
        "Tower < Trench");
    assertTrue(
        LaunchCalculator.trenchPresetDistance < LaunchCalculator.outpostPresetDistance,
        "Trench < Outpost");
    assertTrue(
        LaunchCalculator.outpostPresetDistance < LaunchCalculator.passingPresetDistance,
        "Outpost < Passing");
  }

  // ── Preset flywheel speeds increase with distance ────────────────────────

  @Test
  void flywheelSpeeds_increaseWithDistance() {
    double hub = LaunchCalculator.hubPreset.flywheelSpeed().get();
    double tower = LaunchCalculator.towerPreset.flywheelSpeed().get();
    double trench = LaunchCalculator.trenchPreset.flywheelSpeed().get();
    double outpost = LaunchCalculator.outpostPreset.flywheelSpeed().get();

    assertTrue(hub > 0, "Hub flywheel speed should be positive");
    assertTrue(tower >= hub, "Tower speed should be >= hub speed");
    assertTrue(trench >= tower, "Trench speed should be >= tower speed");
    assertTrue(outpost >= trench, "Outpost speed should be >= trench speed");
  }

  // ── Preset hood angles increase with distance ────────────────────────────

  @Test
  void hoodAngles_increaseWithDistance() {
    double hub = LaunchCalculator.hubPreset.hoodAngleDeg().get();
    double tower = LaunchCalculator.towerPreset.hoodAngleDeg().get();
    double outpost = LaunchCalculator.outpostPreset.hoodAngleDeg().get();

    assertTrue(hub > 0, "Hub hood angle should be positive");
    assertTrue(tower > hub, "Tower angle should be greater than hub angle");
    assertTrue(outpost > tower, "Outpost angle should be greater than tower angle");
  }

  // ── Time of flight accessors ──────────────────────────────────────────────

  @Test
  void minTimeOfFlight_isPositive() {
    assertTrue(LaunchCalculator.getMinTimeOfFlight() > 0);
  }

  @Test
  void maxTimeOfFlight_greaterThanMin() {
    assertTrue(LaunchCalculator.getMaxTimeOfFlight() > LaunchCalculator.getMinTimeOfFlight());
  }

  // ── hoodAngleOffset defaults to zero ─────────────────────────────────────

  @Test
  void newInstance_hoodAngleOffset_isZero() {
    // LaunchCalculator.getInstance() is a singleton; we can't reset it between tests,
    // but the offset should not be non-zero at the start of the test suite.
    assertEquals(0.0, LaunchCalculator.getInstance().getHoodAngleOffsetDeg(), 0.0);
  }
}
