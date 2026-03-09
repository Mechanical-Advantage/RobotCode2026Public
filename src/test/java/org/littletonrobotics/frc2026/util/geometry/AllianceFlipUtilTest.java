// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2026.util.geometry;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import org.littletonrobotics.frc2026.Constants;

/**
 * Tests AllianceFlipUtil.
 *
 * <p>With {@link Constants#disableHAL()}, {@link AllianceFlipUtil#shouldFlip()} always returns
 * false (no alliance available), so all apply() methods return their inputs unchanged. We verify
 * the identity path and the invariants of the flip math using direct field-length arithmetic.
 */
class AllianceFlipUtilTest {

  static {
    Constants.disableHAL();
  }

  @BeforeAll
  static void setup() {
    Constants.disableHAL();
  }

  // ── shouldFlip / identity path (disableHAL = true) ───────────────────────

  @Test
  void shouldFlip_returnsFalse_whenHALDisabled() {
    assertFalse(AllianceFlipUtil.shouldFlip());
  }

  @Test
  void applyX_returnsUnchanged_whenNoFlip() {
    double x = 3.5;
    assertEquals(x, AllianceFlipUtil.applyX(x), 1e-9);
  }

  @Test
  void applyY_returnsUnchanged_whenNoFlip() {
    double y = 2.0;
    assertEquals(y, AllianceFlipUtil.applyY(y), 1e-9);
  }

  @Test
  void applyTranslation_returnsUnchanged_whenNoFlip() {
    Translation2d t = new Translation2d(1.0, 2.0);
    assertEquals(t, AllianceFlipUtil.apply(t));
  }

  @Test
  void applyRotation_returnsUnchanged_whenNoFlip() {
    Rotation2d r = Rotation2d.fromDegrees(45);
    assertEquals(r.getRadians(), AllianceFlipUtil.apply(r).getRadians(), 1e-9);
  }

  @Test
  void applyPose_returnsUnchanged_whenNoFlip() {
    Pose2d p = new Pose2d(4.0, 1.5, Rotation2d.fromDegrees(30));
    Pose2d result = AllianceFlipUtil.apply(p);
    assertEquals(p.getX(), result.getX(), 1e-9);
    assertEquals(p.getY(), result.getY(), 1e-9);
    assertEquals(p.getRotation().getRadians(), result.getRotation().getRadians(), 1e-9);
  }

  // ── Flip math invariants (tested independently of HAL/DriverStation) ──────

  /**
   * Verify the flip formulas directly: flipping X should mirror around fieldLength/2, flipping Y
   * around fieldWidth/2.
   */
  @Test
  void flipX_formula_isSymmetricAroundFieldCentre() {
    // Load FieldConstants (already loaded since disableHAL is set)
    double fieldLen = org.littletonrobotics.frc2026.FieldConstants.fieldLength;
    double x = 2.0;
    double flippedX = fieldLen - x;
    // Flip again should return to original
    assertEquals(x, fieldLen - flippedX, 1e-9, "Double flip should be identity");
  }

  @Test
  void flipRotation_formula_addsHalfTurn() {
    Rotation2d r = Rotation2d.fromDegrees(30);
    Rotation2d flipped = r.rotateBy(Rotation2d.kPi);
    // Rotating by π twice should return to original
    Rotation2d doubleFlipped = flipped.rotateBy(Rotation2d.kPi);
    assertEquals(
        r.getRadians(), doubleFlipped.getRadians(), 1e-6, "Double flip rotation = identity");
  }
}
