// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2026;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

/**
 * Tests field geometry constants.
 *
 * <p>These tests load AprilTag JSON from src/main/deploy so they require {@link
 * Constants#disableHAL()} to be called before the FieldConstants class is first touched.
 */
class FieldConstantsTest {

  static {
    Constants.disableHAL();
  }

  @BeforeAll
  static void setup() {
    Constants.disableHAL();
  }

  // ── Field dimensions ─────────────────────────────────────────────────────

  @Test
  void fieldLength_isPositive() {
    assertTrue(FieldConstants.fieldLength > 0, "Field length must be positive");
  }

  @Test
  void fieldWidth_isPositive() {
    assertTrue(FieldConstants.fieldWidth > 0, "Field width must be positive");
  }

  @Test
  void fieldLength_greaterThan_fieldWidth() {
    // FRC fields are always wider in the X (length) direction
    assertTrue(
        FieldConstants.fieldLength > FieldConstants.fieldWidth,
        "Field length should exceed field width");
  }

  // ── Hub geometry ─────────────────────────────────────────────────────────

  @Test
  void hub_topCenterPoint_isInsideField() {
    double x = FieldConstants.Hub.topCenterPoint.getX();
    double y = FieldConstants.Hub.topCenterPoint.getY();
    assertTrue(x > 0 && x < FieldConstants.fieldLength, "Hub centre X must be inside field");
    assertTrue(y > 0 && y < FieldConstants.fieldWidth, "Hub centre Y must be inside field");
  }

  @Test
  void hub_nearAndFarCornersAreSymmetricAboutCentre() {
    double centreY = FieldConstants.fieldWidth / 2.0;
    double hubHalfWidth = FieldConstants.Hub.width / 2.0;
    assertEquals(
        centreY + hubHalfWidth,
        FieldConstants.Hub.nearLeftCorner.getY(),
        1e-6,
        "Hub near-left Y should be centre + half-width");
    assertEquals(
        centreY - hubHalfWidth,
        FieldConstants.Hub.nearRightCorner.getY(),
        1e-6,
        "Hub near-right Y should be centre - half-width");
  }

  // ── LeftBump geometry ────────────────────────────────────────────────────

  @Test
  void leftBump_nearRightCorner_isAboveFieldCentre() {
    double fieldCentreY = FieldConstants.fieldWidth / 2.0;
    assertTrue(
        FieldConstants.LeftBump.nearRightCorner.getY() > fieldCentreY,
        "LeftBump near-right corner should be above field centre (high-Y side)");
  }

  // ── RightBump geometry ───────────────────────────────────────────────────

  /**
   * BUG (detected by test): FieldConstants.RightBump.nearRightCorner was set to Hub.nearLeftCorner
   * (Y > fieldWidth/2) instead of Hub.nearRightCorner (Y < fieldWidth/2). The right bump is on the
   * low-Y side so its corners must be below field centre.
   */
  @Test
  void rightBump_nearRightCorner_isBelowFieldCentre() {
    double fieldCentreY = FieldConstants.fieldWidth / 2.0;
    assertTrue(
        FieldConstants.RightBump.nearRightCorner.getY() < fieldCentreY,
        "RightBump near-right corner should be below field centre (low-Y side), "
            + "got Y="
            + FieldConstants.RightBump.nearRightCorner.getY()
            + " vs centre="
            + fieldCentreY);
  }

  @Test
  void rightBump_farRightCorner_isBelowFieldCentre() {
    double fieldCentreY = FieldConstants.fieldWidth / 2.0;
    assertTrue(
        FieldConstants.RightBump.farRightCorner.getY() < fieldCentreY,
        "RightBump far-right corner should be below field centre (low-Y side)");
  }

  // ── LinesVertical sanity ─────────────────────────────────────────────────

  @Test
  void linesVertical_centerIsHalfFieldLength() {
    assertEquals(
        FieldConstants.fieldLength / 2.0,
        FieldConstants.LinesVertical.center,
        1e-6,
        "Field centre line should be at fieldLength/2");
  }

  // ── AprilTag layout ──────────────────────────────────────────────────────

  @Test
  void officialLayout_hasAprilTags() {
    assertFalse(
        FieldConstants.AprilTagLayoutType.OFFICIAL.getLayout().getTags().isEmpty(),
        "Official layout must contain AprilTags");
  }

  @Test
  void officialLayout_tagCountMatchesConstant() {
    assertEquals(
        FieldConstants.aprilTagCount,
        FieldConstants.AprilTagLayoutType.OFFICIAL.getLayout().getTags().size(),
        "aprilTagCount constant must match actual tag count in official layout");
  }
}
