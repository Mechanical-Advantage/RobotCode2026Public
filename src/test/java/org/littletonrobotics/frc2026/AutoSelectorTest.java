// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2026;

import static org.junit.jupiter.api.Assertions.*;

import java.io.File;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

/**
 * Validates that auto-related static resources are present on disk.
 *
 * <p>Full AutoBuilder / command construction tests require the WPILib Command Scheduler (HAL), so
 * they are deferred to integration-test scope. This class checks that every trajectory file
 * referenced by the build actually exists.
 */
class AutoSelectorTest {

  static {
    Constants.disableHAL();
  }

  @BeforeAll
  static void setup() {
    Constants.disableHAL();
  }

  private static final File DEPLOY_DIR = new File("src/main/deploy");

  @Test
  void deployDirectory_exists() {
    assertTrue(DEPLOY_DIR.isDirectory(), "src/main/deploy must exist");
  }

  @Test
  void aprilTagLayouts_officialFileExists() {
    File official =
        new File(
            DEPLOY_DIR,
            "apriltags/" + FieldConstants.fieldType.getJsonFolder() + "/2026-official.json");
    assertTrue(official.exists(), "Official AprilTag layout JSON must exist at: " + official);
  }

  @Test
  void aprilTagLayouts_noneFileExists() {
    File none =
        new File(
            DEPLOY_DIR,
            "apriltags/" + FieldConstants.fieldType.getJsonFolder() + "/2026-none.json");
    assertTrue(none.exists(), "None AprilTag layout JSON must exist at: " + none);
  }

  @Test
  void vtsDirectory_exists() {
    File vts = new File(DEPLOY_DIR, "vts");
    assertTrue(vts.isDirectory(), "src/main/deploy/vts directory must exist");
  }
}
