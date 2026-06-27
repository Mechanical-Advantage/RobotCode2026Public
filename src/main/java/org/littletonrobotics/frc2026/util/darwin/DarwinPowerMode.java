// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2026.util.darwin;

import edu.wpi.first.wpilibj.DriverStation;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

public class DarwinPowerMode {
  public enum PowerMode {
    LOW_POWER,
    AUTOMATIC
  }

  // A single-thread executor ensures pmset commands run sequentially without blocking the caller
  private static final ExecutorService executor = Executors.newSingleThreadExecutor();

  // Tracks the active state within the JVM to prevent redundant invocations
  private static PowerMode currentMode = null;
  // Tracks the target state currently being processed by the background thread
  private static PowerMode pendingMode = null;

  /**
   * Sets the system power mode asynchronously. The calling thread is never blocked. The command is
   * only executed if the requested mode differs from the active or pending state.
   *
   * @param mode The desired power mode
   */
  public static synchronized void set(PowerMode mode) {
    // Exit early if the mode is already active or is currently in the queue to be set
    if (mode == currentMode || mode == pendingMode) {
      return;
    }

    // Mark this mode as in-progress
    pendingMode = mode;

    executor.submit(
        () -> {
          // Determine the numeric value for the pmset command
          String modeValue = (mode == PowerMode.LOW_POWER) ? "1" : "0";

          ProcessBuilder processBuilder =
              new ProcessBuilder("sudo", "pmset", "-a", "lowpowermode", modeValue);

          try {
            Process process = processBuilder.start();
            int exitCode = process.waitFor();

            // Re-synchronize to update state variables safely
            synchronized (DarwinPowerMode.class) {
              if (exitCode == 0) {
                currentMode = mode;
              } else {
                DriverStation.reportError(
                    "pmset command failed with exit code: " + exitCode, false);
              }

              // Clear the pending state only if it hasn't been overwritten by a newer request
              if (pendingMode == mode) {
                pendingMode = null;
              }
            }
          } catch (Exception e) {
            DriverStation.reportError("Error while executing pmset: " + e.getMessage(), false);

            synchronized (DarwinPowerMode.class) {
              if (pendingMode == mode) {
                pendingMode = null;
              }
            }
          }
        });
  }
}
