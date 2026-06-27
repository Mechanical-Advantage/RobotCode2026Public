// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2026.util.darwin;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.concurrent.atomic.AtomicReference;

/**
 * A utility class that monitors Mac power usage in the background by directly reading the SAP Power
 * Monitor's binary ring buffer data store.
 */
public class DarwinPowerMonitor {
  private static final AtomicReference<Double> currentPowerWatts = new AtomicReference<>(0.0);
  private static volatile boolean isRunning = false;
  private static Thread pollingThread;

  private static final Path dataStorePath =
      Paths.get("/Users/Shared/Power Monitor/measurements.pwrdata");
  private static final int pollingIntervalMs = 5000;
  private static final int headerSize = 10;
  private static final int recordSize = 13;

  // Private constructor to prevent instantiation
  private DarwinPowerMonitor() {}

  /** Starts the background daemon thread to poll power usage directly from the binary file. */
  public static synchronized void start() {
    if (isRunning) {
      return;
    }
    isRunning = true;

    pollingThread =
        new Thread(
            () -> {
              while (isRunning) {
                double watts = readLatestPowerFromDataStore();
                if (watts >= 0.0) {
                  currentPowerWatts.set(watts);
                }

                try {
                  Thread.sleep(pollingIntervalMs);
                } catch (InterruptedException e) {
                  Thread.currentThread().interrupt();
                  break;
                }
              }
            },
            "DarwinPowerMonitor");

    pollingThread.setDaemon(true);
    pollingThread.start();
  }

  /** Stops the background polling thread. */
  public static synchronized void stop() {
    isRunning = false;
    if (pollingThread != null) {
      pollingThread.interrupt();
      pollingThread = null;
    }
  }

  /**
   * Returns the most recently recorded power draw.
   *
   * @return The current power draw in watts.
   */
  public static double getCurrentPowerWatts() {
    if (!isRunning) {
      start();
    }
    return currentPowerWatts.get();
  }

  /** Reads the binary ring buffer and extracts the most recent measurement. */
  private static double readLatestPowerFromDataStore() {
    if (!Files.exists(dataStorePath)) {
      return -1.0;
    }

    try {
      // The file is typically very small (~200KB for a 24-hour buffer)
      // Reading it fully into memory avoids file locking conflicts with the native daemon
      byte[] fileBytes = Files.readAllBytes(dataStorePath);
      if (fileBytes.length < headerSize) {
        return -1.0;
      }

      ByteBuffer buffer = ByteBuffer.wrap(fileBytes);
      buffer.order(ByteOrder.BIG_ENDIAN);

      // Skip the 10-byte header
      buffer.position(headerSize);

      long latestTimestamp = -1;
      float latestPower = -1.0f;

      // Iterate over the 13-byte chunks to find the most recent entry
      while (buffer.remaining() >= recordSize) {
        long timestamp = buffer.getLong();
        float power = buffer.getFloat();
        buffer.get(); // We must consume this byte to align for the next chunk

        // Because this is a ring buffer, we track the max timestamp across the whole file
        if (timestamp > latestTimestamp) {
          latestTimestamp = timestamp;
          latestPower = power;
        }
      }

      if (latestTimestamp > 0) {
        return latestPower;
      }

    } catch (Exception e) {
      // Silently swallow exceptions (e.g., IO exceptions during a concurrent write)
      // to allow the next polling cycle to retry cleanly.
    }

    return -1.0;
  }
}
