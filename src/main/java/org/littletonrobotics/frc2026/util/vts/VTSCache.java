// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2026.util.vts;

import java.io.File;
import java.io.IOException;
import java.nio.channels.FileChannel;
import java.nio.channels.FileLock;
import java.nio.file.AtomicMoveNotSupportedException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.StandardCopyOption;
import java.nio.file.StandardOpenOption;
import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;
import java.util.function.BooleanSupplier;

public class VTSCache {
  private static final String cacheEnvVar = "VTS_CACHE";
  private static final Map<String, Object> locks = new ConcurrentHashMap<>();

  /**
   * Retrieves a trajectory from the VTS_CACHE directory if available, or invokes the generator and
   * stores the result in VTS_CACHE.
   *
   * @param hashcode The expected trajectory hashcode.
   * @param name The trajectory name.
   * @param targetFile The destination ".traj" file in deploy/vts.
   * @param generator Callback that generates targetFile and returns true on success.
   * @return True if targetFile was successfully populated (from cache or generator).
   */
  public static boolean getOrGenerate(
      String hashcode, String name, File targetFile, BooleanSupplier generator) {
    String cacheEnv = System.getenv(cacheEnvVar);
    if (cacheEnv == null || cacheEnv.isBlank()) {
      return generator.getAsBoolean();
    }

    Path cacheDir = Path.of(cacheEnv);
    try {
      Files.createDirectories(cacheDir);
    } catch (IOException e) {
      System.err.println(
          name + ": Failed to create VTS_CACHE directory (" + cacheDir + "), generating locally");
      e.printStackTrace();
      return generator.getAsBoolean();
    }

    Path lockFile = cacheDir.resolve(hashcode + ".lock");
    Path cacheFile = cacheDir.resolve(hashcode + ".traj");
    Object monitor = locks.computeIfAbsent(hashcode, k -> new Object());

    synchronized (monitor) {
      try (FileChannel channel =
              FileChannel.open(lockFile, StandardOpenOption.CREATE, StandardOpenOption.WRITE);
          FileLock ignored = channel.lock()) {
        // Check if cached trajectory exists and is valid
        if (Files.exists(cacheFile)) {
          String cachedHash = GenerateTrajectories.readTrajHash(cacheFile.toFile(), name);
          if (hashcode.equals(cachedHash)) {
            Files.copy(cacheFile, targetFile.toPath(), StandardCopyOption.REPLACE_EXISTING);
            System.out.println(name + ": Copied from cache ✅");
            return true;
          } else {
            // Corrupted or incomplete cache entry; remove and regenerate
            Files.deleteIfExists(cacheFile);
          }
        }

        // Cache miss, generate trajectory
        boolean generated = generator.getAsBoolean();
        if (generated && targetFile.exists()) {
          Path tempFile = Files.createTempFile(cacheDir, hashcode + "-", ".tmp");
          try {
            Files.copy(targetFile.toPath(), tempFile, StandardCopyOption.REPLACE_EXISTING);
            try {
              Files.move(
                  tempFile,
                  cacheFile,
                  StandardCopyOption.REPLACE_EXISTING,
                  StandardCopyOption.ATOMIC_MOVE);
            } catch (AtomicMoveNotSupportedException e) {
              Files.move(tempFile, cacheFile, StandardCopyOption.REPLACE_EXISTING);
            }
          } finally {
            Files.deleteIfExists(tempFile);
          }
        }
        return generated;
      } catch (IOException e) {
        System.err.println(name + ": Cache operation failed in VTS_CACHE, generating locally");
        e.printStackTrace();
        return generator.getAsBoolean();
      }
    }
  }
}
