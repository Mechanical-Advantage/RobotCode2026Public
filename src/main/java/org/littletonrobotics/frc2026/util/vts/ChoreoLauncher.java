// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2026.util.vts;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.net.URI;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.nio.file.StandardCopyOption;
import java.util.Comparator;
import java.util.Locale;
import java.util.stream.Stream;
import java.util.zip.ZipEntry;
import java.util.zip.ZipInputStream;

public class ChoreoLauncher {
  public static final String choreoVersion = "v2026.0.3";

  private static final String baseURL =
      "https://github.com/SleipnirGroup/Choreo/releases/download/";
  private static final Path cacheDir = Paths.get("build", "choreo");

  /**
   * Checks if the Choreo CLI is available, downloading it if necessary.
   *
   * @return Whether the Choreo CLI is available.
   */
  public static boolean checkChoreoCLI() {
    try {
      getOrDownloadChoreo();
      return true;
    } catch (IOException e) {
      return false;
    }
  }

  /**
   * Generates a trajectory using the Choreo CLI. Downloads the CLI if not already present.
   *
   * @param chorFile The path to the ".chor" project file.
   * @param trajectoryName The name of the trajectory to generate. The ".traj" trajectory file
   *     should be in the same folder as the ".chor" project file.
   * @throws IOException If there is an IO error.
   * @throws InterruptedException If the process is interrupted.
   * @throws RuntimeException If generation fails.
   */
  public static void generateTrajectory(File chorFile, String trajectoryName)
      throws IOException, InterruptedException {
    if (!chorFile.exists()) {
      throw new FileNotFoundException("Choreo file not found: " + chorFile.getAbsolutePath());
    }

    Path binaryPath = getOrDownloadChoreo();

    // Ensure binary is executable
    if (!System.getProperty("os.name").toLowerCase().contains("win")) {
      binaryPath.toFile().setExecutable(true);
    }

    ProcessBuilder pb =
        new ProcessBuilder(
            binaryPath.toString(),
            "--chor",
            chorFile.getAbsolutePath(),
            "--trajectory",
            trajectoryName,
            "--generate");

    // Redirect stderr to stdout to capture all output in one stream for parsing
    pb.redirectErrorStream(true);

    Process process = pb.start();

    // Capture output
    StringBuilder output = new StringBuilder();
    try (BufferedReader reader =
        new BufferedReader(new InputStreamReader(process.getInputStream()))) {
      String line;
      while ((line = reader.readLine()) != null) {
        output.append(line).append(System.lineSeparator());
      }
    }

    int exitCode = process.waitFor();
    String fullOutput = output.toString();

    // Check for macOS Gatekeeper warning
    boolean isMac = System.getProperty("os.name").toLowerCase().contains("mac");
    if (isMac
        && (exitCode == 137
            || fullOutput.contains("killed")
            || fullOutput.contains("Kill")
            || exitCode == 9)) {
      throw new RuntimeException(
          "\n\n!!! CHOREO GENERATION FAILED (MacOS Security Block) !!!\n"
              + "The Choreo CLI binary is unnotarized and was likely blocked by macOS Gatekeeper.\n"
              + "Please go to System Settings > Privacy & Security > Security, scroll down,\n"
              + "and click 'Allow Anyway' next to the message about 'choreo-cli'.\n"
              + "Then try running this generation again.\n");
    }

    // Check for specific success message since exit codes are unused
    if (!fullOutput.contains("Successfully generated trajectory")) {
      throw new RuntimeException(
          "Choreo generation failed for '"
              + trajectoryName
              + "'. Try generating the trajectory in Choreo for details.");
    }
  }

  private static Path getOrDownloadChoreo() throws IOException {
    String os = System.getProperty("os.name").toLowerCase(Locale.ROOT);
    String arch = System.getProperty("os.arch").toLowerCase(Locale.ROOT);

    boolean isArm = arch.equals("aarch64") || arch.equals("arm64");

    String platformString;
    String extension = "zip";
    String binaryName = "choreo-cli";

    if (os.contains("win")) {
      binaryName = "choreo-cli.exe";
      platformString = isArm ? "Windows-aarch64" : "Windows-x86_64";
    } else if (os.contains("mac")) {
      platformString = isArm ? "macOS-aarch64" : "macOS-x86_64";
    } else if (os.contains("nux") || os.contains("nix")) {
      platformString = isArm ? "Linux-aarch64" : "Linux-x86_64";
    } else {
      throw new UnsupportedOperationException("Unsupported OS: " + os);
    }

    String zipFileName =
        "Choreo-" + choreoVersion + "-" + platformString + "-standalone." + extension;
    String downloadUrl = baseURL + choreoVersion + "/" + zipFileName;

    Path cliDir = cacheDir.resolve(choreoVersion);
    Path binaryPath = cliDir.resolve(binaryName);

    // Clean up old versions if they exist (folders not matching current version)
    cleanOldVersions(cacheDir, choreoVersion);

    // If binary exists and looks correct, return it
    if (Files.exists(binaryPath)) {
      return binaryPath;
    }

    // Ensure clean state for this version
    if (Files.exists(cliDir)) {
      deleteDirectory(cliDir);
    }
    Files.createDirectories(cliDir);

    Path zipPath = cliDir.resolve("choreo.zip");

    try {
      // Download Zip
      try (InputStream in = URI.create(downloadUrl).toURL().openStream()) {
        Files.copy(in, zipPath, StandardCopyOption.REPLACE_EXISTING);
      }

      // Extract Zip
      boolean foundBinary = false;
      try (ZipInputStream zis = new ZipInputStream(new FileInputStream(zipPath.toFile()))) {
        ZipEntry zipEntry = zis.getNextEntry();
        while (zipEntry != null) {
          // We only care about the executable
          String fileName = Paths.get(zipEntry.getName()).getFileName().toString();
          if (fileName.equals(binaryName)) {
            Files.copy(zis, binaryPath, StandardCopyOption.REPLACE_EXISTING);
            foundBinary = true;
            break;
          }
          zipEntry = zis.getNextEntry();
        }
        zis.closeEntry();
      }

      if (!foundBinary) {
        throw new FileNotFoundException(
            "Could not find '" + binaryName + "' inside the downloaded zip.");
      }
    } catch (IOException e) {
      // Cleanup failed download to prevent partial state on next run
      try {
        deleteDirectory(cliDir);
      } catch (IOException cleanupEx) {
        e.addSuppressed(cleanupEx);
      }
      throw new IOException(
          "Failed to download or extract Choreo CLI. Check internet connection and version tag.",
          e);
    } finally {
      // Always delete the zip file
      Files.deleteIfExists(zipPath);
    }

    return binaryPath;
  }

  private static void cleanOldVersions(Path cacheDir, String currentVersion) {
    if (!Files.exists(cacheDir)) return;

    try (Stream<Path> stream = Files.list(cacheDir)) {
      stream
          .filter(Files::isDirectory)
          .filter(path -> !path.getFileName().toString().equals(currentVersion))
          .forEach(
              path -> {
                try {
                  System.out.println("Removing old Choreo version: " + path.getFileName());
                  deleteDirectory(path);
                } catch (IOException e) {
                  System.err.println("Failed to clean up old version directory: " + path);
                }
              });
    } catch (IOException e) {
      System.err.println("Failed to access cache directory for cleanup: " + e.getMessage());
    }
  }

  private static void deleteDirectory(Path path) throws IOException {
    if (Files.exists(path)) {
      try (Stream<Path> walk = Files.walk(path)) {
        walk.sorted(Comparator.reverseOrder()).map(Path::toFile).forEach(File::delete);
      }
    }
  }
}
