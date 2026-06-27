// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2026.util.logging;

import com.sun.jna.Library;
import com.sun.jna.Native;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.MatchType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import java.io.BufferedOutputStream;
import java.io.File;
import java.io.FileDescriptor;
import java.io.FileOutputStream;
import java.io.IOException;
import java.lang.reflect.Field;
import java.nio.file.FileSystems;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;
import java.util.HashMap;
import java.util.Map;
import java.util.Random;
import org.littletonrobotics.junction.LogDataReceiver;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.LogTable.LogValue;
import org.littletonrobotics.junction.LogTable.LoggableType;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.wpilog.WPILOGWriter.AdvantageScopeOpenBehavior;

/** Records log values to a compressed WPILOG file (.wpilogxz). */
public class WPILOGXZWriter implements LogDataReceiver {
  private static final double timestampUpdateDelay =
      5.0; // Wait several seconds after DS attached to ensure timestamp/timezone is updated
  private static final long flushPeriod = 250000L; // Flush every 250ms
  private static final String defaultPathRio = "/U/logs";
  private static final String defaultPathSim = "logs";
  private static final DateTimeFormatter timeFormatter =
      DateTimeFormatter.ofPattern("yy-MM-dd_HH-mm-ss");
  private static final String advantageScopeFileName = "ascope-log-path.txt";

  private String folder;
  private String filename;
  private final String randomIdentifier;
  private Double dsAttachedTime;

  private boolean autoRename;
  private LocalDateTime logDate;
  private String logMatchText;

  private WPILOGXZEncoder encoder;
  private boolean isOpen = false;
  private FileOutputStream fileOutputStream;
  private final AdvantageScopeOpenBehavior openBehavior;
  private LogTable lastTable;
  private int timestampID;
  private long lastFlushTimestamp = 0;
  private Map<String, Integer> entryIDs;
  private Map<String, LoggableType> entryTypes;
  private Map<String, String> entryUnits;

  /**
   * Create a new WPILOGWriter for writing to a ".wpilogxz" file.
   *
   * @param path Path to log file or folder. If only a folder is provided, the filename will be
   *     generated based on the current time and match number (if applicable).
   * @param openBehavior Whether to automatically open the log file in AdvantageScope.
   */
  public WPILOGXZWriter(String path, AdvantageScopeOpenBehavior openBehavior) {
    this.openBehavior = openBehavior;

    // Create random identifier
    Random random = new Random();
    StringBuilder randomIdentifierBuilder = new StringBuilder();
    for (int i = 0; i < 4; i++) {
      randomIdentifierBuilder.append(String.format("%04x", random.nextInt(0x10000)));
    }
    randomIdentifier = randomIdentifierBuilder.toString();

    // Set up folder and filename
    if (path.endsWith(".wpilog") || path.endsWith(".wpilogxz")) {
      File pathFile = new File(path);
      folder = pathFile.getParent();
      filename = pathFile.getName();
      autoRename = false;
    } else {
      folder = path;
      filename = "akit_" + randomIdentifier + ".wpilogxz";
      autoRename = true;
    }
  }

  public WPILOGXZWriter(String path) {
    this(path, AdvantageScopeOpenBehavior.AUTO);
  }

  public WPILOGXZWriter(AdvantageScopeOpenBehavior openBehavior) {
    this(RobotBase.isSimulation() ? defaultPathSim : defaultPathRio, openBehavior);
  }

  public WPILOGXZWriter() {
    this(
        RobotBase.isSimulation() ? defaultPathSim : defaultPathRio,
        AdvantageScopeOpenBehavior.AUTO);
  }

  public void start() {
    // Create folder if necessary
    File logFolder = new File(folder);
    if (!logFolder.exists()) {
      logFolder.mkdirs();
    }

    // Delete log if it already exists
    File logFile = new File(folder, filename);
    if (logFile.exists()) {
      logFile.delete();
    }

    // Create new log
    String logPath = Path.of(folder, filename).toString();
    System.out.println("[AdvantageKit] Logging to \"" + logPath + "\" (LZMA2 compressed)");
    try {
      fileOutputStream = new FileOutputStream(logPath);
      BufferedOutputStream bufferedOutputStream = new BufferedOutputStream(fileOutputStream);

      // Initialize the Encoder (which handles LZMA compression)
      encoder = new WPILOGXZEncoder(bufferedOutputStream);
      encoder.writeHeader(WPILOGConstants.extraHeader);

      // Start timestamp entry
      timestampID =
          encoder.startEntry(
              timestampKey, LoggableType.Integer.getWPILOGType(), WPILOGConstants.entryMetadata, 0);
    } catch (IOException e) {
      DriverStation.reportError("[AdvantageKit] Failed to open output log file.", true);
      return;
    }

    isOpen = true;
    lastTable = new LogTable(0);

    // Reset data
    entryIDs = new HashMap<>();
    entryTypes = new HashMap<>();
    entryUnits = new HashMap<>();
    logDate = null;
    logMatchText = null;
  }

  private void flush() throws IOException {
    encoder.flush();
    FileDescriptor fdObj = fileOutputStream.getFD();

    try {
      // Extract the raw POSIX file descriptor integer via reflection
      Field fdField = FileDescriptor.class.getDeclaredField("fd");
      fdField.setAccessible(true);
      int nativeFd = fdField.getInt(fdObj);

      // Execute F_FULLFSYNC
      int result = LibC.INSTANCE.fcntl(nativeFd, LibC.F_FULLFSYNC);

      // If fcntl fails (returns -1), fallback to the standard Java sync
      if (result == -1) {
        fdObj.sync();
      }
    } catch (Exception e) {
      e.printStackTrace();

      // Fallback if reflection fails
      fdObj.sync();
    }
  }

  public void end() {
    if (!isOpen) return;

    // Flush remaining data
    try {
      flush();
    } catch (IOException e) {
      DriverStation.reportError("[AdvantageKit] Failed to flush log file.", false);
    }

    // Close log file
    try {
      encoder.close();
      isOpen = false;
    } catch (IOException e) {
      DriverStation.reportError("[AdvantageKit] Failed to close log file.", false);
    }

    // Send log path to AdvantageScope
    boolean shouldOpen =
        switch (openBehavior) {
          case ALWAYS -> RobotBase.isSimulation();
          case AUTO -> RobotBase.isSimulation() && Logger.hasReplaySource();
          case NEVER -> false;
        };
    if (shouldOpen) {
      try {
        String fullLogPath =
            FileSystems.getDefault()
                .getPath(folder, filename)
                .normalize()
                .toAbsolutePath()
                .toString();
        Path advantageScopeTempPath =
            Paths.get(System.getProperty("java.io.tmpdir"), advantageScopeFileName);
        java.io.PrintWriter writer =
            new java.io.PrintWriter(advantageScopeTempPath.toString(), "UTF-8");
        writer.println(fullLogPath);
        writer.close();
        System.out.println("[AdvantageKit] Log sent to AdvantageScope.");
      } catch (Exception e) {
        DriverStation.reportError("[AdvantageKit] Failed to send log to AdvantageScope.", false);
      }
    }
  }

  public void putTable(LogTable table) {
    // Exit if log not open
    if (!isOpen) return;

    // Auto rename
    if (autoRename) {
      // Update timestamp
      if (logDate == null) {
        if ((table.get("DriverStation/DSAttached", false)
                && table.get("SystemStats/SystemTimeValid", false))
            || RobotBase.isSimulation()) {
          if (dsAttachedTime == null) {
            dsAttachedTime = RobotController.getFPGATime() / 1000000.0;
          } else if (RobotController.getFPGATime() / 1000000.0 - dsAttachedTime
                  > timestampUpdateDelay
              || RobotBase.isSimulation()) {
            logDate = LocalDateTime.now();
          }
        } else {
          dsAttachedTime = null;
        }
      }

      // Update match
      MatchType matchType;
      switch (table.get("DriverStation/MatchType", 0)) {
        case 1:
          matchType = MatchType.Practice;
          break;
        case 2:
          matchType = MatchType.Qualification;
          break;
        case 3:
          matchType = MatchType.Elimination;
          break;
        default:
          matchType = MatchType.None;
          break;
      }
      if (logMatchText == null && matchType != MatchType.None) {
        logMatchText = "";
        switch (matchType) {
          case Practice:
            logMatchText = "p";
            break;
          case Qualification:
            logMatchText = "q";
            break;
          case Elimination:
            logMatchText = "e";
            break;
          default:
            break;
        }
        logMatchText += Integer.toString(table.get("DriverStation/MatchNumber", 0));
      }

      // Update filename
      StringBuilder newFilenameBuilder = new StringBuilder();
      newFilenameBuilder.append("akit_");
      if (logDate == null) {
        newFilenameBuilder.append(randomIdentifier);
      } else {
        newFilenameBuilder.append(timeFormatter.format(logDate));
      }
      String eventName = table.get("DriverStation/EventName", "").toLowerCase();
      if (eventName.length() > 0) {
        newFilenameBuilder.append("_");
        newFilenameBuilder.append(eventName);
      }
      if (logMatchText != null) {
        newFilenameBuilder.append("_");
        newFilenameBuilder.append(logMatchText);
      }
      newFilenameBuilder.append(".wpilogxz");
      String newFilename = newFilenameBuilder.toString();
      if (!newFilename.equals(filename)) {
        String logPath = Path.of(folder, newFilename).toString();
        System.out.println("[AdvantageKit] Renaming log to \"" + logPath + "\"");

        File fileA = new File(folder, filename);
        File fileB = new File(folder, newFilename);
        if (fileA.renameTo(fileB)) {
          filename = newFilename;
        }
      }
    }

    try {
      // Save timestamp
      encoder.appendInteger(timestampID, table.getTimestamp(), table.getTimestamp());

      // Get new and old data
      Map<String, LogValue> newMap = table.getAll(false);
      Map<String, LogValue> oldMap = lastTable.getAll(false);

      // Encode fields
      for (Map.Entry<String, LogValue> field : newMap.entrySet()) {

        // Check if field should be updated
        LoggableType type = field.getValue().type;
        String unit = field.getValue().unitStr;
        boolean appendData = false;

        if (!entryIDs.containsKey(field.getKey())) { // New field
          String metadata =
              unit == null
                  ? WPILOGConstants.entryMetadata
                  : WPILOGConstants.entryMetadataUnits.replace("$UNITSTR", unit);
          int id =
              encoder.startEntry(
                  field.getKey(), field.getValue().getWPILOGType(), metadata, table.getTimestamp());
          entryIDs.put(field.getKey(), id);
          entryTypes.put(field.getKey(), type);
          if (unit != null) {
            entryUnits.put(field.getKey(), unit);
          }
          appendData = true;

        } else if (!field.getValue().equals(oldMap.get(field.getKey()))) { // Updated field
          appendData = true;
        }

        // Append data
        if (appendData) {
          int id = entryIDs.get(field.getKey());

          // Check if unit changed
          if (unit != null && !unit.equals(entryUnits.get(field.getKey()))) {
            encoder.setMetadata(
                id,
                WPILOGConstants.entryMetadataUnits.replace("$UNITSTR", unit),
                table.getTimestamp());
            entryUnits.put(field.getKey(), unit);
          }

          // Add field value
          LogValue val = field.getValue();
          switch (val.type) {
            case Raw:
              encoder.appendRaw(id, val.getRaw(), table.getTimestamp());
              break;
            case Boolean:
              encoder.appendBoolean(id, val.getBoolean(), table.getTimestamp());
              break;
            case Integer:
              encoder.appendInteger(id, val.getInteger(), table.getTimestamp());
              break;
            case Float:
              encoder.appendFloat(id, val.getFloat(), table.getTimestamp());
              break;
            case Double:
              encoder.appendDouble(id, val.getDouble(), table.getTimestamp());
              break;
            case String:
              encoder.appendString(id, val.getString(), table.getTimestamp());
              break;
            case BooleanArray:
              encoder.appendBooleanArray(id, val.getBooleanArray(), table.getTimestamp());
              break;
            case IntegerArray:
              encoder.appendIntegerArray(id, val.getIntegerArray(), table.getTimestamp());
              break;
            case FloatArray:
              encoder.appendFloatArray(id, val.getFloatArray(), table.getTimestamp());
              break;
            case DoubleArray:
              encoder.appendDoubleArray(id, val.getDoubleArray(), table.getTimestamp());
              break;
            case StringArray:
              encoder.appendStringArray(id, val.getStringArray(), table.getTimestamp());
              break;
          }
        }
      }

      // Flush to disk. Manually flushing harms compression performance a bit, but
      // ensures that data is written in a timely manner in case of a hard shutdown
      if (table.getTimestamp() - lastFlushTimestamp > flushPeriod) {
        flush();
        lastFlushTimestamp = table.getTimestamp();
      }

    } catch (IOException e) {
      e.printStackTrace();
    }

    // Update last table
    lastTable = table;
  }

  /** JNA wrapper for fcntl */
  public static interface LibC extends Library {
    LibC INSTANCE = Native.load("c", LibC.class);

    public static final int F_FULLFSYNC = 51;

    int fcntl(int fd, int cmd);
  }
}
