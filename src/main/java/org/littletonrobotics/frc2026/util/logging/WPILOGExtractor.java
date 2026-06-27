// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2026.util.logging;

import edu.wpi.first.util.datalog.DataLogReader;
import edu.wpi.first.util.datalog.DataLogRecord;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.nio.file.*;
import java.util.*;
import java.util.stream.Collectors;
import java.util.stream.Stream;

public class WPILOGExtractor {
  private static final String OUTPUT_SUFFIX = "_extracted.csv";

  public static void main(String[] args) throws Exception {
    if (args.length < 2) {
      System.out.println("Usage: ExtractData <logfile|directory> <key1> [key2] ...");
      return;
    }

    String[] keys = Arrays.copyOfRange(args, 1, args.length);
    List<Path> logFiles = resolveLogFiles(Path.of(args[0]));

    if (logFiles.isEmpty()) {
      System.out.println("No .wpilog or .wpilogxz files found.");
      return;
    }

    for (Path logFile : logFiles) {
      System.out.printf("Processing %s...%n", logFile);
      try {
        extract(logFile, keys);
      } catch (Exception e) {
        System.out.printf("  Error: %s%n", e.getMessage());
      }
    }
  }

  private static List<Path> resolveLogFiles(Path path) throws IOException {
    if (Files.isRegularFile(path)) {
      return List.of(path);
    }
    if (Files.isDirectory(path)) {
      try (Stream<Path> walk = Files.list(path)) {
        return walk.filter(Files::isRegularFile)
            .filter(
                p -> {
                  String name = p.getFileName().toString();
                  return name.endsWith(".wpilog") || name.endsWith(".wpilogxz");
                })
            .sorted()
            .toList();
      }
    }
    return List.of();
  }

  private static void extract(Path logFile, String[] keys) throws Exception {
    String filename = logFile.toString();

    Iterable<DataLogRecord> reader;
    if (filename.endsWith(".wpilogxz")) {
      reader = new WPILOGXZDecoder(filename);
    } else if (filename.endsWith(".wpilog")) {
      reader = new DataLogReader(filename);
    } else {
      System.out.println("  Unsupported file format. Expected .wpilog or .wpilogxz");
      return;
    }

    Map<Integer, String> names = new HashMap<>();
    Map<Integer, String> types = new HashMap<>();

    Map<String, Map<Long, Double>> series = new LinkedHashMap<>();
    for (String key : keys) {
      series.put(key, new HashMap<>());
    }

    var iterator = reader.iterator();
    while (iterator.hasNext()) {
      DataLogRecord record;
      try {
        record = iterator.next();
      } catch (Exception e) {
        break;
      }

      if (record.isControl() && record.isStart()) {
        var start = record.getStartData();
        names.put(start.entry, start.name);
        types.put(start.entry, start.type);
      } else if (!record.isControl()) {
        String name = names.get(record.getEntry());
        if (name == null) continue;
        String clean = name.startsWith("/") ? name.substring(1) : name;

        Map<Long, Double> target = series.get(clean);
        if (target == null) continue;

        String type = types.getOrDefault(record.getEntry(), "");
        if (type.equals("double")) target.put(record.getTimestamp(), record.getDouble());
        else if (type.equals("float"))
          target.put(record.getTimestamp(), (double) record.getFloat());
      }
    }

    // First key defines the time base
    String primaryKey = keys[0];
    var primaryData = series.get(primaryKey);
    var sortedTs = primaryData.keySet().stream().sorted().toList();

    if (sortedTs.isEmpty()) {
      System.out.println("  No data found for primary key: " + primaryKey);
      return;
    }
    long t0 = sortedTs.get(0);

    // Build sorted arrays for interpolation of non-primary keys
    record InterpData(long[] times, double[] values) {}

    Map<String, InterpData> interps = new LinkedHashMap<>();
    for (int i = 1; i < keys.length; i++) {
      var data = series.get(keys[i]);
      var sorted = data.entrySet().stream().sorted(Map.Entry.comparingByKey()).toList();
      long[] t = sorted.stream().mapToLong(Map.Entry::getKey).toArray();
      double[] v = sorted.stream().mapToDouble(Map.Entry::getValue).toArray();
      interps.put(keys[i], new InterpData(t, v));
    }

    String outPath = filename.replaceAll("\\.[^.]+$", "") + OUTPUT_SUFFIX;
    try (var pw = new PrintWriter(new FileWriter(outPath))) {
      String header =
          "t_seconds,"
              + Arrays.stream(keys).map(k -> k.replace("/", "_")).collect(Collectors.joining(","));
      pw.println(header);

      for (long ts : sortedTs) {
        StringBuilder sb = new StringBuilder();
        sb.append(String.format("%.6f", (ts - t0) / 1e6));
        sb.append(String.format(",%.6f", primaryData.get(ts)));
        for (int i = 1; i < keys.length; i++) {
          var interp = interps.get(keys[i]);
          sb.append(String.format(",%.6f", interpolate(interp.times, interp.values, ts)));
        }
        pw.println(sb);
      }
    }

    System.out.printf("  Wrote %d samples to %s%n", sortedTs.size(), outPath);
  }

  private static double interpolate(long[] xs, double[] ys, long x) {
    if (xs.length == 0) return 0.0;
    if (x <= xs[0]) return ys[0];
    if (x >= xs[xs.length - 1]) return ys[ys.length - 1];
    int i = 0;
    while (i < xs.length - 1 && xs[i + 1] < x) i++;
    double frac = (double) (x - xs[i]) / (xs[i + 1] - xs[i]);
    return ys[i] + frac * (ys[i + 1] - ys[i]);
  }
}
