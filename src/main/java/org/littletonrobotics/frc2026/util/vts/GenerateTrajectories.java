// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2026.util.vts;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.SerializationFeature;
import com.fasterxml.jackson.databind.node.ObjectNode;
import com.google.common.hash.Hashing;
import java.io.File;
import java.io.IOException;
import java.math.RoundingMode;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.text.DecimalFormat;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.concurrent.ConcurrentHashMap;
import org.littletonrobotics.frc2026.Constants;
import org.littletonrobotics.frc2026.subsystems.drive.DriveConstants;
import org.littletonrobotics.frc2026.subsystems.drive.DriveTrajectories;
import org.littletonrobotics.frc2026.util.vts.ChoreoJSON.BumperConfig;
import org.littletonrobotics.frc2026.util.vts.ChoreoJSON.ChorFileRoot;
import org.littletonrobotics.frc2026.util.vts.ChoreoJSON.Codegen;
import org.littletonrobotics.frc2026.util.vts.ChoreoJSON.Constraint;
import org.littletonrobotics.frc2026.util.vts.ChoreoJSON.ExpVal;
import org.littletonrobotics.frc2026.util.vts.ChoreoJSON.ModuleConfig;
import org.littletonrobotics.frc2026.util.vts.ChoreoJSON.Params;
import org.littletonrobotics.frc2026.util.vts.ChoreoJSON.RobotConfig;
import org.littletonrobotics.frc2026.util.vts.ChoreoJSON.Snapshot;
import org.littletonrobotics.frc2026.util.vts.ChoreoJSON.TrajFileRoot;
import org.littletonrobotics.frc2026.util.vts.ChoreoJSON.TrajectorySection;
import org.littletonrobotics.frc2026.util.vts.ChoreoJSON.Variable;
import org.littletonrobotics.frc2026.util.vts.ChoreoJSON.Waypoint;
import org.littletonrobotics.frc2026.util.vts.request.PathRequest;
import org.littletonrobotics.frc2026.util.vts.request.PathRequestSegment;
import org.littletonrobotics.frc2026.util.vts.request.PathWaypoint;

public class GenerateTrajectories {
  static ObjectMapper mapper;

  public static void main(String[] args) {
    Constants.disableHAL();
    mapper = new ObjectMapper();
    mapper.enable(SerializationFeature.INDENT_OUTPUT);

    // Configure drive constants
    final Map<String, Double> frontLeft =
        Map.of("x", DriveConstants.trackWidthX / 2, "y", DriveConstants.trackWidthY / 2);
    final Map<String, Double> backLeft =
        Map.of("x", -DriveConstants.trackWidthX / 2, "y", DriveConstants.trackWidthY / 2);
    final Map<String, Double> bumper =
        Map.of(
            "front", DriveConstants.fullApothemY,
            "side", DriveConstants.fullApothemX,
            "back", DriveConstants.fullApothemY);
    final double maxRotorVelocity =
        DriveConstants.maxTrajectoryLinearSpeed
            / DriveConstants.trajectoryWheelRadius
            * DriveConstants.driveReduction;
    final double maxRotorTorque =
        DriveConstants.maxTrajectoryWheelTorque / DriveConstants.driveReduction;

    // Assemble .chor file to establish trajectory generation variables
    RobotConfig robotConfig =
        RobotConfig.builder()
            .frontLeft(
                ModuleConfig.builder()
                    .x(new ExpVal(frontLeft.get("x") + " m", frontLeft.get("x")))
                    .y(new ExpVal(frontLeft.get("y") + " m", frontLeft.get("y")))
                    .build())
            .backLeft(
                ModuleConfig.builder()
                    .x(new ExpVal(backLeft.get("x") + " m", backLeft.get("x")))
                    .y(new ExpVal(backLeft.get("y") + " m", backLeft.get("y")))
                    .build())
            .mass(new ExpVal(DriveConstants.mass + " kg", DriveConstants.mass))
            .inertia(
                new ExpVal(DriveConstants.rotationMOI + " kg m ^ 2", DriveConstants.rotationMOI))
            .gearing(
                new ExpVal(
                    Double.toString(DriveConstants.driveReduction), DriveConstants.driveReduction))
            .radius(
                new ExpVal(
                    DriveConstants.trajectoryWheelRadius + " m",
                    DriveConstants.trajectoryWheelRadius))
            .vmax(new ExpVal(maxRotorVelocity + " rad / s", maxRotorVelocity))
            .tmax(new ExpVal(maxRotorTorque + " N * m", maxRotorTorque))
            .cof(new ExpVal(Double.toString(DriveConstants.wheelCOF), DriveConstants.wheelCOF))
            .bumper(
                BumperConfig.builder()
                    .front(new ExpVal(bumper.get("front") + " m", bumper.get("front")))
                    .side(new ExpVal(bumper.get("side") + " m", bumper.get("side")))
                    .back(new ExpVal(bumper.get("back") + " m", bumper.get("back")))
                    .build())
            .differentialTrackWidth(
                new ExpVal(DriveConstants.trackWidthY + " m", DriveConstants.trackWidthY))
            .build();

    File chorFile = Path.of("src", "main", "deploy", "vts", "VTS.chor").toFile();
    if (!chorFile.exists()
        || readTrajHash(chorFile, "VTS") == null
        || !readTrajHash(chorFile, "VTS").equals(getHashCode(robotConfig, null))) {
      System.out.println("VTS Choreo Project: Not found or not up-to-date: Generating 🔄");
      ChorFileRoot chorRoot =
          ChorFileRoot.builder()
              .name("VTS")
              .version(2)
              .hashcode(getHashCode(robotConfig, null))
              .type("Swerve")
              .variables(new Variable(Map.of(), Map.of()))
              .config(robotConfig)
              .generationFeatures(List.of())
              .codegen(new Codegen(null, true, true, true))
              .build();

      try {
        mapper.writeValue(chorFile, chorRoot);
        System.out.println("VTS Choreo Project: Generated successfully ✅");
      } catch (IOException e) {
        System.out.println("VTS Choreo Project: FAILED to generate 😬");
        e.printStackTrace();
      }
    } else {
      System.out.println("VTS Choreo Project: Up-to-date 👍");
    }

    // Check Choreo CLI, download if needed
    if (!ChoreoLauncher.checkChoreoCLI()) {
      System.exit(1);
    }

    // Generate trajectories using parallel streams
    Set<String> completedPaths = ConcurrentHashMap.newKeySet();
    DriveTrajectories.paths.entrySet().parallelStream()
        .forEach(
            entry -> {
              String name = entry.getKey();
              PathRequest request = entry.getValue();

              // Check hashcodes
              File trajFile = Path.of("src", "main", "deploy", "vts", name + ".traj").toFile();
              try {
                if (trajFile.exists()
                    && trajFile.getCanonicalFile().getName().equals(trajFile.getName())
                    && readTrajHash(trajFile, name) != null
                    && readTrajHash(trajFile, name).equals(getHashCode(robotConfig, request))) {
                  completedPaths.add(name);
                  return;
                }
              } catch (IOException e) {
                // File does not exist, generate
              }

              // Delete old file
              if (trajFile.exists()) {
                trajFile.delete();
              }

              // Print status
              double startTime = System.currentTimeMillis();

              // Create list of constraints from segments
              int n_waypoint = 0;
              List<Constraint> constraints = new ArrayList<>();

              // Check for stopAtStart
              if (request.stopAtStart) {
                constraints.add(
                    Constraint.builder()
                        .from("first")
                        .to(null)
                        .data(Map.of("type", "StopPoint", "props", Map.of()))
                        .enabled(true)
                        .build());
              }

              // Check for stopAtEnd
              if (request.stopAtEnd) {
                constraints.add(
                    Constraint.builder()
                        .from("last")
                        .to(null)
                        .data(Map.of("type", "StopPoint", "props", Map.of()))
                        .enabled(true)
                        .build());
              }

              for (int i = 0; i < request.segments.size(); i++) {
                PathRequestSegment segment = request.segments.get(i);

                // Check for waypoints that are marked as stop points
                for (int j = 0; j < segment.waypoints.size(); j++) {
                  if (segment.waypoints.get(j).stopped) {
                    constraints.add(
                        Constraint.builder()
                            .from(n_waypoint + j)
                            .to(null)
                            .data(Map.of("type", "StopPoint", "props", Map.of()))
                            .enabled(true)
                            .build());
                  }
                }
                // Check for max velocity
                if (segment.maxVelocity >= 0) {
                  constraints.add(
                      Constraint.builder()
                          .from(Math.max(0, n_waypoint - 1))
                          .to(n_waypoint + segment.waypoints.size() - 1)
                          .data(
                              Map.of(
                                  "type",
                                  "MaxVelocity",
                                  "props",
                                  Map.of(
                                      "max",
                                      Map.of(
                                          "exp",
                                          segment.maxVelocity + " m / s",
                                          "val",
                                          segment.maxVelocity))))
                          .enabled(true)
                          .build());
                }
                // Check for max acceleration
                if (segment.maxAcceleration >= 0) {
                  constraints.add(
                      Constraint.builder()
                          .from(Math.max(0, n_waypoint - 1))
                          .to(n_waypoint + segment.waypoints.size() - 1)
                          .data(
                              Map.of(
                                  "type",
                                  "MaxAcceleration",
                                  "props",
                                  Map.of(
                                      "max",
                                      Map.of(
                                          "exp",
                                          segment.maxAcceleration + " m / s ^ 2",
                                          "val",
                                          segment.maxAcceleration))))
                          .enabled(true)
                          .build());
                }
                // Check for max angular velocity
                if (segment.maxAngularVelocity >= 0) {
                  constraints.add(
                      Constraint.builder()
                          .from(Math.max(0, n_waypoint - 1))
                          .to(n_waypoint + segment.waypoints.size() - 1)
                          .data(
                              Map.of(
                                  "type",
                                  "MaxAngularVelocity",
                                  "props",
                                  Map.of(
                                      "max",
                                      Map.of(
                                          "exp",
                                          segment.maxAngularVelocity + " rad / s",
                                          "val",
                                          segment.maxAngularVelocity))))
                          .enabled(true)
                          .build());
                }

                // Check for point at
                if (segment.pointAt != null) {
                  constraints.add(
                      Constraint.builder()
                          .from(Math.max(0, n_waypoint - 1))
                          .to(n_waypoint + segment.waypoints.size() - 1)
                          .data(
                              Map.of(
                                  "type",
                                  "PointAt",
                                  "props",
                                  Map.of(
                                      "x",
                                      new ExpVal(
                                          segment.pointAt.target().getX() + "m",
                                          segment.pointAt.target().getX()),
                                      "y",
                                      new ExpVal(
                                          segment.pointAt.target().getY() + "m",
                                          segment.pointAt.target().getY()),
                                      "tolerance",
                                      new ExpVal(
                                          segment.pointAt.tolerance().getRadians() + " rad",
                                          segment.pointAt.tolerance().getRadians()),
                                      "flip",
                                      segment.pointAt.flip())))
                          .enabled(true)
                          .build());
                }

                // Check for keep in lane
                if (segment.keepInLaneWidth > 0) {
                  constraints.add(
                      Constraint.builder()
                          .from(Math.max(0, n_waypoint - 1))
                          .to(n_waypoint + segment.waypoints.size() - 1)
                          .data(
                              Map.of(
                                  "type",
                                  "KeepInLane",
                                  "props",
                                  Map.of(
                                      "tolerance",
                                      new ExpVal(
                                          segment.keepInLaneWidth + " m",
                                          segment.keepInLaneWidth))))
                          .enabled(true)
                          .build());
                }

                // Check for keep out circles
                if (segment.keepOutCircles.size() > 0) {
                  for (int n_circle = 0; n_circle < segment.keepOutCircles.size(); n_circle++) {
                    constraints.add(
                        Constraint.builder()
                            .from(Math.max(0, n_waypoint - 1))
                            .to(n_waypoint + segment.waypoints.size() - 1)
                            .data(
                                Map.of(
                                    "type",
                                    "KeepOutCircle",
                                    "props",
                                    Map.of(
                                        "x",
                                            new ExpVal(
                                                segment.keepOutCircles.get(n_circle).center().getX()
                                                    + "m",
                                                segment
                                                    .keepOutCircles
                                                    .get(n_circle)
                                                    .center()
                                                    .getX()),
                                        "y",
                                            new ExpVal(
                                                segment.keepOutCircles.get(n_circle).center().getY()
                                                    + "m",
                                                segment
                                                    .keepOutCircles
                                                    .get(n_circle)
                                                    .center()
                                                    .getY()),
                                        "r",
                                            new ExpVal(
                                                segment.keepOutCircles.get(n_circle).radius()
                                                    + "  ",
                                                segment.keepOutCircles.get(n_circle).radius()))))
                            .enabled(true)
                            .build());
                  }
                }

                n_waypoint += segment.waypoints.size();
              }

              // Build root of request
              TrajFileRoot trajRoot =
                  TrajFileRoot.builder()
                      .name(name)
                      .version(2)
                      .snapshot(new Snapshot(List.of(), List.of(), request.targetDt))
                      .params(
                          Params.builder()
                              .waypoints(
                                  request.segments.stream()
                                      .flatMap(segment -> segment.waypoints.stream())
                                      .map(
                                          pathWaypoint -> {
                                            return Waypoint.builder()
                                                .x(
                                                    new ExpVal(
                                                        pathWaypoint.translation.getX() + " m",
                                                        pathWaypoint.translation.getX()))
                                                .y(
                                                    new ExpVal(
                                                        pathWaypoint.translation.getY() + " m",
                                                        pathWaypoint.translation.getY()))
                                                .heading(
                                                    pathWaypoint.rotation == null
                                                        ? new ExpVal("0 rad", 0)
                                                        : new ExpVal(
                                                            pathWaypoint.rotation.getRadians()
                                                                + " rad",
                                                            pathWaypoint.rotation.getRadians()))
                                                .intervals(
                                                    pathWaypoint.intervals > 0
                                                        ? pathWaypoint.intervals
                                                        : 40)
                                                .split(false)
                                                .fixTranslation(true)
                                                .fixHeading(pathWaypoint.rotation != null)
                                                .overrideIntervals(pathWaypoint.intervals >= 0)
                                                .build();
                                          })
                                      .toList())
                              .constraints(constraints)
                              .targetDt(new ExpVal(request.targetDt + " s", request.targetDt))
                              .build())
                      .trajectory(
                          new TrajectorySection(null, null, List.of(), List.of(), List.of()))
                      .events(List.of())
                      .build();

              try {
                mapper.writeValue(trajFile, trajRoot);
              } catch (IOException e) {
                System.out.println(name + ": JSON Creation FAILED 😬");
                e.printStackTrace();
                return;
              }

              System.out.println(name + ": Generating 🔄");

              try {
                ChoreoLauncher.generateTrajectory(chorFile, name);
              } catch (Exception e) {
                System.out.println(
                    name
                        + ": Choreo Generation FAILED 😬 (Try generating the trajectory in Choreo for details).");
                System.exit(1);
              }

              try {
                JsonNode trajWithHash = mapper.readTree(trajFile);
                ((ObjectNode) trajWithHash).put("hashcode", getHashCode(robotConfig, request));
                mapper.writer().writeValue(trajFile, trajWithHash);
              } catch (IOException e) {
                System.out.println(name + ": Hashcode Addition FAILED 😬");
                e.printStackTrace();
              }

              completedPaths.add(name);
              double endTime = System.currentTimeMillis();
              System.out.println(
                  name
                      + ": Generated successfully in "
                      + Math.round((endTime - startTime) / 100.0) / 10.0
                      + " secs ✅");
            });

    // Delete old trajectories
    try {
      Files.list(Path.of("src", "main", "deploy", "vts"))
          .forEach(
              (path) -> {
                String filename = path.getFileName().toString();
                if (!filename.endsWith(".traj") && !filename.endsWith(".chor")
                    || filename.startsWith("manual_")) return;
                String[] components = filename.split("\\.");
                if (components.length == 2
                    && !completedPaths.contains(components[0])
                    && !filename.equals("VTS.chor")) {
                  path.toFile().delete();
                  System.out.println(components[0] + ": Deleted 🫡");
                }
              });
    } catch (IOException e) {
      e.printStackTrace();
    }
    System.out.println("All trajectories up-to-date! 👍");
  }

  private static String readTrajHash(File pathFile, String name) {
    try {
      JsonNode hashcode = mapper.readTree(pathFile).get("hashcode");
      return hashcode.asText();
    } catch (Exception e) {
      System.out.println(name + " Hashcode not found: Regenerating 🔄");
    }
    return null;
  }

  private static String getHashCode(RobotConfig robotConfig, PathRequest request) {
    StringBuilder hashString = new StringBuilder();

    DecimalFormat format = new DecimalFormat("#.000000");
    format.setRoundingMode(RoundingMode.HALF_DOWN);

    // Add Choreo version
    hashString.append(ChoreoLauncher.choreoVersion);

    // Add drive config to hashString
    hashString.append(format.format(robotConfig.frontLeft().x().val()));
    hashString.append(format.format(robotConfig.frontLeft().y().val()));
    hashString.append(format.format(robotConfig.backLeft().x().val()));
    hashString.append(format.format(robotConfig.backLeft().y().val()));
    hashString.append(format.format(robotConfig.inertia().val()));
    hashString.append(format.format(robotConfig.gearing().val()));
    hashString.append(format.format(robotConfig.radius().val()));
    hashString.append(format.format(robotConfig.vmax().val()));
    hashString.append(format.format(robotConfig.tmax().val()));
    hashString.append(format.format(robotConfig.cof().val()));
    hashString.append(format.format(robotConfig.bumper().front().val()));
    hashString.append(format.format(robotConfig.bumper().side().val()));
    hashString.append(format.format(robotConfig.bumper().back().val()));
    hashString.append(format.format(robotConfig.differentialTrackWidth().val()));

    if (request == null) {
      return Hashing.sha256().hashString(hashString, StandardCharsets.UTF_8).toString();
    }

    // Add all aspects of the request to hashString
    hashString.append(request.stopAtStart);
    hashString.append(request.stopAtEnd);
    hashString.append(request.targetDt);

    // Add all aspects of each segment to hashString
    for (PathRequestSegment segment : request.segments) {
      for (PathWaypoint waypoint : segment.waypoints) {
        hashString.append(format.format(waypoint.translation.getX()));
        hashString.append(format.format(waypoint.translation.getY()));
        if (waypoint.rotation != null) {
          hashString.append(format.format(waypoint.rotation.getDegrees()));
        }

        if (waypoint.intervals > 0) {
          hashString.append(waypoint.intervals);
        }

        if (waypoint.stopped) {
          hashString.append(waypoint.stopped);
        }
      }

      if (segment.maxVelocity >= 0) {
        hashString.append(format.format(segment.maxVelocity));
      }

      if (segment.maxAcceleration >= 0) {
        hashString.append(format.format(segment.maxAcceleration));
      }

      if (segment.maxAngularVelocity >= 0) {
        hashString.append(format.format(segment.maxAngularVelocity));
      }

      if (segment.pointAt != null) {
        hashString.append(format.format(segment.pointAt.target().getX()));
        hashString.append(format.format(segment.pointAt.target().getY()));
        hashString.append(format.format(segment.pointAt.tolerance().getDegrees()));
        hashString.append(segment.pointAt.flip());
      }

      if (segment.keepInLaneWidth > 0) {
        hashString.append(format.format(segment.keepInLaneWidth));
      }

      if (segment.keepOutCircles.size() > 0) {
        for (int n_circle = 0; n_circle < segment.keepOutCircles.size(); n_circle++) {
          hashString.append(format.format(segment.keepOutCircles.get(n_circle).center().getX()));
          hashString.append(format.format(segment.keepOutCircles.get(n_circle).center().getY()));
          hashString.append(format.format(segment.keepOutCircles.get(n_circle).radius()));
        }
      }
    }

    return Hashing.sha256().hashString(hashString, StandardCharsets.UTF_8).toString();
  }
}
