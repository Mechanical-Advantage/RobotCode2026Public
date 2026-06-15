// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2026.util;

// https://github.com/hammerheads5000/FuelSim
//
// Copyright (c) 2026 LordOfFrogs
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import java.util.ArrayList;
import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import java.util.stream.Collectors;
import org.littletonrobotics.frc2026.Constants;
import org.littletonrobotics.junction.Logger;

public class FuelSim {
  protected static final double PERIOD = Constants.loopPeriodSecs; // sec
  protected static final Translation3d GRAVITY = new Translation3d(0, 0, -9.81); // m/s^2
  // Room temperature dry air density: https://en.wikipedia.org/wiki/Density_of_air#Dry_air
  protected static final double AIR_DENSITY = 1.2041; // kg/m^3
  protected static final double FIELD_COR =
      Math.sqrt(22 / 51.5); // coefficient of restitution with the field
  protected static final double FUEL_COR = 0.5; // coefficient of restitution with another fuel
  protected static final double NET_COR = 0.2; // coefficient of restitution with the net
  protected static final double ROBOT_COR = 0.1; // coefficient of restitution with a robot
  protected static final double FUEL_RADIUS = 0.075;
  protected static final double FIELD_LENGTH = 16.51;
  protected static final double FIELD_WIDTH = 8.04;
  protected static final double TRENCH_WIDTH = 1.265;
  protected static final double TRENCH_BLOCK_WIDTH = 0.305;
  protected static final double TRENCH_HEIGHT = 0.565;
  protected static final double TRENCH_BAR_HEIGHT = 0.102;
  protected static final double TRENCH_BAR_WIDTH = 0.152;
  protected static final double FRICTION =
      0.1; // proportion of horizontal vel to lose per sec while on ground
  protected static final double FUEL_MASS = 0.448 * 0.45392; // kgs
  protected static final double FUEL_CROSS_AREA = Math.PI * FUEL_RADIUS * FUEL_RADIUS;
  // Drag coefficient of smooth sphere:
  // https://en.wikipedia.org/wiki/Drag_coefficient#/media/File:14ilf1l.svg
  protected static final double DRAG_COF = 0.47; // dimensionless
  protected static final double DRAG_FORCE_FACTOR = 0.5 * AIR_DENSITY * DRAG_COF * FUEL_CROSS_AREA;

  protected static final Translation3d[] FIELD_XZ_LINE_STARTS = {
    new Translation3d(0, 0, 0),
    new Translation3d(3.96, 1.57, 0),
    new Translation3d(3.96, FIELD_WIDTH / 2 + 0.60, 0),
    new Translation3d(4.61, 1.57, 0.165),
    new Translation3d(4.61, FIELD_WIDTH / 2 + 0.60, 0.165),
    new Translation3d(FIELD_LENGTH - 5.18, 1.57, 0),
    new Translation3d(FIELD_LENGTH - 5.18, FIELD_WIDTH / 2 + 0.60, 0),
    new Translation3d(FIELD_LENGTH - 4.61, 1.57, 0.165),
    new Translation3d(FIELD_LENGTH - 4.61, FIELD_WIDTH / 2 + 0.60, 0.165),
    new Translation3d(3.96, TRENCH_WIDTH, TRENCH_HEIGHT),
    new Translation3d(3.96, FIELD_WIDTH - 1.57, TRENCH_HEIGHT),
    new Translation3d(FIELD_LENGTH - 5.18, TRENCH_WIDTH, TRENCH_HEIGHT),
    new Translation3d(FIELD_LENGTH - 5.18, FIELD_WIDTH - 1.57, TRENCH_HEIGHT),
    new Translation3d(4.61 - TRENCH_BAR_WIDTH / 2, 0, TRENCH_HEIGHT + TRENCH_BAR_HEIGHT),
    new Translation3d(
        4.61 - TRENCH_BAR_WIDTH / 2, FIELD_WIDTH - 1.57, TRENCH_HEIGHT + TRENCH_BAR_HEIGHT),
    new Translation3d(
        FIELD_LENGTH - 4.61 - TRENCH_BAR_WIDTH / 2, 0, TRENCH_HEIGHT + TRENCH_BAR_HEIGHT),
    new Translation3d(
        FIELD_LENGTH - 4.61 - TRENCH_BAR_WIDTH / 2,
        FIELD_WIDTH - 1.57,
        TRENCH_HEIGHT + TRENCH_BAR_HEIGHT),
  };

  protected static final Translation3d[] FIELD_XZ_LINE_ENDS = {
    new Translation3d(FIELD_LENGTH, FIELD_WIDTH, 0),
    new Translation3d(4.61, FIELD_WIDTH / 2 - 0.60, 0.165),
    new Translation3d(4.61, FIELD_WIDTH - 1.57, 0.165),
    new Translation3d(5.18, FIELD_WIDTH / 2 - 0.60, 0),
    new Translation3d(5.18, FIELD_WIDTH - 1.57, 0),
    new Translation3d(FIELD_LENGTH - 4.61, FIELD_WIDTH / 2 - 0.60, 0.165),
    new Translation3d(FIELD_LENGTH - 4.61, FIELD_WIDTH - 1.57, 0.165),
    new Translation3d(FIELD_LENGTH - 3.96, FIELD_WIDTH / 2 - 0.60, 0),
    new Translation3d(FIELD_LENGTH - 3.96, FIELD_WIDTH - 1.57, 0),
    new Translation3d(5.18, TRENCH_WIDTH + TRENCH_BLOCK_WIDTH, TRENCH_HEIGHT),
    new Translation3d(5.18, FIELD_WIDTH - 1.57 + TRENCH_BLOCK_WIDTH, TRENCH_HEIGHT),
    new Translation3d(FIELD_LENGTH - 3.96, TRENCH_WIDTH + TRENCH_BLOCK_WIDTH, TRENCH_HEIGHT),
    new Translation3d(FIELD_LENGTH - 3.96, FIELD_WIDTH - 1.57 + TRENCH_BLOCK_WIDTH, TRENCH_HEIGHT),
    new Translation3d(
        4.61 + TRENCH_BAR_WIDTH / 2,
        TRENCH_WIDTH + TRENCH_BLOCK_WIDTH,
        TRENCH_HEIGHT + TRENCH_BAR_HEIGHT),
    new Translation3d(4.61 + TRENCH_BAR_WIDTH / 2, FIELD_WIDTH, TRENCH_HEIGHT + TRENCH_BAR_HEIGHT),
    new Translation3d(
        FIELD_LENGTH - 4.61 + TRENCH_BAR_WIDTH / 2,
        TRENCH_WIDTH + TRENCH_BLOCK_WIDTH,
        TRENCH_HEIGHT + TRENCH_BAR_HEIGHT),
    new Translation3d(
        FIELD_LENGTH - 4.61 + TRENCH_BAR_WIDTH / 2, FIELD_WIDTH, TRENCH_HEIGHT + TRENCH_BAR_HEIGHT),
  };

  protected static class Fuel {
    protected Translation3d pos;
    protected Translation3d vel;

    protected Fuel(Translation3d pos, Translation3d vel) {
      this.pos = pos;
      this.vel = vel;
    }

    protected Fuel(Translation3d pos) {
      this(pos, new Translation3d());
    }

    protected void update(boolean simulateAirResistance, int subticks) {
      pos = pos.plus(vel.times(PERIOD / subticks));
      if (pos.getZ() > FUEL_RADIUS) {
        Translation3d Fg = GRAVITY.times(FUEL_MASS);
        Translation3d Fd = new Translation3d();

        if (simulateAirResistance) {
          double speed = vel.getNorm();
          if (speed > 1e-6) {
            Fd = vel.times(-DRAG_FORCE_FACTOR * speed);
          }
        }

        Translation3d accel = Fg.plus(Fd).div(FUEL_MASS);
        vel = vel.plus(accel.times(PERIOD / subticks));
      }
      if (Math.abs(vel.getZ()) < 0.05 && pos.getZ() <= FUEL_RADIUS + 0.03) {
        vel = new Translation3d(vel.getX(), vel.getY(), 0);
        vel = vel.times(1 - FRICTION * PERIOD / subticks);
        // pos = new Translation3d(pos.getX(), pos.getY(), FUEL_RADIUS);
      }
      handleFieldCollisions(subticks);
    }

    protected void handleXZLineCollision(Translation3d lineStart, Translation3d lineEnd) {
      if (pos.getY() < lineStart.getY() || pos.getY() > lineEnd.getY())
        return; // not within y range
      // Convert into 2D
      Translation2d start2d = new Translation2d(lineStart.getX(), lineStart.getZ());
      Translation2d end2d = new Translation2d(lineEnd.getX(), lineEnd.getZ());
      Translation2d pos2d = new Translation2d(pos.getX(), pos.getZ());
      Translation2d lineVec = end2d.minus(start2d);

      // Get closest point on line
      Translation2d projected =
          start2d.plus(lineVec.times(pos2d.minus(start2d).dot(lineVec) / lineVec.getSquaredNorm()));

      if (projected.getDistance(start2d) + projected.getDistance(end2d) > lineVec.getNorm())
        return; // projected point not on line
      double dist = pos2d.getDistance(projected);
      if (dist > FUEL_RADIUS) return; // not intersecting line
      // Back into 3D
      Translation3d normal =
          new Translation3d(-lineVec.getY(), 0, lineVec.getX()).div(lineVec.getNorm());

      // Apply collision response
      pos = pos.plus(normal.times(FUEL_RADIUS - dist));
      if (vel.dot(normal) > 0) return; // already moving away from line
      vel = vel.minus(normal.times((1 + FIELD_COR) * vel.dot(normal)));
    }

    protected void handleFieldCollisions(int subticks) {
      // floor and bumps
      for (int i = 0; i < FIELD_XZ_LINE_STARTS.length; i++) {
        handleXZLineCollision(FIELD_XZ_LINE_STARTS[i], FIELD_XZ_LINE_ENDS[i]);
      }

      // edges
      if (pos.getX() < FUEL_RADIUS && vel.getX() < 0) {
        pos = pos.plus(new Translation3d(FUEL_RADIUS - pos.getX(), 0, 0));
        vel = vel.plus(new Translation3d(-(1 + FIELD_COR) * vel.getX(), 0, 0));
      } else if (pos.getX() > FIELD_LENGTH - FUEL_RADIUS && vel.getX() > 0) {
        pos = pos.plus(new Translation3d(FIELD_LENGTH - FUEL_RADIUS - pos.getX(), 0, 0));
        vel = vel.plus(new Translation3d(-(1 + FIELD_COR) * vel.getX(), 0, 0));
      }

      if (pos.getY() < FUEL_RADIUS && vel.getY() < 0) {
        pos = pos.plus(new Translation3d(0, FUEL_RADIUS - pos.getY(), 0));
        vel = vel.plus(new Translation3d(0, -(1 + FIELD_COR) * vel.getY(), 0));
      } else if (pos.getY() > FIELD_WIDTH - FUEL_RADIUS && vel.getY() > 0) {
        pos = pos.plus(new Translation3d(0, FIELD_WIDTH - FUEL_RADIUS - pos.getY(), 0));
        vel = vel.plus(new Translation3d(0, -(1 + FIELD_COR) * vel.getY(), 0));
      }

      // hubs
      handleHubCollisions(Hub.BLUE_HUB, subticks);
      handleHubCollisions(Hub.RED_HUB, subticks);

      handleTrenchCollisions();
    }

    protected void handleHubCollisions(Hub hub, int subticks) {
      hub.handleHubInteraction(this, subticks);
      hub.fuelCollideSide(this);

      double netCollision = hub.fuelHitNet(this);
      if (netCollision != 0) {
        pos = pos.plus(new Translation3d(netCollision, 0, 0));
        vel = new Translation3d(-vel.getX() * NET_COR, vel.getY() * NET_COR, vel.getZ());
      }
    }

    protected void handleTrenchCollisions() {
      fuelCollideRectangle(
          this,
          new Translation3d(3.96, TRENCH_WIDTH, 0),
          new Translation3d(5.18, TRENCH_WIDTH + TRENCH_BLOCK_WIDTH, TRENCH_HEIGHT));
      fuelCollideRectangle(
          this,
          new Translation3d(3.96, FIELD_WIDTH - 1.57, 0),
          new Translation3d(5.18, FIELD_WIDTH - 1.57 + TRENCH_BLOCK_WIDTH, TRENCH_HEIGHT));
      fuelCollideRectangle(
          this,
          new Translation3d(FIELD_LENGTH - 5.18, TRENCH_WIDTH, 0),
          new Translation3d(FIELD_LENGTH - 3.96, TRENCH_WIDTH + TRENCH_BLOCK_WIDTH, TRENCH_HEIGHT));
      fuelCollideRectangle(
          this,
          new Translation3d(FIELD_LENGTH - 5.18, FIELD_WIDTH - 1.57, 0),
          new Translation3d(
              FIELD_LENGTH - 3.96, FIELD_WIDTH - 1.57 + TRENCH_BLOCK_WIDTH, TRENCH_HEIGHT));
      fuelCollideRectangle(
          this,
          new Translation3d(4.61 - TRENCH_BAR_WIDTH / 2, 0, TRENCH_HEIGHT),
          new Translation3d(
              4.61 + TRENCH_BAR_WIDTH / 2,
              TRENCH_WIDTH + TRENCH_BLOCK_WIDTH,
              TRENCH_HEIGHT + TRENCH_BAR_HEIGHT));
      fuelCollideRectangle(
          this,
          new Translation3d(4.61 - TRENCH_BAR_WIDTH / 2, FIELD_WIDTH - 1.57, TRENCH_HEIGHT),
          new Translation3d(
              4.61 + TRENCH_BAR_WIDTH / 2, FIELD_WIDTH, TRENCH_HEIGHT + TRENCH_BAR_HEIGHT));
      fuelCollideRectangle(
          this,
          new Translation3d(FIELD_LENGTH - 4.61 - TRENCH_BAR_WIDTH / 2, 0, TRENCH_HEIGHT),
          new Translation3d(
              FIELD_LENGTH - 4.61 + TRENCH_BAR_WIDTH / 2,
              TRENCH_WIDTH + TRENCH_BLOCK_WIDTH,
              TRENCH_HEIGHT + TRENCH_BAR_HEIGHT));
      fuelCollideRectangle(
          this,
          new Translation3d(
              FIELD_LENGTH - 4.61 - TRENCH_BAR_WIDTH / 2, FIELD_WIDTH - 1.57, TRENCH_HEIGHT),
          new Translation3d(
              FIELD_LENGTH - 4.61 + TRENCH_BAR_WIDTH / 2,
              FIELD_WIDTH,
              TRENCH_HEIGHT + TRENCH_BAR_HEIGHT));
    }

    protected void addImpulse(Translation3d impulse) {
      vel = vel.plus(impulse);
    }
  }

  protected static void handleFuelCollision(Fuel a, Fuel b) {
    Translation3d normal = a.pos.minus(b.pos);
    double distance = normal.getNorm();
    if (distance == 0) {
      normal = new Translation3d(1, 0, 0);
      distance = 1;
    }
    normal = normal.div(distance);
    double impulse = 0.5 * (1 + FUEL_COR) * (b.vel.minus(a.vel).dot(normal));
    double intersection = FUEL_RADIUS * 2 - distance;
    a.pos = a.pos.plus(normal.times(intersection / 2));
    b.pos = b.pos.minus(normal.times(intersection / 2));
    a.addImpulse(normal.times(impulse));
    b.addImpulse(normal.times(-impulse));
  }

  protected static final double CELL_SIZE = 0.25;
  protected static final int GRID_COLS = (int) Math.ceil(FIELD_LENGTH / CELL_SIZE);
  protected static final int GRID_ROWS = (int) Math.ceil(FIELD_WIDTH / CELL_SIZE);

  @SuppressWarnings("unchecked")
  protected final ArrayList<Fuel>[][] grid = new ArrayList[GRID_COLS][GRID_ROWS];

  private final ArrayList<ArrayList<Fuel>> activeCells = new ArrayList<>();

  protected void handleFuelCollisions(ArrayList<Fuel> fuels) {
    // Clear grid
    for (ArrayList<Fuel> cell : activeCells) {
      cell.clear();
    }
    activeCells.clear();

    // Populate grid
    for (Fuel fuel : fuels) {
      int col = (int) (fuel.pos.getX() / CELL_SIZE);
      int row = (int) (fuel.pos.getY() / CELL_SIZE);

      if (col >= 0 && col < GRID_COLS && row >= 0 && row < GRID_ROWS) {
        grid[col][row].add(fuel);
        if (grid[col][row].size() == 1) {
          activeCells.add(grid[col][row]);
        }
      }
    }

    // Check collisions
    for (Fuel fuel : fuels) {
      int col = (int) (fuel.pos.getX() / CELL_SIZE);
      int row = (int) (fuel.pos.getY() / CELL_SIZE);

      // Check 3x3 neighbor cells
      for (int i = col - 1; i <= col + 1; i++) {
        for (int j = row - 1; j <= row + 1; j++) {
          if (i >= 0 && i < GRID_COLS && j >= 0 && j < GRID_ROWS) {
            for (Fuel other : grid[i][j]) {
              if (fuel != other && fuel.pos.getDistance(other.pos) < FUEL_RADIUS * 2) {
                if (fuel.hashCode() < other.hashCode()) {
                  handleFuelCollision(fuel, other);
                }
              }
            }
          }
        }
      }
    }
  }

  protected ArrayList<Fuel> fuels = new ArrayList<>();
  protected boolean running = false;
  protected boolean simulateAirResistance = false;
  protected ArrayList<SimRobot> robots = new ArrayList<>();
  protected int subticks = 5;

  /**
   * Creates a new instance of FuelSim
   *
   * @param tableKey NetworkTable to log fuel positions to as an array of {@link Translation3d}
   *     structs.
   */
  public FuelSim(String tableKey) {
    // Initialize grid
    for (int i = 0; i < GRID_COLS; i++) {
      for (int j = 0; j < GRID_ROWS; j++) {
        grid[i][j] = new ArrayList<Fuel>();
      }
    }
  }

  /** Creates a new instance of FuelSim with log path "/Fuel Simulation" */
  public FuelSim() {
    this("/Fuel Simulation");
  }

  /** Clears the field of fuel */
  public void clearFuel() {
    fuels.clear();
  }

  /** Spawns fuel in the neutral zone and depots */
  public void spawnStartingFuel() {
    // Center fuel
    Translation3d center = new Translation3d(FIELD_LENGTH / 2, FIELD_WIDTH / 2, FUEL_RADIUS);
    for (int i = 0; i < 15; i++) {
      for (int j = 0; j < 6; j++) {
        fuels.add(
            new Fuel(
                center.plus(new Translation3d(0.076 + 0.152 * j, 0.0254 + 0.076 + 0.152 * i, 0))));
        fuels.add(
            new Fuel(
                center.plus(new Translation3d(-0.076 - 0.152 * j, 0.0254 + 0.076 + 0.152 * i, 0))));
        fuels.add(
            new Fuel(
                center.plus(new Translation3d(0.076 + 0.152 * j, -0.0254 - 0.076 - 0.152 * i, 0))));
        fuels.add(
            new Fuel(
                center.plus(
                    new Translation3d(-0.076 - 0.152 * j, -0.0254 - 0.076 - 0.152 * i, 0))));
      }
    }

    // Depots
    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 4; j++) {
        fuels.add(
            new Fuel(new Translation3d(0.076 + 0.152 * j, 5.95 + 0.076 + 0.152 * i, FUEL_RADIUS)));
        fuels.add(
            new Fuel(new Translation3d(0.076 + 0.152 * j, 5.95 - 0.076 - 0.152 * i, FUEL_RADIUS)));
        fuels.add(
            new Fuel(
                new Translation3d(
                    FIELD_LENGTH - 0.076 - 0.152 * j, 2.09 + 0.076 + 0.152 * i, FUEL_RADIUS)));
        fuels.add(
            new Fuel(
                new Translation3d(
                    FIELD_LENGTH - 0.076 - 0.152 * j, 2.09 - 0.076 - 0.152 * i, FUEL_RADIUS)));
      }
    }
  }

  /** Adds array of `Translation3d`'s to NetworkTables at tableKey + "/Fuels" */
  public void logFuels() {
    Logger.recordOutput(
        "FuelSim/FuelPoses",
        fuels.stream().map(f -> new Pose3d(f.pos, Rotation3d.kZero)).toArray(Pose3d[]::new));
  }

  /** Returns the list of fuels in the simulation */
  public Set<Translation2d> getFuels() {
    return fuels.stream().map((fuel) -> fuel.pos.toTranslation2d()).collect(Collectors.toSet());
  }

  /** Start the simulation. `updateSim` must still be called every loop */
  public void start() {
    running = true;
  }

  /** Pause the simulation. */
  public void stop() {
    running = false;
  }

  /** Enables accounting for drag force in physics step * */
  public void enableAirResistance() {
    simulateAirResistance = true;
  }

  /**
   * Sets the number of physics iterations per loop (0.02s)
   *
   * @param subticks
   */
  public void setSubticks(int subticks) {
    this.subticks = subticks;
  }

  /** Clears all robots from the simulation except the first registered robot. */
  public void clearAdditionalRobots() {
    if (robots.size() > 1) {
      robots.subList(1, robots.size()).clear();
    }
  }

  /**
   * Registers a robot with the fuel simulator
   *
   * @param width from left to right (y-axis)
   * @param length from front to back (x-axis)
   * @param bumperHeight
   * @param poseSupplier
   * @param fieldSpeedsSupplier field-relative `ChassisSpeeds` supplier
   */
  public SimRobot registerRobot(
      double width,
      double length,
      double bumperHeight,
      Supplier<Pose2d> poseSupplier,
      Supplier<ChassisSpeeds> fieldSpeedsSupplier) {
    SimRobot robot = new SimRobot(poseSupplier, fieldSpeedsSupplier, width, length, bumperHeight);
    robots.add(robot);
    return robot;
  }

  /** To be called periodically Will do nothing if sim is not running */
  public void updateSim() {
    if (!running) return;
    stepSim();
  }

  /** Run the simulation forward 1 time step (0.02s) */
  public void stepSim() {
    for (int i = 0; i < subticks; i++) {
      for (Fuel fuel : fuels) {
        fuel.update(this.simulateAirResistance, this.subticks);
      }

      handleFuelCollisions(fuels);

      if (robots.size() != 0) {
        handleRobotCollisions(fuels);
        handleIntakes(fuels);
      }
    }

    logFuels();
  }

  /**
   * Adds a fuel onto the field
   *
   * @param pos Position to spawn at
   * @param vel Initial velocity vector
   */
  public void spawnFuel(Translation3d pos, Translation3d vel) {
    fuels.add(new Fuel(pos, vel));
  }

  /**
   * Spawns a fuel onto the field with a specified launch velocity and angles, accounting for robot
   * movement
   *
   * @param launchVelocity Initial launch velocity
   * @param hoodAngle Hood angle where 0 is launching horizontally and 90 degrees is launching
   *     straight up
   * @param turretYaw <i>Robot-relative</i> turret yaw
   * @param launchHeight Height of the fuel to launch at. Make sure this is higher than your robot's
   *     bumper height, or else it will collide with your robot immediately.
   * @throws IllegalStateException if robot is not registered
   */
  public void launchFuel(
      SimRobot robot,
      LinearVelocity launchVelocity,
      Angle hoodAngle,
      Angle turretYaw,
      Distance launchHeight) {
    if (robot.poseSupplier == null || robot.fieldSpeedsSupplier == null) {
      throw new IllegalStateException("Robot must be registered before launching fuel.");
    }

    Pose3d launchPose =
        new Pose3d(robot.poseSupplier.get())
            .plus(
                new Transform3d(
                    new Translation3d(Meters.zero(), Meters.zero(), launchHeight),
                    Rotation3d.kZero));
    ChassisSpeeds fieldSpeeds = robot.fieldSpeedsSupplier.get();

    double horizontalVel = Math.cos(hoodAngle.in(Radians)) * launchVelocity.in(MetersPerSecond);
    double verticalVel = Math.sin(hoodAngle.in(Radians)) * launchVelocity.in(MetersPerSecond);
    double xVel =
        horizontalVel
            * Math.cos(turretYaw.plus(launchPose.getRotation().getMeasureZ()).in(Radians));
    double yVel =
        horizontalVel
            * Math.sin(turretYaw.plus(launchPose.getRotation().getMeasureZ()).in(Radians));

    xVel += fieldSpeeds.vxMetersPerSecond;
    yVel += fieldSpeeds.vyMetersPerSecond;

    spawnFuel(launchPose.getTranslation(), new Translation3d(xVel, yVel, verticalVel));
  }

  protected void handleRobotCollision(Fuel fuel, SimRobot robot, Translation2d robotVel) {
    Translation2d relativePos =
        new Pose2d(fuel.pos.toTranslation2d(), Rotation2d.kZero)
            .relativeTo(robot.poseSupplier.get())
            .getTranslation();

    if (fuel.pos.getZ() > robot.bumperHeight) return; // above bumpers
    double distanceToBottom = -FUEL_RADIUS - robot.length / 2 - relativePos.getX();
    double distanceToTop = -FUEL_RADIUS - robot.length / 2 + relativePos.getX();
    double distanceToRight = -FUEL_RADIUS - robot.width / 2 - relativePos.getY();
    double distanceToLeft = -FUEL_RADIUS - robot.width / 2 + relativePos.getY();

    // not inside robot
    if (distanceToBottom > 0 || distanceToTop > 0 || distanceToRight > 0 || distanceToLeft > 0)
      return;

    Translation2d posOffset;
    // find minimum distance to side and send corresponding collision response
    if ((distanceToBottom >= distanceToTop
        && distanceToBottom >= distanceToRight
        && distanceToBottom >= distanceToLeft)) {
      posOffset = new Translation2d(distanceToBottom, 0);
    } else if ((distanceToTop >= distanceToBottom
        && distanceToTop >= distanceToRight
        && distanceToTop >= distanceToLeft)) {
      posOffset = new Translation2d(-distanceToTop, 0);
    } else if ((distanceToRight >= distanceToBottom
        && distanceToRight >= distanceToTop
        && distanceToRight >= distanceToLeft)) {
      posOffset = new Translation2d(0, distanceToRight);
    } else {
      posOffset = new Translation2d(0, -distanceToLeft);
    }

    posOffset = posOffset.rotateBy(robot.poseSupplier.get().getRotation());
    fuel.pos = fuel.pos.plus(new Translation3d(posOffset));
    Translation2d normal = posOffset.div(posOffset.getNorm());
    if (fuel.vel.toTranslation2d().dot(normal) < 0)
      fuel.addImpulse(
          new Translation3d(
              normal.times(-fuel.vel.toTranslation2d().dot(normal) * (1 + ROBOT_COR))));
    if (robotVel.dot(normal) > 0)
      fuel.addImpulse(new Translation3d(normal.times(robotVel.dot(normal))));
  }

  protected void handleRobotCollisions(ArrayList<Fuel> fuels) {
    for (SimRobot robot : robots) {
      ChassisSpeeds speeds = robot.fieldSpeedsSupplier.get();
      for (Fuel fuel : fuels) {
        handleRobotCollision(
            fuel, robot, new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond));
      }
    }
  }

  protected void handleIntakes(ArrayList<Fuel> fuels) {
    for (SimRobot robot : robots) {
      for (int i = 0; i < fuels.size(); i++) {
        if (robot.intake != null && robot.intake.shouldIntake(fuels.get(i), robot)) {
          fuels.remove(i);
          i--;
        }
      }
    }
  }

  protected static void fuelCollideRectangle(Fuel fuel, Translation3d start, Translation3d end) {
    if (fuel.pos.getZ() > end.getZ() + FUEL_RADIUS || fuel.pos.getZ() < start.getZ() - FUEL_RADIUS)
      return; // above rectangle
    double distanceToLeft = start.getX() - FUEL_RADIUS - fuel.pos.getX();
    double distanceToRight = fuel.pos.getX() - end.getX() - FUEL_RADIUS;
    double distanceToTop = fuel.pos.getY() - end.getY() - FUEL_RADIUS;
    double distanceToBottom = start.getY() - FUEL_RADIUS - fuel.pos.getY();

    // not inside hub
    if (distanceToLeft > 0 || distanceToRight > 0 || distanceToTop > 0 || distanceToBottom > 0)
      return;

    Translation2d collision;
    // find minimum distance to side and send corresponding collision response
    if (fuel.pos.getX() < start.getX()
        || (distanceToLeft >= distanceToRight
            && distanceToLeft >= distanceToTop
            && distanceToLeft >= distanceToBottom)) {
      collision = new Translation2d(distanceToLeft, 0);
    } else if (fuel.pos.getX() >= end.getX()
        || (distanceToRight >= distanceToLeft
            && distanceToRight >= distanceToTop
            && distanceToRight >= distanceToBottom)) {
      collision = new Translation2d(-distanceToRight, 0);
    } else if (fuel.pos.getY() > end.getY()
        || (distanceToTop >= distanceToLeft
            && distanceToTop >= distanceToRight
            && distanceToTop >= distanceToBottom)) {
      collision = new Translation2d(0, -distanceToTop);
    } else {
      collision = new Translation2d(0, distanceToBottom);
    }

    if (collision.getX() != 0) {
      fuel.pos = fuel.pos.plus(new Translation3d(collision));
      fuel.vel = fuel.vel.plus(new Translation3d(-(1 + FIELD_COR) * fuel.vel.getX(), 0, 0));
    } else if (collision.getY() != 0) {
      fuel.pos = fuel.pos.plus(new Translation3d(collision));
      fuel.vel = fuel.vel.plus(new Translation3d(0, -(1 + FIELD_COR) * fuel.vel.getY(), 0));
    }
  }

  public static class Hub {
    public static final Hub BLUE_HUB =
        new Hub(
            new Translation2d(4.61, FIELD_WIDTH / 2),
            new Translation3d(5.3, FIELD_WIDTH / 2, 0.89),
            1);
    public static final Hub RED_HUB =
        new Hub(
            new Translation2d(FIELD_LENGTH - 4.61, FIELD_WIDTH / 2),
            new Translation3d(FIELD_LENGTH - 5.3, FIELD_WIDTH / 2, 0.89),
            -1);

    protected static final double ENTRY_HEIGHT = 1.83;
    protected static final double ENTRY_RADIUS = 0.56;

    protected static final double SIDE = 1.2;

    protected static final double NET_HEIGHT_MAX = 3.057;
    protected static final double NET_HEIGHT_MIN = 1.5;
    protected static final double NET_OFFSET = SIDE / 2 + 0.261;
    protected static final double NET_WIDTH = 1.484;

    protected final Translation2d center;
    protected final Translation3d exit;
    protected final int exitVelXMult;

    protected int score = 0;

    protected Hub(Translation2d center, Translation3d exit, int exitVelXMult) {
      this.center = center;
      this.exit = exit;
      this.exitVelXMult = exitVelXMult;
    }

    protected void handleHubInteraction(Fuel fuel, int subticks) {
      if (didFuelScore(fuel, subticks)) {
        fuel.pos = exit;
        fuel.vel = getDispersalVelocity();
        score++;
      }
    }

    protected boolean didFuelScore(Fuel fuel, int subticks) {
      return fuel.pos.toTranslation2d().getDistance(center) <= ENTRY_RADIUS
          && fuel.pos.getZ() <= ENTRY_HEIGHT
          && fuel.pos.minus(fuel.vel.times(PERIOD / subticks)).getZ() > ENTRY_HEIGHT;
    }

    protected Translation3d getDispersalVelocity() {
      return new Translation3d(
          exitVelXMult * (Math.random() + 0.1) * 1.5, Math.random() * 2 - 1, 0);
    }

    /** Reset this hub's score to 0 */
    public void resetScore() {
      score = 0;
    }

    /**
     * Get the current count of fuel scored in this hub
     *
     * @return
     */
    public int getScore() {
      return score;
    }

    protected void fuelCollideSide(Fuel fuel) {
      fuelCollideRectangle(
          fuel,
          new Translation3d(center.getX() - SIDE / 2, center.getY() - SIDE / 2, 0),
          new Translation3d(
              center.getX() + SIDE / 2, center.getY() + SIDE / 2, ENTRY_HEIGHT - 0.1));
    }

    protected double fuelHitNet(Fuel fuel) {
      if (fuel.pos.getZ() > NET_HEIGHT_MAX || fuel.pos.getZ() < NET_HEIGHT_MIN) return 0;
      if (fuel.pos.getY() > center.getY() + NET_WIDTH / 2
          || fuel.pos.getY() < center.getY() - NET_WIDTH / 2) return 0;
      if (fuel.pos.getX() > center.getX() + NET_OFFSET * exitVelXMult) {
        return Math.max(
            0, center.getX() + NET_OFFSET * exitVelXMult - (fuel.pos.getX() - FUEL_RADIUS));
      } else {
        return Math.min(
            0, center.getX() + NET_OFFSET * exitVelXMult - (fuel.pos.getX() + FUEL_RADIUS));
      }
    }
  }

  public class SimRobot {
    SimIntake intake;
    Supplier<Pose2d> poseSupplier;
    Supplier<ChassisSpeeds> fieldSpeedsSupplier;
    double width, length, bumperHeight;

    protected SimRobot(
        Supplier<Pose2d> poseSupplier,
        Supplier<ChassisSpeeds> fieldSpeedsSupplier,
        double width,
        double length,
        double bumperHeight) {

      this.poseSupplier = poseSupplier;
      this.fieldSpeedsSupplier = fieldSpeedsSupplier;
      this.width = width;
      this.length = length;
      this.bumperHeight = bumperHeight;
    }

    /**
     * Registers an intake with the fuel simulator. This intake will remove fuel from the field
     * based on the `ableToIntake` parameter.
     *
     * @param xMin Minimum x position for the bounding box
     * @param xMax Maximum x position for the bounding box
     * @param yMin Minimum y position for the bounding box
     * @param yMax Maximum y position for the bounding box
     * @param ableToIntake Should a return a boolean whether the intake is active
     * @param intakeCallback Function to call when a fuel is intaked
     */
    public void registerIntake(
        double xMin,
        double xMax,
        double yMin,
        double yMax,
        BooleanSupplier ableToIntake,
        Runnable intakeCallback) {
      intake = new SimIntake(xMin, xMax, yMin, yMax, ableToIntake, intakeCallback);
    }
  }

  public class SimIntake {
    double xMin, xMax, yMin, yMax;
    BooleanSupplier ableToIntake;
    Runnable callback;

    protected SimIntake(
        double xMin,
        double xMax,
        double yMin,
        double yMax,
        BooleanSupplier ableToIntake,
        Runnable intakeCallback) {
      this.xMin = xMin;
      this.xMax = xMax;
      this.yMin = yMin;
      this.yMax = yMax;
      this.ableToIntake = ableToIntake;
      this.callback = intakeCallback;
    }

    protected boolean shouldIntake(Fuel fuel, SimRobot robot) {
      if (!ableToIntake.getAsBoolean() || fuel.pos.getZ() > robot.bumperHeight) return false;

      Translation2d fuelRelativePos =
          new Pose2d(fuel.pos.toTranslation2d(), Rotation2d.kZero)
              .relativeTo(robot.poseSupplier.get())
              .getTranslation();

      boolean result =
          fuelRelativePos.getX() >= xMin
              && fuelRelativePos.getX() <= xMax
              && fuelRelativePos.getY() >= yMin
              && fuelRelativePos.getY() <= yMax;
      if (result) {
        callback.run();
      }
      return result;
    }
  }
}
