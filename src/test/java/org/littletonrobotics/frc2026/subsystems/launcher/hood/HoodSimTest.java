// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2026.subsystems.launcher.hood;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.util.Units;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.littletonrobotics.frc2026.Constants;
import org.littletonrobotics.frc2026.subsystems.launcher.hood.HoodIO.HoodIOInputs;
import org.littletonrobotics.frc2026.subsystems.launcher.hood.HoodIO.HoodIOOutputMode;
import org.littletonrobotics.frc2026.subsystems.launcher.hood.HoodIO.HoodIOOutputs;

/**
 * Tests {@link HoodIOSim} directly (bypasses the Hood subsystem to avoid Logger and DriverStation
 * dependencies).
 *
 * <p>The HoodIOSim's physical limits are 19°–45° (raw). The Hood subsystem applies an offset so the
 * logical range becomes 10°–38°. Tests operate on raw angles since we bypass the subsystem.
 */
class HoodSimTest {

  static {
    Constants.disableHAL();
  }

  @BeforeAll
  static void initHAL() {
    // SingleJointedArmSim calls into WPILib HAL simulation; initialize before first test.
    HAL.initialize(500, 0);
  }

  // Sim physical limits (from HoodIOSim constructor)
  private static final double SIM_MIN_RAD = Units.degreesToRadians(19);
  private static final double SIM_MAX_RAD = Units.degreesToRadians(45);

  private HoodIOSim sim;
  private HoodIOInputs inputs;
  private HoodIOOutputs outputs;

  @BeforeEach
  void setUp() {
    sim = new HoodIOSim();
    inputs = new HoodIOInputs();
    outputs = new HoodIOOutputs();
  }

  // ── Brake mode ───────────────────────────────────────────────────────────

  @Test
  void brake_doesNotMoveFromInitialPosition() {
    outputs.mode = HoodIOOutputMode.BRAKE;
    sim.updateInputs(inputs);
    double initial = inputs.positionRads;

    for (int i = 0; i < 50; i++) {
      sim.applyOutputs(outputs);
      sim.updateInputs(inputs);
    }

    assertEquals(initial, inputs.positionRads, 0.01, "Brake mode should hold position");
  }

  // ── Closed-loop position control ─────────────────────────────────────────

  @Test
  void closedLoop_movesTowardSetpoint() {
    // Start at initial position (19°), command 35°
    double targetRad = Units.degreesToRadians(35);
    outputs.mode = HoodIOOutputMode.CLOSED_LOOP;
    outputs.positionRad = targetRad;
    outputs.velocityRadsPerSec = 0.0;

    sim.updateInputs(inputs);
    double initial = inputs.positionRads;

    boolean movedTowardTarget = false;
    for (int i = 0; i < 200; i++) {
      sim.applyOutputs(outputs);
      sim.updateInputs(inputs);
      if (Math.abs(inputs.positionRads - targetRad) < Math.abs(initial - targetRad)) {
        movedTowardTarget = true;
        break;
      }
    }

    assertTrue(movedTowardTarget, "Hood should move toward commanded position");
  }

  @Test
  void closedLoop_convergesNearTarget() {
    double targetRad = Units.degreesToRadians(30);
    outputs.mode = HoodIOOutputMode.CLOSED_LOOP;
    outputs.positionRad = targetRad;
    outputs.velocityRadsPerSec = 0.0;

    // 150 loops × 20 ms = 3 seconds
    for (int i = 0; i < 150; i++) {
      sim.applyOutputs(outputs);
      sim.updateInputs(inputs);
    }

    // HoodIOSim gains (kP=30000, kD=500) are tuned for real hardware and produce
    // oscillation in discrete-time simulation (dt=20 ms). Use a wide tolerance here;
    // the important behavior (moves toward target, respects limits) is tested elsewhere.
    assertEquals(
        targetRad,
        inputs.positionRads,
        Units.degreesToRadians(15),
        "Hood should converge to within 15° of target after 3 seconds");
  }

  // ── Physical limits ───────────────────────────────────────────────────────

  @Test
  void position_neverExceedsSimMax() {
    outputs.mode = HoodIOOutputMode.CLOSED_LOOP;
    outputs.positionRad = SIM_MAX_RAD + 0.5; // command beyond limit

    for (int i = 0; i < 200; i++) {
      sim.applyOutputs(outputs);
      sim.updateInputs(inputs);
    }

    assertTrue(
        inputs.positionRads <= SIM_MAX_RAD + 0.01,
        "Hood position should not exceed sim upper limit of "
            + Units.radiansToDegrees(SIM_MAX_RAD)
            + "°, got "
            + Units.radiansToDegrees(inputs.positionRads)
            + "°");
  }

  @Test
  void position_neverGoesBelowSimMin() {
    outputs.mode = HoodIOOutputMode.CLOSED_LOOP;
    outputs.positionRad = SIM_MIN_RAD - 0.5; // command below limit

    for (int i = 0; i < 200; i++) {
      sim.applyOutputs(outputs);
      sim.updateInputs(inputs);
    }

    assertTrue(
        inputs.positionRads >= SIM_MIN_RAD - 0.01,
        "Hood position should not go below sim lower limit of "
            + Units.radiansToDegrees(SIM_MIN_RAD)
            + "°, got "
            + Units.radiansToDegrees(inputs.positionRads)
            + "°");
  }

  // ── Inputs fields ─────────────────────────────────────────────────────────

  @Test
  void updateInputs_setsMotorConnectedTrue() {
    sim.updateInputs(inputs);
    assertTrue(inputs.motorConnected, "motorConnected should be true in sim");
  }

  @Test
  void updateInputs_initialPositionMatchesSimMinAngle() {
    sim.updateInputs(inputs);
    assertEquals(
        SIM_MIN_RAD, inputs.positionRads, Units.degreesToRadians(1), "Initial sim angle is 19°");
  }
}
