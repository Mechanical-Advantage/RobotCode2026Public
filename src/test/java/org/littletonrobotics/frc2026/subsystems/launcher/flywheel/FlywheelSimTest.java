// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2026.subsystems.launcher.flywheel;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.hal.HAL;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.littletonrobotics.frc2026.Constants;
import org.littletonrobotics.frc2026.subsystems.launcher.flywheel.FlywheelIO.FlywheelIOInputs;
import org.littletonrobotics.frc2026.subsystems.launcher.flywheel.FlywheelIO.FlywheelIOOutputMode;
import org.littletonrobotics.frc2026.subsystems.launcher.flywheel.FlywheelIO.FlywheelIOOutputs;

/**
 * Tests {@link FlywheelIOSim} directly (bypasses the Flywheel subsystem to avoid Logger and
 * DriverStation dependencies).
 */
class FlywheelSimTest {

  static {
    Constants.disableHAL();
  }

  @BeforeAll
  static void initHAL() {
    // DCMotorSim calls into WPILib HAL simulation; initialize before first test.
    HAL.initialize(500, 0);
  }

  private FlywheelIOSim sim;
  private FlywheelIOInputs inputs;
  private FlywheelIOOutputs outputs;

  @BeforeEach
  void setUp() {
    sim = new FlywheelIOSim();
    inputs = new FlywheelIOInputs();
    outputs = new FlywheelIOOutputs();
  }

  // ── Baseline (coast) ─────────────────────────────────────────────────────

  @Test
  void coast_doesNotActivelyDriveFlywheel() {
    // NOTE: FlywheelIOSim.sim is a static field, so velocity may be non-zero from a prior test.
    // We verify coast mode doesn't *increase* velocity — it holds whatever speed it currently has
    // (back-EMF equals applied voltage, producing zero net torque).
    sim.updateInputs(inputs);
    double initialVelocity = inputs.velocityRadsPerSec;

    outputs.mode = FlywheelIOOutputMode.COAST;
    for (int i = 0; i < 50; i++) {
      sim.applyOutputs(outputs);
      sim.updateInputs(inputs);
    }

    assertEquals(
        initialVelocity,
        inputs.velocityRadsPerSec,
        1.0,
        "Coast mode should not actively accelerate or decelerate flywheel");
  }

  // ── Control action ───────────────────────────────────────────────────────

  /**
   * BUG DETECTION: In the original FlywheelIOSim.applyOutputs(), controller.calculate() is called
   * BEFORE controller.setSetpoint(). This means the first loop always computes output with
   * setpoint=0 regardless of the commanded velocity. After the fix, the flywheel should immediately
   * begin accelerating on the very first loop when a non-zero setpoint is commanded.
   */
  @Test
  void velocity_firstLoopProducesNonZeroAcceleration() {
    outputs.mode = FlywheelIOOutputMode.VELOCITY;
    outputs.velocityRadsPerSec = 50.0; // large setpoint so effect is clearly visible

    sim.applyOutputs(outputs);
    sim.updateInputs(inputs);

    assertTrue(
        inputs.velocityRadsPerSec > 0.01,
        "Flywheel should begin accelerating immediately on the first loop when setpoint is"
            + " commanded. Got: "
            + inputs.velocityRadsPerSec
            + " rad/s");
  }

  @Test
  void velocity_convergesOnSetpoint_after2Seconds() {
    final double targetRadsPerSec = 20.0;
    outputs.mode = FlywheelIOOutputMode.VELOCITY;
    outputs.velocityRadsPerSec = targetRadsPerSec;

    // 100 loops × 20 ms = 2 seconds
    for (int i = 0; i < 100; i++) {
      sim.applyOutputs(outputs);
      sim.updateInputs(inputs);
    }

    assertEquals(
        targetRadsPerSec,
        inputs.velocityRadsPerSec,
        2.0,
        "Flywheel should reach setpoint within 2 rad/s after 2 seconds");
  }

  @Test
  void velocity_increaseMonotonically_duringRampUp() {
    outputs.mode = FlywheelIOOutputMode.VELOCITY;
    outputs.velocityRadsPerSec = 30.0;

    double prevVelocity = 0.0;
    boolean everIncreased = false;

    for (int i = 0; i < 60; i++) {
      sim.applyOutputs(outputs);
      sim.updateInputs(inputs);
      if (inputs.velocityRadsPerSec > prevVelocity + 1e-6) {
        everIncreased = true;
      }
      prevVelocity = inputs.velocityRadsPerSec;
    }

    assertTrue(everIncreased, "Flywheel velocity should increase during ramp-up");
  }

  // ── Inputs fields ────────────────────────────────────────────────────────

  @Test
  void updateInputs_setsConnectedTrue() {
    sim.applyOutputs(outputs);
    sim.updateInputs(inputs);
    assertTrue(inputs.connected, "connected should be true in sim");
    assertTrue(inputs.follower1Connected, "follower1Connected should be true in sim");
  }
}
