// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2026.subsystems.hood;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import org.littletonrobotics.frc2026.Constants;

public class HoodIOSim implements HoodIO {
  private static final DCMotor motorModel = DCMotor.getKrakenX44(1);
  private final SingleJointedArmSim sim =
      new SingleJointedArmSim(
          motorModel, 1.0, .004, .33, 0.0, Units.degreesToRadians(45), false, 0, null);

  private double currentOutput = 0.0;
  private double appliedVolts = 0.0;
  private boolean currentControl = false;

  public HoodIOSim() {}

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    if (currentControl) {
      appliedVolts = motorModel.getVoltage(currentOutput, sim.getVelocityRadPerSec());
    } else {
      appliedVolts = 0.0;
    }

    // Update sim state
    sim.setInputVoltage(MathUtil.clamp(appliedVolts, -12.0, 12.0));
    sim.update(Constants.loopPeriodSecs);

    inputs.motorConnected = true;
    inputs.positionRads = sim.getAngleRads();
    inputs.velocityRadsPerSec = sim.getVelocityRadPerSec();
    inputs.appliedVolts = appliedVolts;
    inputs.supplyCurrentAmps = sim.getCurrentDrawAmps();
    inputs.torqueCurrentAmps = currentOutput;
    inputs.tempCelsius = 0.0;
  }

  @Override
  public void applyOutputs(HoodIOOutputs outputs) {
    switch (outputs.mode) {
      case BRAKE -> {
        currentControl = false;
      }
      case COAST -> {
        currentOutput = 0.0;
        currentControl = true;
      }
      case CLOSED_LOOP -> {
        currentOutput =
            outputs.feedforward
                + (sim.getAngleRads() - outputs.positionRad) * outputs.kP
                + (sim.getVelocityRadPerSec() - outputs.velocityRadsPerSec) * outputs.kD;
        currentControl = true;
      }
    }
  }
}
