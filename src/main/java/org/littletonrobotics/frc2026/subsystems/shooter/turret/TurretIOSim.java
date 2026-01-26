// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2026.subsystems.shooter.turret;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import org.littletonrobotics.frc2026.Constants;

public class TurretIOSim implements TurretIO {
  private final DCMotor gearbox = DCMotor.getKrakenX44Foc(1);
  private final DCMotorSim sim =
      new DCMotorSim(LinearSystemId.createDCMotorSystem(gearbox, 0.001, 100.0), gearbox);

  private double currentOutput = 0.0;
  private double appliedVoltage = 0.0;
  private boolean currentControl = false;

  public TurretIOSim() {}

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    if (currentControl) {
      appliedVoltage = gearbox.getVoltage(currentOutput, sim.getAngularVelocityRadPerSec());
    }
    sim.setInputVoltage(MathUtil.clamp(appliedVoltage, -12.0, 12.0));
    sim.update(Constants.loopPeriodSecs);

    inputs.motorConnected = true;
    inputs.positionRads = sim.getAngularPositionRad();
    inputs.velocityRadsPerSec = sim.getAngularVelocityRadPerSec();
    inputs.appliedVolts = appliedVoltage;
    inputs.supplyCurrentAmps = sim.getCurrentDrawAmps();
    inputs.torqueCurrentAmps = currentOutput;
  }

  @Override
  public void applyOutputs(TurretIOOutputs outputs) {
    switch (outputs.mode) {
      case BRAKE -> {
        currentControl = false;
        appliedVoltage = 0.0;
      }
      case COAST -> {
        currentControl = true;
        currentOutput = 0.0;
      }
      case CLOSED_LOOP -> {
        currentControl = true;
        currentOutput =
            (outputs.position - sim.getAngularPositionRad()) * outputs.kP
                + (outputs.velocity - sim.getAngularVelocityRadPerSec()) * outputs.kD;
      }
    }
  }
}
