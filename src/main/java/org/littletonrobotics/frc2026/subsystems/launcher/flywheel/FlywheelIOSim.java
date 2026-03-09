// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2026.subsystems.launcher.flywheel;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import org.littletonrobotics.frc2026.Constants;

public class FlywheelIOSim implements FlywheelIO {
  private static final DCMotor motorModel = DCMotor.getKrakenX60(4);
  private static final DCMotorSim sim =
      new DCMotorSim(LinearSystemId.createDCMotorSystem(motorModel, .3, 1), motorModel);

  private PIDController controller = new PIDController(0.6, 0, 0, Constants.loopPeriodSecs);
  private double currentOutput = 0.0;
  private double currentOutputAsVolt = 0.0;
  private double appliedVolts = 0.0;
  private double lastVelocity = 0.0;

  public FlywheelIOSim() {}

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    currentOutputAsVolt =
        MathUtil.clamp(
            motorModel.getVoltage(currentOutput, sim.getAngularVelocityRadPerSec()), -12.0, 12.0);
    appliedVolts = currentOutputAsVolt;

    // Update sim state
    sim.setInputVoltage(MathUtil.clamp(appliedVolts, -12.0, 12.0));
    sim.update(Constants.loopPeriodSecs);

    lastVelocity = inputs.velocityRadsPerSec;
    inputs.connected = true;
    inputs.follower1Connected = true;
    inputs.follower2Connected = true;
    inputs.follower3Connected = true;
    inputs.positionRads = sim.getAngularPositionRad();
    inputs.velocityRadsPerSec = sim.getAngularVelocityRadPerSec();
    inputs.appliedVoltage = appliedVolts;
    inputs.supplyCurrentAmps = sim.getCurrentDrawAmps();
    inputs.follower1SupplyCurrentAmps = sim.getCurrentDrawAmps();
    inputs.follower2SupplyCurrentAmps = sim.getCurrentDrawAmps();
    inputs.follower3SupplyCurrentAmps = sim.getCurrentDrawAmps();
    inputs.torqueCurrentAmps = currentOutput;
    inputs.tempCelsius = 0.0;
  }

  @Override
  public void applyOutputs(FlywheelIOOutputs outputs) {
    if (outputs.mode == FlywheelIOOutputMode.COAST) {
      currentOutput = 0.0;
    } else {
      controller.setSetpoint(outputs.velocityRadsPerSec);
      currentOutput = controller.calculate(lastVelocity);
    }
  }
}
