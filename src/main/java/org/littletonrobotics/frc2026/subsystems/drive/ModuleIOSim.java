// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2026.subsystems.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import org.littletonrobotics.frc2026.Constants;

/**
 * Physics sim implementation of module IO. The sim models are configured using a set of module
 * constants from Phoenix. Simulation is always based on voltage control.
 */
public class ModuleIOSim implements ModuleIO {
  private static final DCMotor driveMotorModel = DCMotor.getKrakenX60Foc(1);
  private static final DCMotor turnMotorModel = DCMotor.getKrakenX44Foc(1);

  private final DCMotorSim driveSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(driveMotorModel, 0.025, DriveConstants.driveReduction),
          driveMotorModel);
  private final DCMotorSim turnSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(turnMotorModel, 0.004, DriveConstants.turnReduction),
          turnMotorModel);

  private boolean driveClosedLoop = false;
  private boolean turnClosedLoop = false;
  private PIDController driveController = new PIDController(0.1, 0, 0, Constants.loopPeriodSecs);
  private PIDController turnController = new PIDController(10.0, 0, 0, Constants.loopPeriodSecs);
  private double driveFFVolts = 0;
  private double driveAppliedVolts = 0.0;
  private double turnAppliedVolts = 0.0;

  public ModuleIOSim() {
    // Enable wrapping for turn PID
    turnController.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    // Run closed-loop control
    if (driveClosedLoop) {
      driveAppliedVolts =
          driveFFVolts + driveController.calculate(driveSim.getAngularVelocityRadPerSec());
    } else {
      driveController.reset();
    }
    if (turnClosedLoop) {
      turnAppliedVolts = turnController.calculate(turnSim.getAngularPositionRad());
    } else {
      turnController.reset();
    }

    // Update simulation state
    driveSim.setInputVoltage(MathUtil.clamp(driveAppliedVolts, -12.0, 12.0));
    turnSim.setInputVoltage(MathUtil.clamp(turnAppliedVolts, -12.0, 12.0));
    driveSim.update(Constants.loopPeriodSecs);
    turnSim.update(Constants.loopPeriodSecs);

    inputs.driveConnected = true;
    inputs.drivePositionRad = driveSim.getAngularPositionRad();
    inputs.driveVelocityRadPerSec = driveSim.getAngularVelocityRadPerSec();
    inputs.driveAppliedVolts = driveAppliedVolts;
    inputs.driveSupplyCurrentAmps = Math.abs(driveSim.getCurrentDrawAmps());

    inputs.turnConnected = true;
    inputs.turnPosition = new Rotation2d(turnSim.getAngularPosition());
    inputs.turnAbsolutePosition = new Rotation2d(turnSim.getAngularPosition());
    inputs.turnSupplyCurrentAmps = Math.abs(turnSim.getCurrentDrawAmps());
  }

  @Override
  public void applyOutputs(ModuleIOOutputs outputs) {
    switch (outputs.mode) {
      case COAST, BRAKE -> {
        driveClosedLoop = false;
        turnClosedLoop = false;
        driveAppliedVolts = 0.0;
        turnAppliedVolts = 0.0;
      }
      case DRIVE -> {
        driveClosedLoop = true;
        turnClosedLoop = true;
        driveFFVolts = outputs.driveFeedforward;
        driveController.setSetpoint(outputs.driveVelocityRadPerSec);
        turnController.setSetpoint(outputs.turnRotation.getRadians());
      }
      case CHARACTERIZE -> {
        driveClosedLoop = false;
        turnClosedLoop = true;
        driveFFVolts = outputs.driveCharacterizationOutput;
        turnController.setSetpoint(outputs.turnRotation.getRadians());
      }
    }
  }
}
