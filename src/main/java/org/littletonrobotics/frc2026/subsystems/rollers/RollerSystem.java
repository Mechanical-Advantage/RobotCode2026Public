// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2026.subsystems.rollers;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.function.BooleanSupplier;
import lombok.Setter;
import org.littletonrobotics.frc2026.Robot;
import org.littletonrobotics.frc2026.energy.FinanceDepartment;
import org.littletonrobotics.frc2026.subsystems.rollers.RollerSystemIO.RollerSystemIOMode;
import org.littletonrobotics.frc2026.subsystems.rollers.RollerSystemIO.RollerSystemIOOutputs;
import org.littletonrobotics.junction.Logger;

public class RollerSystem {
  private final String name;
  private final String inputsName;
  private final RollerSystemIO io;
  protected final RollerSystemIOInputsAutoLogged inputs = new RollerSystemIOInputsAutoLogged();
  private final RollerSystemIOOutputs outputs = new RollerSystemIOOutputs();

  private double kS = 0.0;
  private double kV = 0.0;

  @Setter private BooleanSupplier coastOverride = () -> false;

  private final Debouncer motorConnectedDebouncer =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final Debouncer followerMotorConnectedDebouncer =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final Alert disconnected;
  private final Alert followerDisconnected;

  public RollerSystem(String name, String inputsName, RollerSystemIO io, double kP, double kD) {
    this.name = name;
    this.inputsName = inputsName;
    this.io = io;

    disconnected = new Alert(name + " motor disconnected!", Alert.AlertType.kError);
    followerDisconnected =
        new Alert(name + " follower motor disconnected!", Alert.AlertType.kError);
    outputs.kP = kP;
    outputs.kD = kD;
  }

  public RollerSystem(String name, String inputsName, RollerSystemIO io) {
    this.name = name;
    this.inputsName = inputsName;
    this.io = io;

    disconnected = new Alert(name + " motor disconnected!", Alert.AlertType.kError);
    followerDisconnected =
        new Alert(name + " follower motor disconnected!", Alert.AlertType.kError);
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(inputsName, inputs);
    disconnected.set(
        Robot.showHardwareAlerts() && !motorConnectedDebouncer.calculate(inputs.connected));
    followerDisconnected.set(
        Robot.showHardwareAlerts()
            && inputs.hasFollower
            && !followerMotorConnectedDebouncer.calculate(inputs.followerConnected));

    // Record energy usage
    if (inputs.hasFollower) {
      FinanceDepartment.getInstance()
          .reportCurrentUsage(
              name,
              false,
              inputs.connected ? inputs.supplyCurrentAmps : 0.0,
              inputs.followerConnected ? inputs.followerSupplyCurrentAmps : 0.0);
    } else {
      FinanceDepartment.getInstance()
          .reportCurrentUsage(name, false, inputs.connected ? inputs.supplyCurrentAmps : 0.0);
    }

    // Update mode
    if (DriverStation.isDisabled()) {
      outputs.mode = RollerSystemIOMode.BRAKE;

      if (coastOverride.getAsBoolean()) {
        outputs.mode = RollerSystemIOMode.COAST;
      }
    }
  }

  public void periodicAfterScheduler() {
    io.applyOutputs(outputs);
  }

  public void runOpenLoop(double volts) {
    outputs.mode = RollerSystemIOMode.VOLTAGE_CONTROL;
    outputs.appliedVoltage = volts;
  }

  public void runClosedLoop(double setpointVelocity) {
    outputs.mode = RollerSystemIOMode.CLOSED_LOOP;
    outputs.velocity = setpointVelocity;
    outputs.feedforward = Math.signum(setpointVelocity) * kS + setpointVelocity * kV;
  }

  public void setGains(double kP, double kD) {
    outputs.kP = kP;
    outputs.kD = kD;
  }

  public void setFeedforward(double kS, double kV) {
    this.kS = kS;
    this.kV = kV;
  }

  public double getTorqueCurrent() {
    return inputs.torqueCurrentAmps;
  }

  public double getVelocity() {
    return inputs.velocityRadsPerSec;
  }

  public void stop() {
    outputs.mode = RollerSystemIOMode.BRAKE;
  }
}
