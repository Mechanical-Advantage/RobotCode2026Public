// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2026.subsystems.rollers;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.Alert;
import lombok.Setter;
import org.littletonrobotics.frc2026.Robot;
import org.littletonrobotics.frc2026.subsystems.rollers.RollerSystemIO.RollerSystemIOOutputs;
import org.littletonrobotics.frc2026.util.FullSubsystem;
import org.littletonrobotics.frc2026.util.LoggedTracer;
import org.littletonrobotics.junction.Logger;

public class RollerSystem extends FullSubsystem {
  private final String name;
  private final String inputsName;
  private final RollerSystemIO io;
  protected final RollerSystemIOInputsAutoLogged inputs = new RollerSystemIOInputsAutoLogged();
  private final RollerSystemIOOutputs outputs = new RollerSystemIOOutputs();

  private final Debouncer motorConnectedDebouncer =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final Alert disconnected;

  @Setter private double volts = 0.0;
  @Setter private boolean brakeModeEnabled = true;

  public RollerSystem(String name, String inputsName, RollerSystemIO io) {
    this.name = name;
    this.inputsName = inputsName;
    this.io = io;

    disconnected = new Alert(name + " motor disconnected!", Alert.AlertType.kWarning);
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(inputsName, inputs);
    disconnected.set(
        Robot.showHardwareAlerts() && !motorConnectedDebouncer.calculate(inputs.connected));

    // Run roller
    outputs.appliedVoltage = volts;

    // Update brake mode
    outputs.brakeModeEnabled = brakeModeEnabled;

    // Record cycle time
    LoggedTracer.record(name);

    Logger.recordOutput(inputsName + "/BrakeModeEnabled", brakeModeEnabled);
  }

  @Override
  public void periodicAfterScheduler() {
    io.applyOutputs(outputs);
  }

  public double getTorqueCurrent() {
    return inputs.torqueCurrentAmps;
  }

  public double getVelocity() {
    return inputs.velocityRadsPerSec;
  }

  public void stop() {
    volts = 0.0;
  }
}
