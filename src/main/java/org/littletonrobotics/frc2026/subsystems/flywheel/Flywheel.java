// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2026.subsystems.flywheel;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Alert;
import org.littletonrobotics.frc2026.Constants;
import org.littletonrobotics.frc2026.Robot;
import org.littletonrobotics.frc2026.subsystems.flywheel.FlywheelIO.FlywheelIOOutputs;
import org.littletonrobotics.frc2026.util.FullSubsystem;
import org.littletonrobotics.frc2026.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class Flywheel extends FullSubsystem {
  private final FlywheelIO io;
  private final FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();
  private final FlywheelIOOutputs outputs = new FlywheelIOOutputs();

  private final Debouncer motorConnectedDebouncer =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final Alert disconnected;

  public static final LoggedTunableNumber kS = new LoggedTunableNumber("Flywheel/kS");
  public static final LoggedTunableNumber kP = new LoggedTunableNumber("Flywheel/kP");
  public static final LoggedTunableNumber kD = new LoggedTunableNumber("Flywheel/kD");
  private static final LoggedTunableNumber rateLimiter =
      new LoggedTunableNumber("Flywheel/SlewRateLimiter");
  SlewRateLimiter slewRateLimiter = new SlewRateLimiter(rateLimiter.get());

  static {
    switch (Constants.robot) {
      case COMPBOT -> {
        rateLimiter.initDefault(100);
        kS.initDefault(0.0);
        kP.initDefault(0.0);
        kD.initDefault(0.0);
      }
      case ALPHABOT -> {
        rateLimiter.initDefault(100);
        kS.initDefault(0.0);
        kP.initDefault(15.0);
        kD.initDefault(0.0);
      }
      default -> {
        rateLimiter.initDefault(0.0);
        kS.initDefault(0.0);
        kP.initDefault(0.0);
        kD.initDefault(0.0);
      }
    }
  }

  public Flywheel(FlywheelIO io) {
    this.io = io;

    disconnected = new Alert("Flywheel motor disconnected!", Alert.AlertType.kWarning);
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Flywheel", inputs);

    if (rateLimiter.hasChanged(hashCode())) {
      slewRateLimiter = new SlewRateLimiter(rateLimiter.get());
    }
    outputs.kP = kP.get();
    outputs.kD = kD.get();

    if (outputs.coast) {
      slewRateLimiter.reset(inputs.velocityRadsPerSec);
    }

    disconnected.set(
        Robot.showHardwareAlerts() && !motorConnectedDebouncer.calculate(inputs.connected));
  }

  @Override
  public void periodicAfterScheduler() {
    io.applyOutputs(outputs);
  }

  /** Run closed loop at the specified velocity. */
  public void runVelocity(double velocityRadsPerSec) {
    outputs.coast = false;
    outputs.velocityRadsPerSec = slewRateLimiter.calculate(velocityRadsPerSec);
    outputs.feedForward = kS.get() * Math.signum(velocityRadsPerSec);

    // Log flywheel setpoint
    Logger.recordOutput("Flywheel/Setpoint", outputs.velocityRadsPerSec);
  }

  /** Stops the flywheel. */
  public void stop() {
    outputs.velocityRadsPerSec = 0.0;
    outputs.coast = true;
  }

  /** Returns the current velocity in RPM. */
  public double getVelocity() {
    return inputs.velocityRadsPerSec;
  }
}
