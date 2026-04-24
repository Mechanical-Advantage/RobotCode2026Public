// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2026.subsystems.slamtake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.function.BooleanSupplier;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.frc2026.Robot;
import org.littletonrobotics.frc2026.subsystems.slamtake.SlamIO.SlamIOOutputMode;
import org.littletonrobotics.frc2026.subsystems.slamtake.SlamIO.SlamIOOutputs;
import org.littletonrobotics.frc2026.util.LoggedTunableNumber;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Slam {
  private final SlamIO io;
  private final SlamIOInputsAutoLogged inputs = new SlamIOInputsAutoLogged();
  private final SlamIOOutputs outputs = new SlamIOOutputs();

  private static final LoggedTunableNumber kP = new LoggedTunableNumber("Slam/kP", 80.0);
  private static final LoggedTunableNumber kD = new LoggedTunableNumber("Slam/kD", 0.0);

  private static final LoggedTunableNumber toleranceDeg =
      new LoggedTunableNumber("Slam/ToleranceDeg", 4);

  private static final LoggedTunableNumber homingVolts =
      new LoggedTunableNumber("Slam/Homing/Volts", -4);
  private static final LoggedTunableNumber homingVelocityThreshold =
      new LoggedTunableNumber("Slam/Homing/VelocityThreshold", .05);

  // Connected debouncer
  private final Debouncer motorConnectedDebouncer =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final Alert motorDisconnectedAlert =
      new Alert("Slam motor disconnected!", Alert.AlertType.kError);

  public static final double slamMaxAngle = Units.degreesToRadians(145.8170667799);
  public static final double slamMinAngle =
      Units.degreesToRadians(17.4088627799); // Determined from CAD defined as angle to washer
  @AutoLogOutput private double goalAngle = 0.0;
  private static double slamOffset = 0.0;
  @Getter private boolean zeroed = false;

  @Setter private BooleanSupplier coastOverride = () -> false;

  public Slam(SlamIO io) {
    this.io = io;
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Slamtake/Slam", inputs);

    motorDisconnectedAlert.set(
        Robot.showHardwareAlerts() && !motorConnectedDebouncer.calculate(inputs.connected));

    Robot.batteryLogger.reportCurrentUsage(
        "Slam", false, inputs.connected ? inputs.supplyCurrentAmps : 0.0);

    if (kP.hasChanged(hashCode()) || kD.hasChanged(hashCode())) {
      outputs.kP = kP.get();
      outputs.kD = kD.get();
    }
  }

  public void periodicAfterScheduler() {

    // Stop when disabled
    if (DriverStation.isDisabled()) {
      outputs.mode = SlamIOOutputMode.BRAKE;

      if (coastOverride.getAsBoolean()) {
        outputs.mode = SlamIOOutputMode.COAST;
      }
    }

    io.applyOutputs(outputs);
  }

  public void runVolts(double volts) {
    outputs.mode = SlamIOOutputMode.RUN_OPEN_LOOP;
    outputs.appliedVolts = volts;
  }

  public void runPosition(double positionRads) {
    goalAngle = positionRads;
    outputs.mode = SlamIOOutputMode.RUN_CLOSED_LOOP;
    outputs.position = MathUtil.clamp(goalAngle, slamMinAngle, slamMaxAngle) - slamOffset;
  }

  @AutoLogOutput(key = "Slam/MeasuredAngleRads")
  public double getMeasuredAngleRad() {
    return inputs.positionRads + slamOffset;
  }

  @AutoLogOutput
  public boolean atGoal() {
    return DriverStation.isEnabled()
        && zeroed
        && Math.abs(getMeasuredAngleRad() - goalAngle)
            <= Units.degreesToRadians(toleranceDeg.get());
  }

  public void zeroMaxAngle() {
    slamOffset = slamMaxAngle - inputs.positionRads;
    zeroed = true;
  }

  private void zeroMinAngle() {
    slamOffset = slamMinAngle - inputs.positionRads;
    zeroed = true;
  }

  public Command zeroCommand() {
    return Commands.run(
            () -> {
              runVolts(homingVolts.get());
              zeroed = false;
            })
        .raceWith(
            Commands.waitSeconds(1.0)
                .andThen(
                    Commands.waitUntil(
                        () ->
                            Math.abs(inputs.velocityRadsPerSec) <= homingVelocityThreshold.get())))
        .andThen(this::zeroMinAngle);
  }

  public void setMode(SlamIOOutputMode mode) {
    outputs.mode = mode;
  }

  public double getVelocity() {
    return inputs.velocityRadsPerSec;
  }
}
