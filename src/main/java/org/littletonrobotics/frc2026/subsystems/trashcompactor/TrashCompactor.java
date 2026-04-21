// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2026.subsystems.trashcompactor;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.function.BooleanSupplier;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.frc2026.Constants;
import org.littletonrobotics.frc2026.DarwinMechanism3d;
import org.littletonrobotics.frc2026.Robot;
import org.littletonrobotics.frc2026.subsystems.trashcompactor.TrashCompactorIO.TrashCompactorIOOutputMode;
import org.littletonrobotics.frc2026.subsystems.trashcompactor.TrashCompactorIO.TrashCompactorIOOutputs;
import org.littletonrobotics.frc2026.util.FullSubsystem;
import org.littletonrobotics.frc2026.util.LoggedTunableNumber;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class TrashCompactor extends FullSubsystem {
  private static final double sprocketRadius = Units.inchesToMeters(0.654195);
  private static final double tolerance = 0.003;
  public static final double minHeight = 0.554692;
  public static final double maxHeight = 0.764242;

  private static final LoggedTunableNumber kP =
      new LoggedTunableNumber("TrashCompactor/Profile/kP", 1000.0);
  private static final LoggedTunableNumber kD =
      new LoggedTunableNumber("TrashCompactor/Profile/kD", 10.0);
  private static final LoggedTunableNumber kS =
      new LoggedTunableNumber("TrashCompactor/Profile/kS", 30.0);
  private static final LoggedTunableNumber kG =
      new LoggedTunableNumber("TrashCompactor/Profile/kG", 15.0);

  private static final LoggedTunableNumber maxVelocity =
      new LoggedTunableNumber("TrashCompactor/Profile/MaxVelocity", 2.0);
  private static final LoggedTunableNumber maxAcceleration =
      new LoggedTunableNumber("TrashCompactor/Profile/MaxAcceleration", 15.0);

  private static final LoggedTunableNumber homingDownVolts =
      new LoggedTunableNumber("TrashCompactor/Homing/VoltsDown", -4.0);
  private static final LoggedTunableNumber homingUpVolts =
      new LoggedTunableNumber("TrashCompactor/Homing/VoltsUp", 4.0);
  private static final LoggedTunableNumber homingVelocityThreshold =
      new LoggedTunableNumber("TrashCompactor/Homing/VelocityThreshold", 0.02);

  private static final LoggedTunableNumber compactLaunchingOffset =
      new LoggedTunableNumber("TrashCompactor/LaunchingOffsetMeters", 0.15);
  private static final LoggedTunableNumber compactPassiveAmpsDown =
      new LoggedTunableNumber("TrashCompactor/PassiveAmpsDown", -20.0);
  private static final LoggedTunableNumber compactPassiveHelperAmpsDown =
      new LoggedTunableNumber("TrashCompactor/PassiveHelperAmpsDown", 0.0);
  private static final LoggedTunableNumber compactPassiveHelperTime =
      new LoggedTunableNumber("TrashCompactor/PassiveHelperTime", 0.0);

  private final TrashCompactorIO io;
  private final TrashCompactorIOInputsAutoLogged inputs = new TrashCompactorIOInputsAutoLogged();
  private final TrashCompactorIOOutputs outputs = new TrashCompactorIOOutputs();

  private TrapezoidProfile profile;

  // Connected debouncer
  private final Debouncer motorConnectedDebouncer =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final Alert motorDisconnectedAlert =
      new Alert("Trash compactor motor disconnected!", Alert.AlertType.kError);

  private Timer modeTimer = new Timer();

  private State goalState = new State(minHeight, 0.0);
  private State setpoint = new State(minHeight, 0.0);

  @AutoLogOutput
  private TrashCompactorCompactingMode compactingMode = TrashCompactorCompactingMode.IDLE;

  private double offset = 0.0;
  private double launchingPosition = maxHeight;
  @Getter private boolean zeroed = false;

  @Setter private BooleanSupplier coastOverride = () -> false;

  public TrashCompactor(TrashCompactorIO io) {
    this.io = io;
    profile =
        new TrapezoidProfile(
            new TrapezoidProfile.Constraints(maxVelocity.get(), maxAcceleration.get()));
    modeTimer.start();
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("TrashCompactor", inputs);

    motorDisconnectedAlert.set(
        Robot.showHardwareAlerts() && !motorConnectedDebouncer.calculate(inputs.connected));

    Robot.batteryLogger.reportCurrentUsage(
        "TrashCompactor", false, inputs.connected ? inputs.supplyCurrentAmps : 0.0);

    if (kP.hasChanged(hashCode()) || kD.hasChanged(hashCode())) {
      outputs.kP = kP.get();
      outputs.kD = kD.get();
    }
    if (maxVelocity.hasChanged(hashCode()) || maxAcceleration.hasChanged(hashCode())) {
      profile =
          new TrapezoidProfile(
              new TrapezoidProfile.Constraints(maxVelocity.get(), maxAcceleration.get()));
    }

    // Stop when disabled
    if (DriverStation.isDisabled()) {
      outputs.mode = TrashCompactorIOOutputMode.BRAKE;
      setpoint = new State(getMeasuredHeightMeters(), 0.0);

      if (coastOverride.getAsBoolean()) {
        outputs.mode = TrashCompactorIOOutputMode.COAST;
      }
    } else {
      switch (compactingMode) {
        case PASSIVE_DOWN -> {
          if (getMeasuredHeightMeters() > minHeight + tolerance) {
            if (!modeTimer.hasElapsed(compactPassiveHelperTime.get())) {
              runAmps(compactPassiveHelperAmpsDown.get());
            } else {
              runAmps(compactPassiveAmpsDown.get());
            }
          } else {
            // Prevent fuel from forcing up
            runVolts(0.0);
          }
          setpoint = new State(getMeasuredHeightMeters(), 0.0);
        }
        case FORCE_MAX -> {
          runPosition(maxHeight);
        }
        case FORCE_MIN -> {
          runPosition(minHeight);
        }
        case LAUNCHING -> {
          runPosition(launchingPosition);
        }
        case IDLE -> {
          runAmps(0.0);
          setpoint =
              new State(MathUtil.clamp(getMeasuredHeightMeters(), minHeight, maxHeight), 0.0);
        }
      }
    }

    SmartDashboard.putBoolean("Trash Compactor At Min Height", atMinHeight());

    // Visualize 3D
    DarwinMechanism3d.getMeasured().setTrashCompactorHeight(this.getMeasuredHeightMeters());
  }

  public void periodicAfterScheduler() {
    io.applyOutputs(outputs);
  }

  private void runVolts(double volts) {
    outputs.mode = TrashCompactorIOOutputMode.RUN_VOLTS;
    outputs.appliedVolts = volts;
  }

  private void runAmps(double amps) {
    outputs.mode = TrashCompactorIOOutputMode.RUN_AMPS;
    outputs.appliedAmps = amps;
  }

  private void runPosition(double positionMeters) {
    goalState = new State(positionMeters, 0.0);
    setpoint = profile.calculate(Constants.loopPeriodSecs, setpoint, goalState);

    outputs.mode = TrashCompactorIOOutputMode.RUN_POSITION;
    outputs.position =
        (MathUtil.clamp(setpoint.position, minHeight, maxHeight) - offset) / sprocketRadius;
    outputs.velocity = setpoint.velocity / sprocketRadius;
    outputs.feedforward = Math.signum(setpoint.velocity) * kS.get() + kG.get();

    Logger.recordOutput("TrashCompactor/GoalPositionRadians", goalState.position / sprocketRadius);
    Logger.recordOutput("TrashCompactor/SetpointPositionMeters", setpoint.position);
    Logger.recordOutput("TrashCompactor/SetpointVelocityMetersPerSec", setpoint.velocity);
  }

  @AutoLogOutput(key = "TrashCompactor/MeasuredHeightMeters")
  public double getMeasuredHeightMeters() {
    return inputs.positionRads * sprocketRadius + offset;
  }

  @AutoLogOutput(key = "TrashCompactor/MeasuredVelocityMetersPerSec")
  public double getMeasuredVelocityMetersPerSec() {
    return inputs.velocityRadsPerSec * sprocketRadius;
  }

  @AutoLogOutput(key = "TrashCompactor/AtMinHeight")
  public boolean atMinHeight() {
    return getMeasuredHeightMeters() < minHeight + tolerance;
  }

  public void zeroMinHeight() {
    offset = minHeight - inputs.positionRads * sprocketRadius;
    zeroed = true;
  }

  public void zeroMaxHeight() {
    offset = maxHeight - inputs.positionRads * sprocketRadius;
    zeroed = true;
  }

  public Command zeroMinCommand() {
    return Commands.runOnce(() -> zeroMinHeight());
  }

  public Command homeRunMin() {
    return run(() -> {
          // Override command from periodic
          runVolts(homingDownVolts.get());
          zeroed = false;
        })
        .raceWith(
            Commands.waitSeconds(1.0)
                .andThen(
                    Commands.waitUntil(
                        () ->
                            Math.abs(getMeasuredVelocityMetersPerSec())
                                <= homingVelocityThreshold.get())))
        .andThen(this::zeroMinHeight);
  }

  public Command homeRunMax() {
    return run(() -> {
          // Override command from periodic
          runVolts(homingUpVolts.get());
          zeroed = false;
        })
        .raceWith(
            Commands.waitSeconds(1.0)
                .andThen(
                    Commands.waitUntil(
                        () ->
                            Math.abs(getMeasuredVelocityMetersPerSec())
                                <= homingVelocityThreshold.get())))
        .andThen(this::zeroMaxHeight);
  }

  public void setCompactingMode(TrashCompactorCompactingMode mode) {
    if (compactingMode != mode) {
      modeTimer.restart();
      if (mode == TrashCompactorCompactingMode.LAUNCHING) {
        launchingPosition = getMeasuredHeightMeters() + compactLaunchingOffset.get();
      }
    }
    compactingMode = mode;
  }

  public enum TrashCompactorCompactingMode {
    PASSIVE_DOWN,
    FORCE_MAX,
    FORCE_MIN,
    LAUNCHING,
    IDLE
  }
}
