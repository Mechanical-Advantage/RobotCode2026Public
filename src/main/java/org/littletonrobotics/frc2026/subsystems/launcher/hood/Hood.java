// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2026.subsystems.launcher.hood;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.frc2026.DarwinMechanism3d;
import org.littletonrobotics.frc2026.Robot;
import org.littletonrobotics.frc2026.RobotState;
import org.littletonrobotics.frc2026.TrenchBounds;
import org.littletonrobotics.frc2026.energy.FinanceDepartment;
import org.littletonrobotics.frc2026.subsystems.launcher.LaunchCalculator;
import org.littletonrobotics.frc2026.subsystems.launcher.hood.HoodIO.HoodIOOutputMode;
import org.littletonrobotics.frc2026.subsystems.launcher.hood.HoodIO.HoodIOOutputs;
import org.littletonrobotics.frc2026.util.ContinuousConditionalCommand;
import org.littletonrobotics.frc2026.util.EqualsUtil;
import org.littletonrobotics.frc2026.util.FullSubsystem;
import org.littletonrobotics.frc2026.util.LoggedTracer;
import org.littletonrobotics.frc2026.util.LoggedTunableNumber;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Hood extends FullSubsystem {
  public static final double homeAngle = Units.degreesToRadians(9.5);
  public static final double minAngle = Units.degreesToRadians(10);
  public static final double maxAngle = Units.degreesToRadians(41.2);

  private static final LoggedTunableNumber kP = new LoggedTunableNumber("Hood/kP");
  private static final LoggedTunableNumber kD = new LoggedTunableNumber("Hood/kD");
  private static final LoggedTunableNumber toleranceDeg =
      new LoggedTunableNumber("Hood/ToleranceDeg");

  private static final LoggedTunableNumber homingVolts =
      new LoggedTunableNumber("Hood/Homing/Volts", -2);
  private static final LoggedTunableNumber homingVelocityThreshold =
      new LoggedTunableNumber("Hood/Homing/VelocityThreshold", 0.05);

  static {
    kP.initDefault(30000);
    kD.initDefault(500);
    toleranceDeg.initDefault(1.5);
  }

  private final HoodIO io;
  private final HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();
  private final HoodIOOutputs outputs = new HoodIOOutputs();

  // Connected debouncer
  private final Debouncer motorConnectedDebouncer =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final Alert motorDisconnectedAlert =
      new Alert("Hood motor disconnected!", Alert.AlertType.kError);

  @Setter private BooleanSupplier coastOverride = () -> false;

  private double goalAngle = 0.0;
  private double goalVelocity = 0.0;

  private static double hoodOffset = 0.0;
  @Getter private boolean zeroed = false;

  public Hood(HoodIO io) {
    this.io = io;
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Hood", inputs);

    motorDisconnectedAlert.set(
        Robot.showHardwareAlerts() && !motorConnectedDebouncer.calculate(inputs.motorConnected));

    FinanceDepartment.getInstance()
        .reportCurrentUsage("Hood", false, inputs.motorConnected ? inputs.supplyCurrentAmps : 0.0);

    // Stop when disabled
    if (DriverStation.isDisabled() || (!zeroed && outputs.mode != HoodIOOutputMode.OPEN_LOOP)) {
      outputs.mode = HoodIOOutputMode.BRAKE;

      if (coastOverride.getAsBoolean()) {
        outputs.mode = HoodIOOutputMode.COAST;
      }
    }

    // Update tunable numbers
    outputs.kP = kP.get();
    outputs.kD = kD.get();

    // Visualize launcher in 3D
    if (isZeroed()) {
      DarwinMechanism3d.getMeasured().setHoodAngle(new Rotation2d(getMeasuredAngleRad()));
    } else {
      DarwinMechanism3d.getMeasured().setHoodAngle(new Rotation2d(minAngle));
    }

    // Record cycle time
    LoggedTracer.record("Hood/Periodic");
  }

  @Override
  public void periodicAfterScheduler() {
    if (DriverStation.isEnabled() && zeroed) {
      outputs.positionRad = MathUtil.clamp(goalAngle, minAngle, maxAngle) - hoodOffset;
      outputs.velocityRadsPerSec = goalVelocity;
      outputs.mode = HoodIOOutputMode.CLOSED_LOOP;

      // Log state
      Logger.recordOutput("Hood/Profile/GoalPositionRad", goalAngle);
      Logger.recordOutput("Hood/Profile/GoalVelocityRadPerSec", goalVelocity);
    }

    SmartDashboard.putBoolean("Hood At Goal", atGoal());
    SmartDashboard.putBoolean(
        "Hood At Min Angle", EqualsUtil.epsilonEquals(inputs.positionRads, minAngle));

    io.applyOutputs(outputs);
    LoggedTracer.record("Hood/AfterScheduler");
  }

  private void setGoalParams(double angle, double velocity) {
    goalAngle = angle;
    goalVelocity = velocity;
  }

  @AutoLogOutput(key = "Hood/MeasuredAngleRads")
  public double getMeasuredAngleRad() {
    return inputs.positionRads + hoodOffset;
  }

  @AutoLogOutput
  public boolean atGoal() {
    return DriverStation.isEnabled()
        && zeroed
        && Math.abs(getMeasuredAngleRad() - goalAngle)
            <= Units.degreesToRadians(toleranceDeg.get());
  }

  public Command zeroCommand() {
    return run(() -> {
          outputs.appliedVolts = homingVolts.get();
          outputs.mode = HoodIOOutputMode.OPEN_LOOP;
          zeroed = false;
        })
        .raceWith(
            Commands.waitSeconds(0.5)
                .andThen(
                    Commands.waitUntil(
                        () ->
                            Math.abs(inputs.velocityRadsPerSec) <= homingVelocityThreshold.get())))
        .andThen(
            () -> {
              hoodOffset = homeAngle - inputs.positionRads;
              zeroed = true;
            });
  }

  public Command forceZeroCommand() {
    return Commands.runOnce(
            () -> {
              hoodOffset = minAngle - inputs.positionRads;
              zeroed = true;
            })
        .ignoringDisable(true);
  }

  public Command autonomousCommand() {
    return new ContinuousConditionalCommand(
        runFixedCommand(() -> minAngle, () -> 0.0),
        runTrackTargetCommand(),
        TrenchBounds.Hood.contains(
            () -> RobotState.getInstance().getEstimatedPose().getTranslation()));
  }

  public Command runTrackTargetCommand() {
    return run(
        () -> {
          var params = LaunchCalculator.getInstance().getParameters();
          setGoalParams(params.hoodAngle(), params.hoodVelocity());
        });
  }

  public Command runFixedCommand(DoubleSupplier angle, DoubleSupplier velocity) {
    return run(() -> setGoalParams(angle.getAsDouble(), velocity.getAsDouble()));
  }
}
