// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2026.subsystems.shooter.hood;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import lombok.Setter;
import org.littletonrobotics.frc2026.AlphaMechanism3d;
import org.littletonrobotics.frc2026.Robot;
import org.littletonrobotics.frc2026.subsystems.shooter.ShotCalculator;
import org.littletonrobotics.frc2026.subsystems.shooter.hood.HoodIO.HoodIOOutputMode;
import org.littletonrobotics.frc2026.subsystems.shooter.hood.HoodIO.HoodIOOutputs;
import org.littletonrobotics.frc2026.util.FullSubsystem;
import org.littletonrobotics.frc2026.util.LoggedTracer;
import org.littletonrobotics.frc2026.util.LoggedTunableNumber;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Hood extends FullSubsystem {
  private static final double minAngle = Units.degreesToRadians(19);
  private static final double maxAngle = Units.degreesToRadians(51);

  private static final LoggedTunableNumber kP = new LoggedTunableNumber("Hood/kP");
  private static final LoggedTunableNumber kD = new LoggedTunableNumber("Hood/kD");
  private static final LoggedTunableNumber toleranceDeg =
      new LoggedTunableNumber("Hood/ToleranceDeg");

  static {
    kP.initDefault(30000);
    kD.initDefault(300);
    toleranceDeg.initDefault(1.0);
  }

  private final HoodIO io;
  private final HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();
  private final HoodIOOutputs outputs = new HoodIOOutputs();

  // Connected debouncer
  private final Debouncer motorConnectedDebouncer =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final Alert motorDisconnectedAlert =
      new Alert("Hood motor disconnected!", Alert.AlertType.kWarning);

  @Setter private BooleanSupplier coastOverride = () -> false;

  private double goalAngle = 0.0;
  private double goalVelocity = 0.0;

  private static double hoodOffset = 0.0;
  private boolean hoodZeroed = false;

  public Hood(HoodIO io) {
    this.io = io;
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Hood", inputs);

    motorDisconnectedAlert.set(
        Robot.showHardwareAlerts() && !motorConnectedDebouncer.calculate(inputs.motorConnected));

    // Stop when disabled
    if (DriverStation.isDisabled() || !hoodZeroed) {
      outputs.mode = HoodIOOutputMode.BRAKE;

      if (coastOverride.getAsBoolean()) {
        outputs.mode = HoodIOOutputMode.COAST;
      }
    }

    // Update tunable numbers
    outputs.kP = kP.get();
    outputs.kD = kD.get();

    // Visualize turret in 3D
    AlphaMechanism3d.getMeasured().setHoodAngle(new Rotation2d(getMeasuredAngleRad()));

    // Record cycle time
    LoggedTracer.record("Hood");
  }

  @Override
  public void periodicAfterScheduler() {
    if (DriverStation.isEnabled() && hoodZeroed) {
      outputs.positionRad = MathUtil.clamp(goalAngle, minAngle, maxAngle) - hoodOffset;
      outputs.velocityRadsPerSec = goalVelocity;
      outputs.mode = HoodIOOutputMode.CLOSED_LOOP;

      // Log state
      Logger.recordOutput("Hood/Profile/GoalPositionRad", goalAngle);
      Logger.recordOutput("Hood/Profile/GoalVelocityRadPerSec", goalVelocity);
    }

    io.applyOutputs(outputs);
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
        && hoodZeroed
        && Math.abs(getMeasuredAngleRad() - goalAngle)
            <= Units.degreesToRadians(toleranceDeg.get());
  }

  private void zero() {
    hoodOffset = minAngle - inputs.positionRads;
    hoodZeroed = true;
  }

  public Command runTrackTargetCommand() {
    return run(
        () -> {
          var params = ShotCalculator.getInstance().getParameters();
          setGoalParams(params.hoodAngle(), params.hoodVelocity());
        });
  }

  public Command runFixedCommand(DoubleSupplier angle, DoubleSupplier velocity) {
    return run(() -> setGoalParams(angle.getAsDouble(), velocity.getAsDouble()));
  }

  public Command zeroCommand() {
    return runOnce(this::zero).ignoringDisable(true);
  }
}
