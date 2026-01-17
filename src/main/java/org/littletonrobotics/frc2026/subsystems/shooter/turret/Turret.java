// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2026.subsystems.shooter.turret;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.frc2026.AlphaMechanism3d;
import org.littletonrobotics.frc2026.Constants;
import org.littletonrobotics.frc2026.Robot;
import org.littletonrobotics.frc2026.RobotState;
import org.littletonrobotics.frc2026.subsystems.shooter.ShotCalculator;
import org.littletonrobotics.frc2026.subsystems.shooter.turret.TurretIO.TurretIOOutputMode;
import org.littletonrobotics.frc2026.subsystems.shooter.turret.TurretIO.TurretIOOutputs;
import org.littletonrobotics.frc2026.util.FullSubsystem;
import org.littletonrobotics.frc2026.util.LoggedTracer;
import org.littletonrobotics.frc2026.util.LoggedTunableNumber;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Turret extends FullSubsystem {
  private static final double minAngle = Units.degreesToRadians(-210.0);
  private static final double maxAngle = Units.degreesToRadians(210.0);
  private static final double trackOverlapMargin = Units.degreesToRadians(10);
  private static final double trackCenterRads = (minAngle + maxAngle) / 2;
  private static final double trackMinAngle = trackCenterRads - Math.PI - trackOverlapMargin;
  private static final double trackMaxAngle = trackCenterRads + Math.PI + trackOverlapMargin;

  private static final LoggedTunableNumber maxVelocity =
      new LoggedTunableNumber("Turret/MaxVelocity");
  private static final LoggedTunableNumber kP = new LoggedTunableNumber("Turret/kP");
  private static final LoggedTunableNumber kD = new LoggedTunableNumber("Turret/kD");
  private static final LoggedTunableNumber kA = new LoggedTunableNumber("Turret/kA");

  static {
    switch (Constants.robot) {
      case COMPBOT -> {
        maxVelocity.initDefault(0.5);
        kP.initDefault(500.0);
        kD.initDefault(0.0);
        kA.initDefault(0.0);
      }
      case ALPHABOT -> {
        maxVelocity.initDefault(12.0);
        kP.initDefault(2000.0);
        kD.initDefault(50.0);
        kA.initDefault(0.0);
      }
      case SIMBOT -> {
        maxVelocity.initDefault(10.0);
        kP.initDefault(100.0);
        kD.initDefault(0.0);
        kA.initDefault(0.0);
      }
      default -> {
        maxVelocity.initDefault(6.0);
        kP.initDefault(0.0);
        kD.initDefault(0.0);
        kA.initDefault(1.0);
      }
    }
  }

  private final TurretIO turretIO;
  private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();
  private final TurretIOOutputs outputs = new TurretIOOutputs();

  @Getter @Setter @AutoLogOutput private ShootState shootState = ShootState.ACTIVE_SHOOTING;
  private Rotation2d goalAngle = Rotation2d.kZero;
  private double goalVelocity = 0.0;
  private double lastGoalAngle = 0.0;

  private final Debouncer motorConnectedDebouncer = new Debouncer(0.5, DebounceType.kFalling);

  private final Alert disconnected =
      new Alert("Turret motor disconnected!", Alert.AlertType.kWarning);
  @Setter private BooleanSupplier coastOverride = () -> false;

  SlewRateLimiter profile = new SlewRateLimiter(maxVelocity.get());
  private double turretOffset;

  public Turret(TurretIO turretIO) {
    this.turretIO = turretIO;
  }

  public void periodic() {
    turretIO.updateInputs(inputs);
    Logger.processInputs("Turret", inputs);

    disconnected.set(
        Robot.showHardwareAlerts() && !motorConnectedDebouncer.calculate(inputs.motorConnected));

    // Stop when disabled
    if (DriverStation.isDisabled()) {
      outputs.mode = TurretIOOutputMode.BRAKE;

      if (coastOverride.getAsBoolean()) {
        outputs.mode = TurretIOOutputMode.COAST;
      }
    }

    // Update profile constraints
    if (maxVelocity.hasChanged(hashCode())) {
      profile = new SlewRateLimiter(maxVelocity.get());
    }

    // Reset profile when disabled
    if (DriverStation.isDisabled()) {
      profile.reset(getPosition());
      lastGoalAngle = getPosition();
    }

    // Publish position
    RobotState.getInstance()
        .addTurretObservation(
            new RobotState.TurretObservation(Timer.getTimestamp(), new Rotation2d(getPosition())));

    // Visualize turret in 3D
    AlphaMechanism3d.getMeasured().setTurretAngle(new Rotation2d(getPosition()));

    // Record cycle time
    LoggedTracer.record("Turret");
  }

  @Override
  public void periodicAfterScheduler() {
    // Delay tracking math until after the RobotState has been updated
    if (DriverStation.isEnabled()) {
      Rotation2d robotAngle = RobotState.getInstance().getRotation();
      double robotAngularVelocity =
          RobotState.getInstance().getFieldVelocity().omegaRadiansPerSecond;

      Rotation2d robotRelativeGoalAngle = goalAngle.minus(robotAngle);
      double robotRelativeGoalVelocity = goalVelocity - robotAngularVelocity;

      boolean hasBestAngle = false;
      double bestAngle = 0;
      double minLegalAngle =
          switch (shootState) {
            case ACTIVE_SHOOTING -> minAngle;
            case TRACKING -> trackMinAngle;
          };
      double maxLegalAngle =
          switch (shootState) {
            case ACTIVE_SHOOTING -> maxAngle;
            case TRACKING -> trackMaxAngle;
          };
      for (int i = -2; i < 3; i++) {
        double potentialSetpoint = robotRelativeGoalAngle.getRadians() + Math.PI * 2.0 * i;
        if (potentialSetpoint < minLegalAngle || potentialSetpoint > maxLegalAngle) {
          continue;
        } else {
          if (!hasBestAngle) {
            bestAngle = potentialSetpoint;
            hasBestAngle = true;
          }
          if (Math.abs(lastGoalAngle - potentialSetpoint) < Math.abs(lastGoalAngle - bestAngle)) {
            bestAngle = potentialSetpoint;
          }
        }
      }
      lastGoalAngle = bestAngle;

      double setpoint = profile.calculate(MathUtil.clamp(bestAngle, minLegalAngle, maxLegalAngle));
      Logger.recordOutput("Turret/GoalPositionRad", bestAngle);
      Logger.recordOutput("Turret/GoalVelocityRadPerSec", robotRelativeGoalVelocity);
      Logger.recordOutput("Turret/SetpointPositionRad", setpoint);
      Logger.recordOutput("Turret/SetpointVelocityRadPerSec", robotRelativeGoalVelocity);

      outputs.mode = TurretIOOutputMode.CLOSED_LOOP;
      outputs.position = setpoint - turretOffset;
      outputs.velocity = robotRelativeGoalVelocity;
      outputs.kP = kP.get();
      outputs.kD = kD.get();
    }

    // Apply outputs
    turretIO.applyOutputs(outputs);
  }

  private void setFieldRelativeTarget(Rotation2d angle, double velocity) {
    this.goalAngle = angle;
    this.goalVelocity = velocity;
  }

  private void zero() {
    turretOffset = -inputs.positionRads;
  }

  @AutoLogOutput(key = "Turret/MeasuredPositionRad")
  public double getPosition() {
    return inputs.positionRads + turretOffset;
  }

  @AutoLogOutput(key = "Turret/MeasuredVelocityRadPerSec")
  public double getVelocity() {
    return inputs.velocityRadsPerSec;
  }

  public Command runTrackTargetCommand() {
    return run(
        () -> {
          var params = ShotCalculator.getInstance().getParameters();
          setFieldRelativeTarget(params.turretAngle(), params.turretVelocity());
          setShootState(ShootState.TRACKING);
        });
  }

  public Command runTrackTargetActiveShootingCommand() {
    return run(
        () -> {
          var params = ShotCalculator.getInstance().getParameters();
          setFieldRelativeTarget(params.turretAngle(), params.turretVelocity());
          setShootState(ShootState.ACTIVE_SHOOTING);
        });
  }

  public Command runFixedCommand(Supplier<Rotation2d> angle, DoubleSupplier velocity) {
    return run(
        () -> {
          setFieldRelativeTarget(angle.get(), velocity.getAsDouble());
          setShootState(ShootState.TRACKING);
        });
  }

  public Command zeroCommand() {
    return runOnce(this::zero).ignoringDisable(true);
  }

  public enum ShootState {
    ACTIVE_SHOOTING,
    TRACKING
  }
}
