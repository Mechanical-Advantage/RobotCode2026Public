// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2026.subsystems.hood;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import lombok.Getter;
import lombok.experimental.Accessors;
import org.littletonrobotics.frc2026.Constants;
import org.littletonrobotics.frc2026.Robot;
import org.littletonrobotics.frc2026.subsystems.hood.HoodIO.HoodIOOutputMode;
import org.littletonrobotics.frc2026.subsystems.hood.HoodIO.HoodIOOutputs;
import org.littletonrobotics.frc2026.util.EqualsUtil;
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
  private static final LoggedTunableNumber kS = new LoggedTunableNumber("Hood/kS");
  private static final LoggedTunableNumber kG = new LoggedTunableNumber("Hood/kG");
  private static final LoggedTunableNumber kA = new LoggedTunableNumber("Hood/kA");
  private static final LoggedTunableNumber maxVelocityRadPerSec =
      new LoggedTunableNumber("Hood/MaxVelocityRadPerSec", 60.0);
  private static final LoggedTunableNumber maxAccelerationRadPerSec2 =
      new LoggedTunableNumber("Hood/MaxAccelerationRadPerSec2", 80.0);

  static {
    kP.initDefault(30000);
    kD.initDefault(300);
    kS.initDefault(0);
    kG.initDefault(0);
    kA.initDefault(0);
  }

  private final HoodIO io;
  private final HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();
  private final HoodIOOutputs outputs = new HoodIOOutputs();

  // Connected debouncer
  private final Debouncer motorConnectedDebouncer =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final Alert motorDisconnectedAlert =
      new Alert("Hood motor disconnected!", Alert.AlertType.kWarning);

  private TrapezoidProfile profile;
  @Getter private State setpoint = new State();

  @Getter
  @Accessors(fluent = true)
  @AutoLogOutput(key = "Hood/Profile/AtGoal")
  private boolean atGoal = false;

  private double goalAngle = 0.0;
  private double goalVelocity = 0.0;

  private static double hoodOffset = 0.0;

  public Hood(HoodIO io) {
    this.io = io;

    profile =
        new TrapezoidProfile(
            new TrapezoidProfile.Constraints(
                maxVelocityRadPerSec.get(), maxAccelerationRadPerSec2.get()));
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Hood", inputs);

    motorDisconnectedAlert.set(
        Robot.showHardwareAlerts() && !motorConnectedDebouncer.calculate(inputs.motorConnected));

    // Update tunable numbers
    outputs.kP = kP.get();
    outputs.kD = kD.get();

    if (maxVelocityRadPerSec.hasChanged(hashCode())
        || maxAccelerationRadPerSec2.hasChanged(hashCode())) {
      profile =
          new TrapezoidProfile(
              new TrapezoidProfile.Constraints(
                  maxVelocityRadPerSec.get(), maxAccelerationRadPerSec2.get()));
    }

    // Clamp goal
    var goalState =
        new State(
            MathUtil.clamp(goalAngle, minAngle, maxAngle),
            MathUtil.clamp(goalVelocity, 0.0, maxVelocityRadPerSec.get()));
    double previousVelocity = setpoint.velocity;
    setpoint = profile.calculate(Constants.loopPeriodSecs, setpoint, goalState);
    if (setpoint.position < minAngle || setpoint.position > maxAngle) {
      setpoint =
          new State(
              MathUtil.clamp(setpoint.position, minAngle, maxAngle),
              MathUtil.clamp(setpoint.velocity, 0.0, maxVelocityRadPerSec.get()));
    }

    // Check at goal
    atGoal =
        EqualsUtil.epsilonEquals(setpoint.position, goalState.position)
            && EqualsUtil.epsilonEquals(setpoint.velocity, goalState.velocity);

    // Run
    double accel = (setpoint.velocity - previousVelocity) / Constants.loopPeriodSecs;
    outputs.positionRad = setpoint.position - hoodOffset;
    outputs.velocityRadsPerSec = setpoint.velocity;
    outputs.feedforward =
        kS.get() * Math.signum(setpoint.velocity)
            + kG.get() * Math.cos(setpoint.position)
            + kA.get() * accel;
    outputs.mode = HoodIOOutputMode.CLOSED_LOOP;

    // Log state
    Logger.recordOutput("Hood/Profile/SetpointPositionRad", setpoint.position);
    Logger.recordOutput("Hood/Profile/SetpointVelocityRadPerSec", setpoint.velocity);
    Logger.recordOutput("Hood/Profile/GoalPositionRad", goalState.position);
    Logger.recordOutput("Hood/Profile/GoalVelocityRadPerSec", goalState.velocity);

    // Record cycle time
    LoggedTracer.record("Hood");
  }

  public void setGoalParams(double angle, double velocity) {
    goalAngle = angle;
    goalVelocity = velocity;
  }

  @Override
  public void periodicAfterScheduler() {
    io.applyOutputs(outputs);
  }

  @AutoLogOutput(key = "Hood/MeasuredAngleRads")
  public double getMeasuredAngleRad() {
    return inputs.internalPosition + hoodOffset;
  }

  public void zero() {
    hoodOffset = minAngle - inputs.internalPosition;
  }
}
