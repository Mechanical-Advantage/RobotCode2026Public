// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2026.subsystems.slamtake;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.function.BooleanSupplier;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.frc2026.DarwinMechanism3d;
import org.littletonrobotics.frc2026.RobotState;
import org.littletonrobotics.frc2026.subsystems.rollers.RollerSystem;
import org.littletonrobotics.frc2026.subsystems.rollers.RollerSystemIO;
import org.littletonrobotics.frc2026.subsystems.slamtake.SlamIO.SlamIOOutputMode;
import org.littletonrobotics.frc2026.util.EqualsUtil;
import org.littletonrobotics.frc2026.util.FullSubsystem;
import org.littletonrobotics.frc2026.util.LoggedTracer;
import org.littletonrobotics.frc2026.util.LoggedTunableNumber;
import org.littletonrobotics.junction.AutoLogOutput;

public class Slamtake extends FullSubsystem {
  private static final LoggedTunableNumber rollerIntakeVolts =
      new LoggedTunableNumber("Slamtake/Roller/IntakeVolts", 13.0);
  private static final LoggedTunableNumber rollerOuttakeVolts =
      new LoggedTunableNumber("Slamtake/Roller/OuttakeVolts", -6.0);
  private final RollerSystem roller;
  private final Slam slam;

  private static final LoggedTunableNumber retractMaxVelocityRadsPerSec =
      new LoggedTunableNumber("Slamtake/Slam/RetractMaxVelocity", 3.0);
  private static final LoggedTunableNumber slowRetractMaxVelocityRadsPerSec =
      new LoggedTunableNumber("Slamtake/Slam/SlowRetractMaxVelocity", 0.88);
  private static final LoggedTunableNumber slamGoalDebounceTime =
      new LoggedTunableNumber("Slamtake/Slam/DebounceTime", 0.5);
  private Debouncer slamGoalDebouncer =
      new Debouncer(slamGoalDebounceTime.get(), DebounceType.kRising);

  private SlewRateLimiter retractSlewRateLimiter =
      new SlewRateLimiter(retractMaxVelocityRadsPerSec.get());
  private SlewRateLimiter slowRetractSlewRateLimiter =
      new SlewRateLimiter(slowRetractMaxVelocityRadsPerSec.get());

  @Getter @Setter @AutoLogOutput private IntakeGoal intakeGoal = IntakeGoal.STOP;
  @Getter @Setter @AutoLogOutput private SlamGoal slamGoal = SlamGoal.RETRACT;
  @Getter @AutoLogOutput private SlamState slamState = SlamState.MOVING;

  public Slamtake(SlamIO slamIO, RollerSystemIO rollerIO) {
    this.slam = new Slam(slamIO);
    this.roller = new RollerSystem("Intake roller", "Slamtake/Roller", rollerIO);
  }

  public void periodic() {
    slam.periodic();
    roller.periodic();

    if (slamGoalDebounceTime.hasChanged(hashCode())) {
      slamGoalDebouncer = new Debouncer(slamGoalDebounceTime.get());
    }
    if (retractMaxVelocityRadsPerSec.hasChanged(hashCode())) {
      retractSlewRateLimiter = new SlewRateLimiter(retractMaxVelocityRadsPerSec.get());
    }
    if (slowRetractMaxVelocityRadsPerSec.hasChanged(hashCode())) {
      slowRetractSlewRateLimiter = new SlewRateLimiter(slowRetractMaxVelocityRadsPerSec.get());
    }

    double rollerVolts = 0.0;
    switch (intakeGoal) {
      case INTAKE -> {
        rollerVolts = rollerIntakeVolts.get();
      }
      case OUTTAKE -> {
        rollerVolts = rollerOuttakeVolts.get();
      }
      case STOP -> {
        rollerVolts = 0.0;
      }
    }
    roller.runOpenLoop(rollerVolts);
    switch (slamGoal) {
      case DEPLOY -> {
        slam.runPosition(Slam.slamMinAngle);
        slamState =
            slamGoalDebouncer.calculate(slam.atGoal()) ? SlamState.DEPLOYED : SlamState.MOVING;
        retractSlewRateLimiter.reset(slam.getMeasuredAngleRad());
        slowRetractSlewRateLimiter.reset(slam.getMeasuredAngleRad());
      }
      case RETRACT -> {
        double goal = Slam.slamMaxAngle;
        double setpoint = retractSlewRateLimiter.calculate(goal);
        slam.runPosition(setpoint);
        slamState =
            EqualsUtil.epsilonEquals(setpoint, goal) ? SlamState.RETRACTED : SlamState.MOVING;
        slowRetractSlewRateLimiter.reset(slam.getMeasuredAngleRad());
      }
      case RETRACT_SLOW -> {
        double goal = Slam.slamMaxAngle;
        double setpoint = slowRetractSlewRateLimiter.calculate(goal);
        slam.runPosition(setpoint);
        slamState =
            EqualsUtil.epsilonEquals(setpoint, goal) ? SlamState.RETRACTED : SlamState.MOVING;
        retractSlewRateLimiter.reset(slam.getMeasuredAngleRad());
      }
      case IDLE -> {
        slam.setMode(SlamIOOutputMode.COAST);
        slamState = SlamState.MOVING;
        retractSlewRateLimiter.reset(slam.getMeasuredAngleRad());
        slowRetractSlewRateLimiter.reset(slam.getMeasuredAngleRad());
      }
    }

    // Send hopper extension data to RobotState
    if (DriverStation.isEnabled()) {
      if (slam.isZeroed()) {
        RobotState.getInstance()
            .addSlamObservation(
                new RobotState.SlamObservation(
                    Timer.getTimestamp(), new Rotation2d(slam.getMeasuredAngleRad())));
      }
    }

    // Visualize intake in 3D
    if (slam.isZeroed()) {
      DarwinMechanism3d.getMeasured().setIntakeAngle(new Rotation2d(slam.getMeasuredAngleRad()));
    } else {
      DarwinMechanism3d.getMeasured().setIntakeAngle(new Rotation2d(Slam.slamMaxAngle));
    }

    // Record cycle time
    LoggedTracer.record("Slamtake/Periodic");
  }

  @Override
  public void periodicAfterScheduler() {
    roller.periodicAfterScheduler();
    slam.periodicAfterScheduler();

    LoggedTracer.record("Slamtake/AfterScheduler");
  }

  public void setCoastOverride(BooleanSupplier coast) {
    slam.setCoastOverride(coast);
    roller.setCoastOverride(coast);
  }

  public boolean isZeroed() {
    return slam.isZeroed();
  }

  public Command homeSlam() {
    return slam.zeroCommand().deadlineFor(Commands.idle(this)); // Require slamtake
  }

  public Command zeroMaxSlam() {
    return Commands.runOnce(() -> slam.zeroMaxAngle()).ignoringDisable(true);
  }

  public enum IntakeGoal {
    INTAKE,
    OUTTAKE,
    STOP
  }

  public enum SlamGoal {
    DEPLOY,
    RETRACT,
    RETRACT_SLOW,
    IDLE
  }

  public enum SlamState {
    DEPLOYED,
    RETRACTED,
    MOVING
  }
}
