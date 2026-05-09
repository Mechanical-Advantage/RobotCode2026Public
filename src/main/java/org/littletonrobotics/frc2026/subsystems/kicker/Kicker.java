// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2026.subsystems.kicker;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.frc2026.Constants;
import org.littletonrobotics.frc2026.RobotContainer.SimFuelCount;
import org.littletonrobotics.frc2026.subsystems.launcher.LaunchCalculator;
import org.littletonrobotics.frc2026.subsystems.rollers.RollerSystem;
import org.littletonrobotics.frc2026.subsystems.rollers.RollerSystemIO;
import org.littletonrobotics.frc2026.util.FullSubsystem;
import org.littletonrobotics.frc2026.util.LoggedTracer;
import org.littletonrobotics.frc2026.util.LoggedTunableNumber;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Kicker extends FullSubsystem {
  private static final double frontRollerRadius = Units.inchesToMeters(1.25);
  private static final double backRollerRadius = Units.inchesToMeters(0.79);

  private static final LoggedTunableNumber frontkP =
      new LoggedTunableNumber("Kicker/RollerFront/kP", 1.5);
  private static final LoggedTunableNumber frontkD =
      new LoggedTunableNumber("Kicker/RollerFront/kD", 0.0);
  private static final LoggedTunableNumber frontkS =
      new LoggedTunableNumber("Kicker/RollerFront/kS", 0.48);
  private static final LoggedTunableNumber frontkV =
      new LoggedTunableNumber("Kicker/RollerFront/kV", 0.056);

  private static final LoggedTunableNumber backkP =
      new LoggedTunableNumber("Kicker/RollerBack/kP", 3.0);
  private static final LoggedTunableNumber backkD =
      new LoggedTunableNumber("Kicker/RollerBack/kD", 0.0);
  private static final LoggedTunableNumber backkS =
      new LoggedTunableNumber("Kicker/RollerBack/kS", 0.4);
  private static final LoggedTunableNumber backkV =
      new LoggedTunableNumber("Kicker/RollerBack/kV", 0.0575);

  private static final LoggedTunableNumber blearghhSurfaceSpeed =
      new LoggedTunableNumber("Kicker/BlearghhSurfaceSpeed", 6.0);

  private final RollerSystem rollerFront;
  private final RollerSystem rollerBack;
  private final Optional<SimFuelCount> simFuelCount;

  @Setter private BooleanSupplier coastOverride = () -> false;

  @Getter @Setter @AutoLogOutput private Goal goal = Goal.STOP;
  private Goal lastGoal = Goal.STOP;

  private Timer launchTimer = new Timer();

  public Kicker(
      RollerSystemIO rollerIOFront,
      RollerSystemIO rollerIOBack,
      Optional<SimFuelCount> simFuelCount) {
    this.rollerFront = new RollerSystem("Kicker roller front", "Kicker/RollerFront", rollerIOFront);
    this.rollerBack = new RollerSystem("Kicker roller back", "Kicker/RollerBack", rollerIOBack);
    this.simFuelCount = simFuelCount;
    rollerFront.setCoastOverride(coastOverride);
    rollerBack.setCoastOverride(coastOverride);
  }

  public void periodic() {
    rollerFront.periodic();
    rollerBack.periodic();

    if (frontkP.hasChanged(hashCode()) || frontkD.hasChanged(hashCode())) {
      rollerFront.setGains(frontkP.get(), frontkD.get());
    }
    if (frontkS.hasChanged(hashCode()) || frontkV.hasChanged(hashCode())) {
      rollerFront.setFeedforward(frontkS.get(), frontkV.get());
    }
    if (backkP.hasChanged(hashCode()) || backkD.hasChanged(hashCode())) {
      rollerBack.setGains(backkP.get(), backkD.get());
    }
    if (backkS.hasChanged(hashCode()) || backkV.hasChanged(hashCode())) {
      rollerBack.setFeedforward(backkS.get(), backkV.get());
    }

    LoggedTracer.record("Kicker/Periodic");
  }

  @Override
  public void periodicAfterScheduler() {
    double frontSetpointSpeed =
        LaunchCalculator.getInstance().getParameters().kickerSurfaceSpeed() / frontRollerRadius;
    double backSetpointSpeed =
        LaunchCalculator.getInstance().getParameters().kickerSurfaceSpeed() / backRollerRadius;
    Logger.recordOutput("Kicker/FrontSetpointRadPerSec", frontSetpointSpeed);
    Logger.recordOutput("Kicker/BackSetpointRadPerSec", backSetpointSpeed);

    switch (goal) {
      case LAUNCH -> {
        rollerFront.runClosedLoop(frontSetpointSpeed);
        rollerBack.runClosedLoop(backSetpointSpeed);
      }
      case BLEARGHH -> {
        rollerFront.runClosedLoop(blearghhSurfaceSpeed.get() / frontRollerRadius);
        rollerBack.runClosedLoop(blearghhSurfaceSpeed.get() / backRollerRadius);
      }
      case OUTTAKE -> {
        rollerFront.runClosedLoop(frontSetpointSpeed * -1.0);
        rollerBack.runClosedLoop(backSetpointSpeed * -1.0);
      }
      case TEST_FRONT -> {
        rollerFront.runClosedLoop(frontSetpointSpeed);
        rollerBack.runOpenLoop(0.0);
      }
      case TEST_BACK -> {
        rollerFront.runOpenLoop(0.0);
        rollerBack.runClosedLoop(backSetpointSpeed);
      }
      case STOP -> {
        rollerFront.runOpenLoop(0.0);
        rollerBack.runOpenLoop(0.0);
      }
    }

    if (Constants.getMode() == Constants.Mode.SIM
        && goal == Goal.LAUNCH
        && simFuelCount.isPresent()) {
      if (lastGoal != Goal.LAUNCH) {
        launchTimer.restart();
      } else if (launchTimer.advanceIfElapsed(1.0 / SimFuelCount.getLaunchBPS())
          && simFuelCount.get().getFuelStored() > 0) {
        simFuelCount.get().setFuelStored(Math.max(0, simFuelCount.get().getFuelStored() - 1));
      }
    }

    lastGoal = goal;

    rollerFront.periodicAfterScheduler();
    rollerBack.periodicAfterScheduler();

    LoggedTracer.record("Kicker/AfterScheduler");
  }

  public enum Goal {
    LAUNCH,
    BLEARGHH,
    OUTTAKE,
    TEST_FRONT,
    TEST_BACK,
    STOP
  }
}
