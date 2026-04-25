// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2026.subsystems.hopper;

import java.util.Optional;
import java.util.function.BooleanSupplier;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.frc2026.RobotContainer.SimFuelCount;
import org.littletonrobotics.frc2026.subsystems.rollers.RollerSystem;
import org.littletonrobotics.frc2026.subsystems.rollers.RollerSystemIO;
import org.littletonrobotics.frc2026.util.FullSubsystem;
import org.littletonrobotics.frc2026.util.LoggedTracer;
import org.littletonrobotics.frc2026.util.LoggedTunableNumber;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Hopper extends FullSubsystem {
  private static final LoggedTunableNumber rollerLaunchVolts =
      new LoggedTunableNumber("Hopper/Roller/LaunchVolts", 12.0);
  private static final LoggedTunableNumber rollerConservativeLaunchVolts =
      new LoggedTunableNumber("Hopper/Roller/ConservativeLaunchVolts", 3.0);
  private static final LoggedTunableNumber rollerOuttakeVolts =
      new LoggedTunableNumber("Hopper/Roller/OuttakeVolts", -12.0);

  private final RollerSystem roller;

  @Setter private BooleanSupplier coastOverride = () -> false;
  @Setter private boolean sportMode = false;
  private final Optional<SimFuelCount> simFuelCount;

  @Getter @Setter @AutoLogOutput private Goal goal = Goal.STOP;

  public Hopper(RollerSystemIO rollerIO, Optional<SimFuelCount> simFuelCount) {
    this.roller = new RollerSystem("Hopper roller", "Hopper/Roller", rollerIO);
    this.simFuelCount = simFuelCount;
    roller.setCoastOverride(coastOverride);
  }

  public void periodic() {
    roller.periodic();

    // Log sim fuel count
    if (simFuelCount.isPresent()) {
      Logger.recordOutput("Hopper/SimFueldCount", simFuelCount.get().getFuelStored());
    }

    // Update roller output
    double rollerVolts = 0.0;
    switch (goal) {
      case LAUNCH -> {
        rollerVolts = sportMode ? rollerConservativeLaunchVolts.get() : rollerLaunchVolts.get();
      }
      case OUTTAKE -> {
        rollerVolts = rollerOuttakeVolts.get();
      }
      case STOP -> {
        rollerVolts = 0.0;
      }
    }

    roller.runOpenLoop(rollerVolts);
    LoggedTracer.record("Hopper/Periodic");
  }

  @Override
  public void periodicAfterScheduler() {
    roller.periodicAfterScheduler();
    LoggedTracer.record("Hopper/AfterScheduler");
  }

  public enum Goal {
    LAUNCH,
    OUTTAKE,
    STOP
  }
}
