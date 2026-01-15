// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2026.subsystems.kicker;

import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.frc2026.subsystems.rollers.RollerSystem;
import org.littletonrobotics.frc2026.subsystems.rollers.RollerSystemIO;
import org.littletonrobotics.frc2026.util.FullSubsystem;
import org.littletonrobotics.frc2026.util.LoggedTunableNumber;
import org.littletonrobotics.junction.AutoLogOutput;

public class Kicker extends FullSubsystem {
  private static final LoggedTunableNumber rollerIntakeVolts =
      new LoggedTunableNumber("Kicker/Roller/IntakeVolts", 12.0);
  private static final LoggedTunableNumber rollerOuttakeVolts =
      new LoggedTunableNumber("Kicker/Roller/OuttakeVolts", -8.0);

  private final RollerSystem rollerFront;
  private final RollerSystem rollerBack;

  @Getter @Setter @AutoLogOutput private Goal goal = Goal.STOP;

  public Kicker(RollerSystemIO rollerIOFront, RollerSystemIO rollerIOBack) {
    this.rollerFront = new RollerSystem("Kicker roller front", "Kicker/RollerFront", rollerIOFront);
    this.rollerBack = new RollerSystem("Kicker roller back", "Kicker/RollerBack", rollerIOBack);
  }

  public void periodic() {

    rollerFront.periodic();
    rollerBack.periodic();

    double rollerVolts = 0.0;
    switch (goal) {
      case SHOOT -> {
        rollerVolts = rollerIntakeVolts.get();
      }

      case OUTTAKE -> {
        rollerVolts = rollerOuttakeVolts.get();
      }
      case STOP -> {
        rollerVolts = 0.0;
      }
    }
    rollerFront.setVolts(rollerVolts);
    rollerBack.setVolts(rollerVolts);
  }

  @Override
  public void periodicAfterScheduler() {
    rollerFront.periodicAfterScheduler();
    rollerBack.periodicAfterScheduler();
  }

  public enum Goal {
    SHOOT,
    OUTTAKE,
    STOP
  }
}
