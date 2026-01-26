// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2026.subsystems.hopper;

import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.frc2026.subsystems.rollers.RollerSystem;
import org.littletonrobotics.frc2026.subsystems.rollers.RollerSystemIO;
import org.littletonrobotics.frc2026.util.FullSubsystem;
import org.littletonrobotics.frc2026.util.LoggedTunableNumber;
import org.littletonrobotics.junction.AutoLogOutput;

public class Hopper extends FullSubsystem {
  private static final LoggedTunableNumber rollerShootVolts =
      new LoggedTunableNumber("Hopper/Roller/ShootVolts", 12.0);
  private static final LoggedTunableNumber rollerOuttakeVolts =
      new LoggedTunableNumber("Hopper/Roller/OuttakeVolts", -12.0);
  private static final LoggedTunableNumber leftIndexerShootVolts =
      new LoggedTunableNumber("Hopper/Indexer/LeftShootVolts", 6.0);
  private static final LoggedTunableNumber leftIndexerOuttakeVolts =
      new LoggedTunableNumber("Hopper/Indexer/LeftOuttakeVolts", -6.0);
  private static final LoggedTunableNumber rightIndexerShootVolts =
      new LoggedTunableNumber("Hopper/Indexer/RightShootVolts", 6.0);
  private static final LoggedTunableNumber rightIndexerOuttakeVolts =
      new LoggedTunableNumber("Hopper/Indexer/LeftOuttakeVolts", -6.0);

  private final RollerSystem roller;
  private final RollerSystem leftIndexer;
  private final RollerSystem rightIndexer;

  @Getter @Setter @AutoLogOutput private Goal goal = Goal.STOP;

  public Hopper(
      RollerSystemIO rollerIO,
      RollerSystemIO leftIndexRollerIO,
      RollerSystemIO rightIndexRollerIO) {
    this.roller = new RollerSystem("Hopper roller", "Hopper/Roller", rollerIO);
    this.leftIndexer = new RollerSystem("Left indexer", "Hopper/LeftIndexer", leftIndexRollerIO);
    this.rightIndexer =
        new RollerSystem("Right indexer", "Hopper/RightIndexer", rightIndexRollerIO);
  }

  public void periodic() {

    roller.periodic();
    leftIndexer.periodic();
    rightIndexer.periodic();

    double rollerVolts = 0.0;
    double leftIndexerVolts = 0.0;
    double rightIndexerVolts = 0.0;

    switch (goal) {
      case SHOOT -> {
        rollerVolts = rollerShootVolts.get();
        leftIndexerVolts = leftIndexerShootVolts.get();
        rightIndexerVolts = rightIndexerShootVolts.get();
      }
      case OUTTAKE -> {
        rollerVolts = rollerOuttakeVolts.get();
        leftIndexerVolts = leftIndexerOuttakeVolts.get();
        rightIndexerVolts = rightIndexerOuttakeVolts.get();
      }
      case STOP -> {
        rollerVolts = 0.0;
        leftIndexerVolts = 0.0;
        rightIndexerVolts = 0.0;
      }
    }
    roller.setVolts(rollerVolts);
    leftIndexer.setVolts(leftIndexerVolts);
    rightIndexer.setVolts(rightIndexerVolts);
  }

  @Override
  public void periodicAfterScheduler() {
    roller.periodicAfterScheduler();
    leftIndexer.periodicAfterScheduler();
    rightIndexer.periodicAfterScheduler();
  }

  public enum Goal {
    SHOOT,
    OUTTAKE,
    STOP
  }
}
