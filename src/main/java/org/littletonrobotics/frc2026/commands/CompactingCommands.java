// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2026.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import org.littletonrobotics.frc2026.subsystems.slamtake.Slamtake;
import org.littletonrobotics.frc2026.subsystems.slamtake.Slamtake.IntakeGoal;
import org.littletonrobotics.frc2026.subsystems.slamtake.Slamtake.SlamGoal;
import org.littletonrobotics.frc2026.subsystems.trashcompactor.TrashCompactor;
import org.littletonrobotics.frc2026.subsystems.trashcompactor.TrashCompactor.TrashCompactorCompactingMode;
import org.littletonrobotics.frc2026.util.LoggedTunableNumber;
import org.littletonrobotics.frc2026.util.SuppliedWaitCommand;

public class CompactingCommands {
  public static final LoggedTunableNumber slamLaunchDelay =
      new LoggedTunableNumber("CompactingCommands/SlamLaunchDelay", 0.2);

  private CompactingCommands() {}

  public static Command compact(TrashCompactor trashCompactor, Slamtake slamtake) {
    return Commands.parallel(
        Commands.run(
            () -> trashCompactor.setCompactingMode(TrashCompactorCompactingMode.LAUNCHING),
            trashCompactor),
        new SuppliedWaitCommand(slamLaunchDelay)
            .andThen(
                Commands.runOnce(() -> slamtake.setSlamGoal(SlamGoal.RETRACT_SLOW)),
                Commands.runEnd(
                        () -> slamtake.setIntakeGoal(IntakeGoal.INTAKE),
                        () -> slamtake.setIntakeGoal(IntakeGoal.STOP),
                        slamtake)
                    .asProxy()));
  }
}
