// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2026.util;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.ArrayList;
import java.util.List;

/**
 * A standard subsystem that includes an extra periodic callback which runs after the command
 * scheduler. Allows outputs to be published after all other periodic code has finished.
 */
public abstract class FullSubsystem extends SubsystemBase {
  private static List<FullSubsystem> instances = new ArrayList<>();

  public FullSubsystem() {
    super();
    instances.add(this);
  }

  public FullSubsystem(String name) {
    super(name);
    instances.add(this);
  }

  /**
   * This method is called periodically after the command scheduler, and should be used for applying
   * outputs.
   */
  public abstract void periodicAfterScheduler();

  /** Run the "after periodic" methods for all subsystems. */
  public static void runAllPeriodicAfterScheduler() {
    for (FullSubsystem instance : instances) {
      instance.periodicAfterScheduler();
    }
  }
}
