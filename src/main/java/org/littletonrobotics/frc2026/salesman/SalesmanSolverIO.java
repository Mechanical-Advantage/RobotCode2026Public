// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2026.salesman;

import edu.wpi.first.math.geometry.Translation2d;
import org.littletonrobotics.frc2026.util.geometry.Bounds;
import org.littletonrobotics.junction.AutoLog;
import org.wpilib.math.optimization.solver.ExitStatus;

public interface SalesmanSolverIO {
  @AutoLog
  public static class SalesmanSolverIOInputs {
    public long seqnum = -1;
    public ExitStatus exitStatus = ExitStatus.SUCCESS;
    public double solveTimeMS = 0.0;
    public double reward = 0.0;
    public double[] xCoeffs = new double[3];
    public double[] yCoeffs = new double[3];
  }

  public default void updateInputs(SalesmanSolverIOInputs inputs) {}

  /** Enables or disables the solver. */
  public default void setEnabled(boolean enabled) {}

  /**
   * Sets the parameters to use when the solver begins the next iteration. Bounds must have a
   * rotation of zero.
   */
  public default void setRequest(
      Translation2d[] fuel,
      Translation2d intakePosition,
      Translation2d intakeVelocity,
      Bounds bounds) {}
}
