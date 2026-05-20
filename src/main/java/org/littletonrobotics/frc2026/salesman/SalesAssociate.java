// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2026.salesman;

import static org.wpilib.math.autodiff.Variable.*;
import static org.wpilib.math.optimization.Constraints.*;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import org.littletonrobotics.frc2026.FieldConstants;
import org.littletonrobotics.frc2026.subsystems.drive.DriveConstants;
import org.littletonrobotics.frc2026.util.geometry.Bounds;
import org.wpilib.math.autodiff.Variable;
import org.wpilib.math.optimization.Problem;
import org.wpilib.math.optimization.solver.ExitStatus;
import org.wpilib.math.optimization.solver.Options;

public class SalesAssociate implements SalesmanSolverIO {
  /// MARK: - Constants

  public static final double tMax = 1.0;
  public static final double tStep = 0.1;
  private static final double maxSquareCoeff = 1.5;
  private static final double intakeRangeScale = 0.75;
  private static final double intakeDistanceSigmoidCompression = 6.0;
  private static final double intakeRewardSigmoidCompression = 8.0;
  private static final Options solverOptions =
      new Options().withTimeout(0.15).withDiagnostics(false);
  private static final Translation2d blueHubAvoidance =
      new Translation2d(
          FieldConstants.LinesVertical.neutralZoneNear, FieldConstants.LinesHorizontal.center);
  private static final Translation2d redHubAvoidance =
      new Translation2d(
          FieldConstants.LinesVertical.neutralZoneFar, FieldConstants.LinesHorizontal.center);
  private static final double hubAvoidanceRadius = 1.0;
  private static final Variable zero = new Variable(0);
  private static final Variable one = new Variable(1);

  /// MARK: - IO methods

  // Request data (requires mutex)
  private final Object requestLock = new Object();
  private boolean requestEnabled = false;
  private Translation2d[] requestFuel = new Translation2d[] {};
  private Translation2d requestIntakePosition = Translation2d.kZero;
  private Translation2d requestIntakeVelocity = Translation2d.kZero;
  private Bounds requestBounds = SalesmanSolver.defaultBounds;

  // Result data (requires mutex)
  private final Object resultLock = new Object();
  private long resultSeqnum = -1;
  private ExitStatus resultExitStatus = ExitStatus.SUCCESS;
  private double resultSolveTimeMS = 0.0;
  private double resultReward = 0.0;
  private double[] resultXCoeffs = new double[3];
  private double[] resultYCoeffs = new double[3];

  public void updateInputs(SalesmanSolverIOInputs inputs) {
    synchronized (resultLock) {
      inputs.seqnum = resultSeqnum;
      inputs.exitStatus = resultExitStatus;
      inputs.solveTimeMS = resultSolveTimeMS;
      inputs.reward = resultReward;
      inputs.xCoeffs = resultXCoeffs;
      inputs.yCoeffs = resultYCoeffs;
    }
  }

  public void setEnabled(boolean enabled) {
    synchronized (requestLock) {
      requestEnabled = enabled;
    }
  }

  public void setRequest(
      Translation2d[] fuel,
      Translation2d intakePosition,
      Translation2d intakeVelocity,
      Bounds bounds) {
    synchronized (requestLock) {
      requestFuel = fuel;
      requestIntakePosition = intakePosition;
      requestIntakeVelocity = intakeVelocity;
      requestBounds = bounds;
    }
  }

  /// MARK: - Solver

  public SalesAssociate() {
    var solverThread = new Thread(this::run, "SalesAssociate");
    solverThread.setDaemon(true);
    solverThread.start();
  }

  private void run() {
    if (RobotBase.isReal()) {
      throw new RuntimeException(
          "The salesman solver is not designed to run on the roboRIO due to performance constraints.");
    }

    boolean enabled;
    Translation2d[] fuel;
    Translation2d intakePosition;
    Translation2d intakeVelocity;
    Bounds bounds;

    long seqnum = 0;
    double lastXCoeff2 = 0.0;
    double lastYCoeff2 = 0.0;

    while (true) {
      // Get request data
      synchronized (requestLock) {
        enabled = requestEnabled;
        fuel = requestFuel;
        intakePosition = requestIntakePosition;
        intakeVelocity = requestIntakeVelocity;
        bounds = requestBounds;
      }

      // If disabled, spin
      if (!enabled) {
        try {
          Thread.sleep(5);
        } catch (InterruptedException e) {
        }
        continue;
      }

      // Run the solver
      try (var problem = new Problem()) {
        // Initialize coefficients
        var xCoeffs = problem.decisionVariable(3);
        var yCoeffs = problem.decisionVariable(3);

        // Constrain coefficients
        problem.subjectTo(eq(xCoeffs.get(0), intakePosition.getX()));
        problem.subjectTo(eq(yCoeffs.get(0), intakePosition.getY()));
        problem.subjectTo(eq(xCoeffs.get(1), intakeVelocity.getX()));
        problem.subjectTo(eq(yCoeffs.get(1), intakeVelocity.getY()));
        problem.subjectTo(bounds(-maxSquareCoeff, xCoeffs.get(2), maxSquareCoeff));
        problem.subjectTo(bounds(-maxSquareCoeff, yCoeffs.get(2), maxSquareCoeff));

        // Set initial values
        xCoeffs.get(0).setValue(intakePosition.getX());
        yCoeffs.get(0).setValue(intakePosition.getY());
        xCoeffs.get(1).setValue(intakeVelocity.getX());
        yCoeffs.get(1).setValue(intakeVelocity.getY());
        xCoeffs.get(2).setValue(lastXCoeff2);
        yCoeffs.get(2).setValue(lastYCoeff2);

        // Iterate over fuel
        Variable reward = zero;
        for (Translation2d fuelTranslation : fuel) {
          // Skip if out of bounds
          if (!bounds.contains(fuelTranslation)) continue;

          // Discretize trajectory
          Variable intakeReward = null;
          for (double t = 0; t <= tMax; t += tStep) {
            // Calculate position
            var x = xCoeffs.get(0).plus(xCoeffs.get(1).times(t)).plus(xCoeffs.get(2).times(t * t));
            var y = yCoeffs.get(0).plus(yCoeffs.get(1).times(t)).plus(yCoeffs.get(2).times(t * t));

            // Calculate intake reward
            var dist = hypot(x.minus(fuelTranslation.getX()), y.minus(fuelTranslation.getY()));
            var term =
                compressedSigmoid(
                    dist,
                    DriveConstants.intakeWidth / 2.0 * intakeRangeScale,
                    true,
                    intakeDistanceSigmoidCompression);
            intakeReward = intakeReward == null ? term : intakeReward.plus(term);
          }

          // Add intake reward
          intakeReward = compressedSigmoid(intakeReward, 1, false, intakeRewardSigmoidCompression);
          reward = reward.plus(intakeReward);
        }

        // Enforce workplace regulations (HR)
        var endX =
            xCoeffs.get(0).plus(xCoeffs.get(1).times(tMax)).plus(xCoeffs.get(2).times(tMax * tMax));
        var endY =
            yCoeffs.get(0).plus(yCoeffs.get(1).times(tMax)).plus(yCoeffs.get(2).times(tMax * tMax));
        problem.subjectTo(bounds(bounds.minX(), endX, bounds.maxX()));
        problem.subjectTo(bounds(bounds.minY(), endY, bounds.maxY()));
        problem.subjectTo(
            gt(
                hypot(endX.minus(blueHubAvoidance.getX()), endY.minus(blueHubAvoidance.getY())),
                hubAvoidanceRadius));
        problem.subjectTo(
            gt(
                hypot(endX.minus(redHubAvoidance.getX()), endY.minus(redHubAvoidance.getY())),
                hubAvoidanceRadius));

        // Solve the problem
        problem.maximize(reward);
        seqnum++;
        double solveStart = Timer.getFPGATimestamp();
        var exitStatus = problem.solve(solverOptions);
        double solveTimeMS = (Timer.getFPGATimestamp() - solveStart) * 1000.0;

        // Update result data
        var xCoeffVals =
            new double[] {xCoeffs.get(0).value(), xCoeffs.get(1).value(), xCoeffs.get(2).value()};
        var yCoeffVals =
            new double[] {yCoeffs.get(0).value(), yCoeffs.get(1).value(), yCoeffs.get(2).value()};
        lastXCoeff2 = xCoeffs.get(2).value();
        lastYCoeff2 = yCoeffs.get(2).value();
        synchronized (resultLock) {
          resultSeqnum = seqnum;
          resultExitStatus = exitStatus;
          resultSolveTimeMS = solveTimeMS;
          resultReward = reward.value();
          resultXCoeffs = xCoeffVals;
          resultYCoeffs = yCoeffVals;
        }
      }

      // Pause to avoid busylooping
      try {
        Thread.sleep(5);
      } catch (InterruptedException e) {
      }
    }
  }

  private static Variable compressedSigmoid(
      Variable x, double center, boolean invert, double compression) {
    var f = one.div(pow(Math.E, x.minus(center).times(-compression)).plus(one));
    if (invert) {
      f = one.minus(f);
    }
    return f;
  }
}
