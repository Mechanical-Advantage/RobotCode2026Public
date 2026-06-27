// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2026.energy;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import org.littletonrobotics.frc2026.Constants;
import org.littletonrobotics.frc2026.util.FullSubsystem;
import org.littletonrobotics.frc2026.util.LoggedTracer;
import org.littletonrobotics.frc2026.util.darwin.DarwinPowerMonitor;
import org.littletonrobotics.idun.IdunPlatform;
import org.littletonrobotics.idun.IdunServer;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

public class FinanceDepartment extends FullSubsystem {
  // MARK: - Constants
  private static final double minVoltageBrownout = 7.0;
  private static final double maxBudgetAmps = 200.0;
  private static final double breakerNiceness = 0.05;
  private static final double budgetWarningThreshold = 180.0;
  // Allow ramping from other subsystems
  private static final double budgetHeadroom = 0.9;
  // Time we can run max budget before trip
  private static final double breakerDangerHorizonSecs = 3.0;
  private final double breakerDamageWarningThreshold;

  private static final Alert budgetWarning =
      new Alert("Battery is low, robot performance may be degraded.", AlertType.kInfo);
  private static final Debouncer budgetWarningDebouncer = new Debouncer(0.5, DebounceType.kBoth);
  private static final Alert brownoutWarning =
      new Alert("Brownout detected, drive performance may be degraded.", AlertType.kWarning);
  private static final Alert breakerDamageWarning =
      new Alert("Breaker damage is high, please stop using the robot.", AlertType.kWarning);
  private static final Debouncer breakerDamageWarningDebouncer =
      new Debouncer(0.5, DebounceType.kBoth);

  private static FinanceDepartment instance;

  public static FinanceDepartment getInstance() {
    if (instance == null) instance = new FinanceDepartment();
    return instance;
  }

  // MARK: - Members
  private final BatteryLogger energyLogger = new BatteryLogger();
  private final BatteryEstimator battery = new BatteryEstimator();
  private final BreakerModel breaker = new BreakerModel(breakerNiceness);
  private final BatteryIOInputsAutoLogged inputs = new BatteryIOInputsAutoLogged();

  private double budget = 0.0;
  private double driveBudget = 0.0;
  private Debouncer brownoutDebouncer = new Debouncer(2.0, DebounceType.kFalling);

  private FinanceDepartment() {
    // Solve for damage state where breaker will trip if we run at maxBudgetAmps for horizon.
    breakerDamageWarningThreshold =
        (1.0 - breakerNiceness)
            - (breakerDangerHorizonSecs
                / BreakerModel.getTripTime(maxBudgetAmps / BreakerModel.I_RATED));
  }

  public void reset() {
    battery.setInitialVoltage(inputs.batteryVoltage, energyLogger.getTotalCurrent());
  }

  public void reportCurrentUsage(String key, boolean drive, double... amps) {
    double totalAmps = 0.0;
    for (double amp : amps) totalAmps += Math.max(0.0, amp);
    energyLogger.reportCurrentUsage(key, drive, totalAmps);
  }

  @Override
  public void periodic() {
    inputs.batteryVoltage = RobotController.getBatteryVoltage();
    inputs.rioCurrent = RobotController.getInputCurrent();
    inputs.brownedOut = IdunServer.isRoboRioBrownedOut();
    inputs.macMiniCurrent = IdunPlatform.isRobot ? DarwinPowerMonitor.getCurrentPowerWatts() : 0.0;
    Logger.processInputs("EnergyLogger", inputs);
    energyLogger.setBatteryVoltage(inputs.batteryVoltage);
    energyLogger.setRioCurrent(inputs.rioCurrent);
    energyLogger.setMacMiniCurrent(inputs.macMiniCurrent);
    LoggedTracer.record("FinanceDepartment/Periodic");
  }

  @Override
  public void periodicAfterScheduler() {
    // Run energy logger
    energyLogger.periodicAfterScheduler();

    // Update models
    battery.update(energyLogger.getTotalCurrent(), inputs.batteryVoltage);
    breaker.update(energyLogger.getTotalCurrent());

    // Calculate budgets
    double batteryMaxCurrent = battery.calculateMaxCurrent(minVoltageBrownout);
    double breakerMaxCurrent = breaker.calculateMaxCurrent(breakerDangerHorizonSecs);
    Logger.recordOutput("FinanceDepartment/BatteryMaxCurrent", batteryMaxCurrent);
    Logger.recordOutput("FinanceDepartment/BreakerMaxCurrent", breakerMaxCurrent);
    budget =
        Math.min(Math.min(batteryMaxCurrent, breakerMaxCurrent) * budgetHeadroom, maxBudgetAmps);

    boolean brownoutDebounced = brownoutDebouncer.calculate(inputs.brownedOut);
    if (!brownoutDebounced) {
      driveBudget = budget - energyLogger.getTotalCurrent() + energyLogger.getDriveCurrent();
    } else {
      double calculatedBudget =
          budget - energyLogger.getTotalCurrent() + energyLogger.getDriveCurrent();
      // Asymmetric ramping of drive budget
      driveBudget =
          calculatedBudget < driveBudget
              ? calculatedBudget
              : Math.min(
                  calculatedBudget,
                  driveBudget + CurrentLimits.driveProbeRateBrownout * Constants.loopPeriodSecs);
    }
    driveBudget = Math.max(0.0, driveBudget);

    Logger.recordOutput("FinanceDepartment/Budget", budget);
    Logger.recordOutput("FinanceDepartment/DriveBudget", driveBudget);

    // Update alerts
    budgetWarning.set(budgetWarningDebouncer.calculate(budget < budgetWarningThreshold));
    brownoutWarning.set(brownoutDebounced);
    breakerDamageWarning.set(
        breakerDamageWarningDebouncer.calculate(
            breaker.getDamageState() > breakerDamageWarningThreshold));

    energyLogger.resetTotals();
    LoggedTracer.record("FinanceDepartment/AfterScheduler");
  }

  /** Get the current limits for a subsystem. */
  public double getDriveLimit() {
    double driveLimit =
        Math.floor(
                MathUtil.clamp(
                        driveBudget / 4.0,
                        CurrentLimits.driveMinLimitAmps,
                        CurrentLimits.driveMaxLimitAmps)
                    / 0.5)
            * 0.5;
    if (DriverStation.isAutonomous()) {
      driveLimit = CurrentLimits.driveAutoLimitAmps;
    }
    Logger.recordOutput("FinanceDepartment/DriveLimit", driveLimit);
    return driveLimit;
  }

  public double getBatteryVoltage() {
    return inputs.batteryVoltage;
  }

  @AutoLog
  public static class BatteryIOInputs {
    public double batteryVoltage = 12.0;
    public double rioCurrent = 0.0;
    public double macMiniCurrent = 0.0;
    public boolean brownedOut = false;
  }
}
