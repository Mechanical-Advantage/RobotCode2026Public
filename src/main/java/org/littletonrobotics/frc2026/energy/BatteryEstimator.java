// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2026.energy;

import edu.wpi.first.math.MathUtil;
import org.littletonrobotics.frc2026.Constants;
import org.littletonrobotics.junction.Logger;

public class BatteryEstimator {
  // Battery
  private static final double CAPACITY_AS = 18.0 * 3600.0;

  // R0(soc) = R0_BASE + R0_KNEE_GAIN * exp(-R0_KNEE_RATE * soc)
  private static final double R0_BASE = 0.012;
  private static final double R0_KNEE_GAIN = 0.005;
  private static final double R0_KNEE_RATE = 2.0;
  private static final double R0_KNEE_SOC = 0.4;

  // RC polarization
  private static final double RP_BASE = 0.003;
  private static final double RP_SOC_SCALE = 3.0;
  private static final double CP = 400.0;

  // Peukert (SOC-scaled)
  private static final double PEUKERT_BASE = 1.05;
  private static final double PEUKERT_SOC_SCALE = 0.1;
  private static final double I_NOMINAL = 0.9;

  // Voltage feedback correction — steady-state Kalman gain form:
  //   gain = MAX_GAIN / (1 + (I / NOISE_SCALE)^2)
  // MAX_GAIN: Max correction strength for V_p
  // NOISE_SCALE: Current (amps) at which gain drops to half.
  private static final double MAX_GAIN = 0.2;
  private static final double NOISE_SCALE = 100.0;

  // State
  private double soc;
  private double vP;

  public BatteryEstimator() {
    setInitialVoltage(12.5, 0.0);
  }

  void setInitialVoltage(double initialVoltage, double initialCurrent) {
    double estimatedOcv = initialVoltage + (initialCurrent * R0_BASE) + (initialCurrent * RP_BASE);
    soc = MathUtil.clamp(calculateSoc(estimatedOcv), 0.0, 1.0);
    vP = initialCurrent * getRp(soc);
  }

  double calculateMaxCurrent(double minVoltageThreshold, double budgetPeriodSecs) {
    double r0 = getR0(soc);
    double maxCurrent = (calculateOcv(soc) - vP - minVoltageThreshold) / r0;
    double derating = 1.0 - budgetPeriodSecs / (r0 * CP);
    return Math.max(0.0, maxCurrent * derating);
  }

  /**
   * Runs one estimator cycle: propagate state with physics, then correct from measured voltage.
   *
   * @param current Total current draw (Amperes).
   * @param measuredVoltage Terminal voltage from the PDP/PDH.
   */
  void update(double current, double measuredVoltage) {
    double dt = Constants.loopPeriodSecs;
    // Predict
    // SOC (coulomb counting with Peukert correction)
    double effectiveAmps = current;
    if (current > I_NOMINAL) {
      double peukert = PEUKERT_BASE + PEUKERT_SOC_SCALE * (1.0 - soc);
      effectiveAmps = current * Math.pow(current / I_NOMINAL, peukert - 1.0);
    }
    soc = MathUtil.clamp(soc - (effectiveAmps * dt) / CAPACITY_AS, 0.0, 1.0);

    // Polarization voltage (exact RC step for constant current over dt)
    vP = stepRc(vP, current, getRp(soc), dt);

    // Correct:
    double predicted = calculateOcv(soc) - (current * getR0(soc)) - vP;
    double error = measuredVoltage - predicted;
    double gain = MAX_GAIN / (1.0 + (current / NOISE_SCALE) * (current / NOISE_SCALE));
    vP -= error * gain;

    // Log state
    double corrected = calculateOcv(soc) - (current * getR0(soc)) - vP;
    Logger.recordOutput("BatteryModel/PredictedVoltage", corrected, "volts");
    Logger.recordOutput("BatteryModel/OCV", calculateOcv(soc), "volts");
    Logger.recordOutput("BatteryModel/SOC", soc);
    Logger.recordOutput("BatteryModel/VP", vP, "volts");
    Logger.recordOutput("BatteryModel/CorrectionGain", gain);
    Logger.recordOutput("BatteryModel/PredictionError", error, "volts");
  }

  private static double stepRc(double vP, double current, double rp, double dt) {
    double vpSteady = current * rp;
    double decay = Math.exp(-dt / (rp * CP));
    return vpSteady + (vP - vpSteady) * decay;
  }

  private static double getR0(double soc) {
    return R0_BASE + R0_KNEE_GAIN * Math.exp(R0_KNEE_RATE * (R0_KNEE_SOC - soc));
  }

  private static double getRp(double soc) {
    double d = 1.0 - soc;
    return RP_BASE * (1.0 + RP_SOC_SCALE * d * d);
  }

  private static double calculateOcv(double soc) {
    if (soc > 0.8) return 12.6 + ((soc - 0.8) / 0.2) * 0.4;
    if (soc > 0.2) return 12.0 + ((soc - 0.2) / 0.6) * 0.6;
    return 11.5 + (soc / 0.2) * 0.5;
  }

  private static double calculateSoc(double ocv) {
    if (ocv > 12.6) return ((ocv - 12.6) / 0.4) * 0.2 + 0.8;
    if (ocv > 12.0) return ocv - 11.8;
    return ((ocv - 11.5) / 0.5) * 0.2;
  }
}
