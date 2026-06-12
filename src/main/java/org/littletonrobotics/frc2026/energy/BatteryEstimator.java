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

/**
 * Battery state estimator using a single RC Thevenin model with Peukert correction. SOC is
 * determined from open-loop coulomb counting, which is then used to determine OCV, R0, and RP. VP
 * (polarization voltage) is determined using RC model and corrected from battery terminal voltage
 * measurements using Kalman gain. Model structure based on this paper
 * (https://doi.org/10.1016/j.ifacol.2022.06.031).
 *
 * <p>Battery parameters are for MK Powered battery based on this post
 * (https://www.chiefdelphi.com/t/detailed-frc-battery-comparison-for-2026/508077/22) fitted using
 * data from logs.
 */
public class BatteryEstimator {
  // Battery
  private static final double CAPACITY_AH = 19.75;
  private static final double CAPACITY_AS = CAPACITY_AH * 3600.0;

  // OCV(soc) = OCV_VOLTS[i] + t * (OCV_VOLTS[i + 1] - OCV_VOLTS[i])
  private static final double[] OCV_SOC_KNOTS = {0.0, 0.2, 0.4, 0.6, 0.8, 1.0};
  private static final double[] OCV_VOLTS = {12.112, 12.113, 12.311, 12.401, 12.535, 13.196};

  // R0(soc) = R0_BASE + R0_KNEE_GAIN * exp(-R0_KNEE_RATE * (R0_KNEE_SOC - soc))
  private static final double R0_BASE = 0.01181;
  private static final double R0_KNEE_GAIN = 0.00335;
  private static final double R0_KNEE_RATE = 1.36726;
  private static final double R0_KNEE_SOC = 0.4;

  // Single RC polarization
  private static final double RP_BASE = 0.00274;
  private static final double RP_SOC_SCALE = 2.10529;
  private static final double CP = 400.0;

  // Peukert (SOC-scaled)
  private static final double PEUKERT_BASE = 1.07;
  private static final double PEUKERT_SOC_SCALE = 0.1;
  private static final double I_NOMINAL = CAPACITY_AH / 20.0;

  // Variance parameters
  private static final double Q_VP = Math.pow(0.02, 2.0);
  private static final double R_VBATT = Math.pow(0.1, 2.0);
  private static final double R0_UNCERTAINTY = Math.pow(0.0015, 2.0);

  // State
  private double soc;
  private double vP;
  private double pVP;

  public BatteryEstimator() {
    setInitialVoltage(12.5, 0.0);
  }

  void setInitialVoltage(double initialVoltage, double initialCurrent) {
    double estimatedOcv = initialVoltage + initialCurrent * (R0_BASE + RP_BASE);
    soc = MathUtil.clamp(calculateSoc(estimatedOcv), 0.0, 1.0);
    vP = initialCurrent * getRp(soc);
    pVP = Q_VP;
  }

  double calculateMaxCurrent(double minVoltageThreshold) {
    return Math.max(0.0, (calculateOcv(soc) - vP - minVoltageThreshold) / getR0(soc));
  }

  /**
   * Update battery state estimate
   *
   * @param current Total current draw (Amperes).
   * @param measuredVoltage Terminal voltage from the PDP/PDH.
   */
  void update(double current, double measuredVoltage) {
    double dt = Constants.loopPeriodSecs;

    // Coloumb counting with peukert correction
    double effectiveAmps = current;
    if (current > I_NOMINAL) {
      double peukert = PEUKERT_BASE + PEUKERT_SOC_SCALE * (1.0 - soc);
      effectiveAmps = current * Math.pow(current / I_NOMINAL, peukert - 1.0);
    } else effectiveAmps = current;
    soc = MathUtil.clamp(soc - effectiveAmps * dt / CAPACITY_AS, 0.0, 1.0);

    // Update polarization voltage
    double rp = getRp(soc);
    // Calculate vP(dt) using RC dynamics: dV/dt = (I * R - V) / (R * C)
    double tau = rp * CP;
    double vP_inf = current * rp;
    double decay = Math.exp(-dt / tau);
    vP = vP_inf - (vP_inf - vP) * decay;
    // Calculate pVP-
    pVP = Math.pow(decay, 2.0) * pVP + Q_VP * dt;

    // Correct vP from voltage measurement
    double r0 = getR0(soc);
    double ocv = calculateOcv(soc);
    double predicted = ocv - current * r0 - vP;
    double K = pVP / (pVP + (R_VBATT + R0_UNCERTAINTY * current * current));
    vP += K * -(measuredVoltage - predicted);
    pVP *= (1.0 - K);

    // Log
    double corrected = calculateOcv(soc) - (current * getR0(soc)) - vP;
    Logger.recordOutput("BatteryModel/EstimatedVoltage", corrected, "volts");
    Logger.recordOutput("BatteryModel/OCV", calculateOcv(soc), "volts");
    Logger.recordOutput("BatteryModel/SOC", soc);
    Logger.recordOutput("BatteryModel/VP", vP, "volts");
    Logger.recordOutput("BatteryModel/P_VP", pVP);
  }

  private static double getR0(double soc) {
    return R0_BASE + R0_KNEE_GAIN * Math.exp(R0_KNEE_RATE * (R0_KNEE_SOC - soc));
  }

  private static double getRp(double soc) {
    double d = 1.0 - soc;
    return RP_BASE * (1.0 + RP_SOC_SCALE * d * d);
  }

  private static double calculateOcv(double soc) {
    soc = MathUtil.clamp(soc, 0.0, 1.0);
    for (int i = 0; i < OCV_SOC_KNOTS.length - 1; i++) {
      if (soc <= OCV_SOC_KNOTS[i + 1]) {
        double t = (soc - OCV_SOC_KNOTS[i]) / (OCV_SOC_KNOTS[i + 1] - OCV_SOC_KNOTS[i]);
        return OCV_VOLTS[i] + t * (OCV_VOLTS[i + 1] - OCV_VOLTS[i]);
      }
    }
    return OCV_VOLTS[OCV_VOLTS.length - 1];
  }

  private static double calculateSoc(double ocv) {
    ocv = MathUtil.clamp(ocv, OCV_VOLTS[0], OCV_VOLTS[OCV_VOLTS.length - 1]);
    for (int i = 0; i < OCV_VOLTS.length - 1; i++) {
      if (ocv <= OCV_VOLTS[i + 1]) {
        double t = (ocv - OCV_VOLTS[i]) / (OCV_VOLTS[i + 1] - OCV_VOLTS[i]);
        return OCV_SOC_KNOTS[i] + t * (OCV_SOC_KNOTS[i + 1] - OCV_SOC_KNOTS[i]);
      }
    }
    return OCV_SOC_KNOTS[OCV_SOC_KNOTS.length - 1];
  }
}
