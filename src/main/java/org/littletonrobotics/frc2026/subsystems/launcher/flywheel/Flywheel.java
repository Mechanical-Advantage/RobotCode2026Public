// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2026.subsystems.launcher.flywheel;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.LinkedList;
import java.util.List;
import java.util.function.DoubleSupplier;
import lombok.Getter;
import lombok.Setter;
import lombok.experimental.Accessors;
import org.littletonrobotics.frc2026.Constants;
import org.littletonrobotics.frc2026.Constants.Mode;
import org.littletonrobotics.frc2026.Robot;
import org.littletonrobotics.frc2026.subsystems.launcher.LaunchCalculator;
import org.littletonrobotics.frc2026.subsystems.launcher.flywheel.FlywheelIO.FlywheelIOOutputMode;
import org.littletonrobotics.frc2026.subsystems.launcher.flywheel.FlywheelIO.FlywheelIOOutputs;
import org.littletonrobotics.frc2026.util.EqualsUtil;
import org.littletonrobotics.frc2026.util.FullSubsystem;
import org.littletonrobotics.frc2026.util.LoggedTracer;
import org.littletonrobotics.frc2026.util.LoggedTunableNumber;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Flywheel extends FullSubsystem {
  private final FlywheelIO io;
  private final FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();
  private final FlywheelIOOutputs outputs = new FlywheelIOOutputs();

  private final Debouncer motorConnectedDebouncer =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final Debouncer motorFollower1ConnectedDebouncer =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final Debouncer motorFollower2ConnectedDebouncer =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final Debouncer motorFollower3ConnectedDebouncer =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final Alert disconnected;
  private final Alert follower1Disconnected;
  private final Alert follower2Disconnected;
  private final Alert follower3Disconnected;

  private static final DCMotor gearbox = DCMotor.getKrakenX60Foc(4); // Reduction of 1.0
  private static final double moi = 0.02; // kg*m^2
  private static final double efficiency = 0.8; // W(out)/W(in)
  private static double accelFilterTimeConstant = 0.050; // Seconds

  private static final double ffStartDelay = 0.5; // Secs
  private static final double ffRampRate = 0.2; // Volts/Sec

  private static final LoggedTunableNumber bangBangConstant =
      new LoggedTunableNumber("Flywheel/BangBangConstant", 2.0);
  private static final LoggedTunableNumber bangBangMinDistance =
      new LoggedTunableNumber("Flywheel/BangBangMinDistance", 3.5);
  private static final LoggedTunableNumber pidSetpointOffset =
      new LoggedTunableNumber(
          "Flywheel/PIDSetpointOffset", 2.0); // Offset up when not running bang-bang
  private static final LoggedTunableNumber kP = new LoggedTunableNumber("Flywheel/kP", 0.6);
  private static final LoggedTunableNumber kD = new LoggedTunableNumber("Flywheel/kD", 0.0);
  private static final LoggedTunableNumber kS = new LoggedTunableNumber("Flywheel/kS", 0.28);
  private static final LoggedTunableNumber kV = new LoggedTunableNumber("Flywheel/kV", 0.0192);
  private static final LoggedTunableNumber kA = new LoggedTunableNumber("Flywheel/kA", 0.007);
  private static final LoggedTunableNumber kSTolerance =
      new LoggedTunableNumber("Flywheel/CharacterizationTolerance/kS", 0.15);
  private static final LoggedTunableNumber kVTolerance =
      new LoggedTunableNumber("Flywheel/CharacterizationTolerance/kV", 0.002);
  private static final LoggedTunableNumber maxAcceleration =
      new LoggedTunableNumber("Flywheel/MaxAccelerationRadPerSec2", 350.0);
  private static final LoggedTunableNumber supplyLimitTeleop =
      new LoggedTunableNumber("Flywheel/SupplyLimitTeleopAmps", 160.0);
  private static final LoggedTunableNumber supplyLimitTeleopConservative =
      new LoggedTunableNumber("Flywheel/SupplyLimitTeleopConservativeAmps", 50.0);
  private static final LoggedTunableNumber supplyLimitAuto =
      new LoggedTunableNumber("Flywheel/SupplyLimitAutoAmps", 60.0);

  @Getter private boolean withinTolerancekS = true;
  @Getter private boolean withinTolerancekV = true;

  private double goalVel = 0.0;
  private double supplyLimit = 200.0;
  private double setpointVel = 0.0;
  private double filteredAccel = 0.0; // Used for kA
  private boolean nonZeroAccel = false;

  @Setter private boolean sportMode = false;

  @Getter
  @Accessors(fluent = true)
  @AutoLogOutput(key = "Flywheel/AtGoal")
  private boolean atGoal = false;

  public Flywheel(FlywheelIO io) {
    this.io = io;

    disconnected = new Alert("Flywheel motor disconnected!", Alert.AlertType.kError);
    follower1Disconnected =
        new Alert("Flywheel follower 1 motor disconnected!", Alert.AlertType.kError);
    follower2Disconnected =
        new Alert("Flywheel follower 2 motor disconnected!", Alert.AlertType.kError);
    follower3Disconnected =
        new Alert("Flywheel follower 3 motor disconnected!", Alert.AlertType.kError);
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Flywheel", inputs);

    outputs.kP = kP.get();
    outputs.kD = kD.get();

    if (DriverStation.isDisabled()) {
      stop();
    }

    if (DriverStation.isAutonomous()) {
      supplyLimit = supplyLimitAuto.get();
    } else {
      supplyLimit = sportMode ? supplyLimitTeleopConservative.get() : supplyLimitTeleop.get();
    }

    disconnected.set(
        Robot.showHardwareAlerts() && !motorConnectedDebouncer.calculate(inputs.connected));
    follower1Disconnected.set(
        Robot.showHardwareAlerts()
            && !motorFollower1ConnectedDebouncer.calculate(inputs.follower1Connected));
    follower2Disconnected.set(
        Robot.showHardwareAlerts()
            && !motorFollower2ConnectedDebouncer.calculate(inputs.follower2Connected));
    follower3Disconnected.set(
        Robot.showHardwareAlerts()
            && !motorFollower3ConnectedDebouncer.calculate(inputs.follower3Connected));

    // Report energy usage
    Robot.batteryLogger.reportCurrentUsage(
        "Flywheel",
        false,
        inputs.connected ? inputs.supplyCurrentAmps : 0.0,
        inputs.follower1Connected ? inputs.follower1SupplyCurrentAmps : 0.0,
        inputs.follower2Connected ? inputs.follower2SupplyCurrentAmps : 0.0,
        inputs.follower3Connected ? inputs.follower3SupplyCurrentAmps : 0.0);

    SmartDashboard.putString("Flywheel Speed", String.format("%.0f", inputs.velocityRadsPerSec));
    SmartDashboard.putBoolean("Flywheel At Goal", atGoal);
    LoggedTracer.record("Flywheel/Periodic");
  }

  @Override
  public void periodicAfterScheduler() {
    Logger.recordOutput("Flywheel/Mode", outputs.mode);
    io.applyOutputs(outputs);

    LoggedTracer.record("Flywheel/AfterScheduler");
  }

  /** Run closed loop at the specified velocity. */
  private void runVelocity(double velocityRadsPerSec, boolean bangBang) {
    // Calculate power budget from supply limit
    double vBus = Robot.batteryLogger.getBatteryVoltage();
    if (Constants.getMode() == Mode.SIM) {
      vBus = 10.0;
    }
    double powerBudget = supplyLimit * vBus * efficiency;
    double backEmf = setpointVel / gearbox.KvRadPerSecPerVolt;

    // Solve for max stator current from power budget
    // I_sator x V_motor - P = 0
    // I_stator x (I_stator x R - backEmf) - P = 0
    // R x I_stator^2 - backEmf x I_stator - P = 0
    double maxStatorCurrent =
        (-backEmf + Math.sqrt(backEmf * backEmf + 4.0 * gearbox.rOhms * powerBudget))
            / (2.0 * gearbox.rOhms);
    double voltageLimitedCurrent = Math.max(0.0, (vBus - backEmf) / gearbox.rOhms);
    maxStatorCurrent = Math.min(maxStatorCurrent, voltageLimitedCurrent);

    // Max acceleration from max stator current
    double maxAccelFromCurrent =
        (gearbox.getTorque(maxStatorCurrent) - gearbox.getTorque(kS.get() / gearbox.rOhms)) / moi;
    maxAccelFromCurrent = MathUtil.clamp(maxAccelFromCurrent, 0.0, maxAcceleration.get());

    // Rate-limit velocity setpoint
    double maxStep = maxAccelFromCurrent * Constants.loopPeriodSecs;
    double error = velocityRadsPerSec - setpointVel;
    double rawAccel;
    if (Math.abs(error) <= maxStep) {
      setpointVel = velocityRadsPerSec;
      rawAccel = error / Constants.loopPeriodSecs;
    } else {
      setpointVel += Math.copySign(maxStep, error);
      rawAccel = Math.copySign(maxAccelFromCurrent, error);
    }
    if (!nonZeroAccel) {
      filteredAccel = rawAccel;
    } else {
      filteredAccel +=
          (rawAccel - filteredAccel) * Constants.loopPeriodSecs / accelFilterTimeConstant;
    }
    nonZeroAccel = true;

    // Apply outputs
    atGoal = EqualsUtil.epsilonEquals(setpointVel, velocityRadsPerSec, 2.0);
    goalVel = velocityRadsPerSec;
    outputs.mode = bangBang ? FlywheelIOOutputMode.VOLTAGE : FlywheelIOOutputMode.VELOCITY;
    outputs.velocityRadsPerSec = setpointVel;
    outputs.voltage =
        Math.signum(setpointVel) * kS.get() + setpointVel * kV.get() + filteredAccel * kA.get();
    if (bangBang) {
      if (inputs.velocityRadsPerSec < setpointVel) {
        outputs.voltage *= bangBangConstant.get();
      }
    } else {
      outputs.velocityRadsPerSec += pidSetpointOffset.get();
    }

    Logger.recordOutput("Flywheel/Setpoint", setpointVel);
    Logger.recordOutput("Flywheel/SetpointAccel", filteredAccel);
    Logger.recordOutput("Flywheel/Goal", velocityRadsPerSec);
    Logger.recordOutput("Flywheel/Feedforward", outputs.voltage);
    Logger.recordOutput("Flywheel/BangBang", bangBang);
  }

  private void runVoltage(double voltage) {
    outputs.mode = FlywheelIOOutputMode.VOLTAGE;
    outputs.voltage = voltage;
  }

  /** Stops the flywheel. */
  private void stop() {
    outputs.mode = FlywheelIOOutputMode.COAST;
    outputs.velocityRadsPerSec = 0.0;
    atGoal = false;
    setpointVel = getVelocity();
  }

  /** Returns the current velocity in RPM. */
  public double getVelocity() {
    return inputs.velocityRadsPerSec;
  }

  public boolean withinTolerance(double toleranceRadPerSec) {
    return Math.abs(inputs.velocityRadsPerSec - goalVel) < toleranceRadPerSec;
  }

  public Command feedforwardCharacterizationCommand() {
    List<Double> velocitySamples = new LinkedList<>();
    List<Double> voltageSamples = new LinkedList<>();
    Timer timer = new Timer();

    return Commands.sequence(
        // Reset data
        Commands.runOnce(
            () -> {
              velocitySamples.clear();
              voltageSamples.clear();
              withinTolerancekS = true;
              withinTolerancekV = true;
            }),

        // Make sure flywheel is not moving
        stopCommand().withTimeout(ffStartDelay),

        // Start timer
        Commands.runOnce(timer::restart),

        // Accelerate and gather data
        Commands.run(
                () -> {
                  double voltage = timer.get() * ffRampRate;
                  runVoltage(voltage);
                  velocitySamples.add(inputs.velocityRadsPerSec);
                  voltageSamples.add(voltage);
                },
                this)

            // When cancelled, calculate and print results
            .finallyDo(
                () -> {
                  int n = velocitySamples.size();
                  double sumX = 0.0;
                  double sumY = 0.0;
                  double sumXY = 0.0;
                  double sumX2 = 0.0;
                  for (int i = 0; i < n; i++) {
                    sumX += velocitySamples.get(i);
                    sumY += voltageSamples.get(i);
                    sumXY += velocitySamples.get(i) * voltageSamples.get(i);
                    sumX2 += velocitySamples.get(i) * velocitySamples.get(i);
                  }
                  double ffkS = (sumY * sumX2 - sumX * sumXY) / (n * sumX2 - sumX * sumX);
                  double ffkV = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);

                  withinTolerancekS = EqualsUtil.epsilonEquals(ffkS, kS.get(), kSTolerance.get());
                  withinTolerancekV = EqualsUtil.epsilonEquals(ffkV, kV.get(), kVTolerance.get());

                  NumberFormat formatter = new DecimalFormat("#0.00000");
                  System.out.println("********** Flywheel FF Characterization Results **********");
                  System.out.println("\tkS: " + formatter.format(ffkS));
                  System.out.println("\tkV: " + formatter.format(ffkV));
                  System.out.println("\tCurrent kS: " + formatter.format(kS.get()));
                  System.out.println("\tCurrent kV: " + formatter.format(kV.get()));
                }));
  }

  public Command runTrackTargetCommand() {
    return runEnd(
        () ->
            runVelocity(
                LaunchCalculator.getInstance().getParameters().flywheelSpeed(),
                LaunchCalculator.getInstance().getParameters().distanceNoLookahead()
                        > bangBangMinDistance.get()
                    && !LaunchCalculator.getInstance().getParameters().passing()),
        this::stop);
  }

  public Command runFixedCommand(DoubleSupplier velocity, boolean bangBang) {
    return runEnd(() -> runVelocity(velocity.getAsDouble(), bangBang), this::stop);
  }

  public Command stopCommand() {
    return run(this::stop);
  }
}
