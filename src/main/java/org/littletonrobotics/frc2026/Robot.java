// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2026;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.math.MathShared;
import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.MathUsageId;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.IterativeRobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Watchdog;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import java.lang.reflect.Field;
import java.net.InetAddress;
import java.net.UnknownHostException;
import java.util.HashMap;
import java.util.Map;
import java.util.function.BiConsumer;
import org.littletonrobotics.frc2026.Constants.Mode;
import org.littletonrobotics.frc2026.Constants.RobotType;
import org.littletonrobotics.frc2026.energy.FinanceDepartment;
import org.littletonrobotics.frc2026.subsystems.launcher.LaunchCalculator;
import org.littletonrobotics.frc2026.util.FullSubsystem;
import org.littletonrobotics.frc2026.util.HubShiftUtil;
import org.littletonrobotics.frc2026.util.LoggedTracer;
import org.littletonrobotics.frc2026.util.VirtualSubsystem;
import org.littletonrobotics.junction.AutoLogOutputManager;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

public class Robot extends LoggedRobot {
  private Command autonomousCommand;
  private double autoStart;
  private boolean autoMessagePrinted;
  private RobotContainer robotContainer;

  private final Timer fuelLoggingTimer = new Timer();
  private static final Timer disabledTimer = new Timer();

  public Robot() {
    super(Constants.loopPeriodSecs);

    // Record metadata
    Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    Logger.recordMetadata(
        "GitDirty",
        switch (BuildConstants.DIRTY) {
          case 0 -> "All changes committed";
          case 1 -> "Uncommitted changes";
          default -> "Unknown";
        });
    try {
      Logger.recordMetadata(
          "Hostname", InetAddress.getLocalHost().getHostName().replaceAll("\\.local$", ""));
    } catch (UnknownHostException e) {
      Logger.recordMetadata("Hostname", "Unknown");
    }
    Logger.recordMetadata(
        "Platform",
        "%s %s (%s)"
            .formatted(
                System.getProperty("os.name").replace(" ", ""),
                System.getProperty("os.version"),
                System.getProperty("os.arch")));

    // Set up data receivers & replay source
    switch (Constants.getMode()) {
      case REAL:
        Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case SIM:
        // Running a physics simulator, log to NT
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case REPLAY:
        // Replaying a log, set up replay source
        String inPath = LogFileUtil.findReplayLog();
        String outPath = LogFileUtil.addPathSuffix(inPath, "_sim");
        Logger.setReplaySource(new WPILOGReader(inPath));
        Logger.addDataReceiver(new WPILOGWriter(outPath));
        break;
    }

    // Set timing mode
    setUseTiming(Constants.getMode() != Mode.REPLAY);

    // Start AdvantageKit logger
    Logger.start();

    // Adjust loop overrun warning timeout
    try {
      Field watchdogField = IterativeRobotBase.class.getDeclaredField("m_watchdog");
      watchdogField.setAccessible(true);
      Watchdog watchdog = (Watchdog) watchdogField.get(this);
      watchdog.setTimeout(Constants.loopPeriodWatchdogSecs);
    } catch (Exception e) {
      DriverStation.reportWarning("Failed to disable loop overrun warnings.", false);
    }
    CommandScheduler.getInstance().setPeriod(Constants.loopPeriodWatchdogSecs);

    // Silence joystick alerts
    DriverStation.silenceJoystickConnectionWarning(true);

    // Silence Rotation2d warnings
    var mathShared = MathSharedStore.getMathShared();
    MathSharedStore.setMathShared(
        new MathShared() {
          @Override
          public void reportError(String error, StackTraceElement[] stackTrace) {
            if (error.startsWith("x and y components of Rotation2d are zero")) {
              return;
            }
            mathShared.reportError(error, stackTrace);
          }

          @Override
          public void reportUsage(MathUsageId id, int count) {
            mathShared.reportUsage(id, count);
          }

          @Override
          public double getTimestamp() {
            return mathShared.getTimestamp();
          }
        });

    // Log active commands
    Map<String, Integer> commandCounts = new HashMap<>();
    BiConsumer<Command, Boolean> logCommandFunction =
        (Command command, Boolean active) -> {
          String name = command.getName();
          int count = commandCounts.getOrDefault(name, 0) + (active ? 1 : -1);
          commandCounts.put(name, count);
          Logger.recordOutput(
              "CommandsUnique/" + name + "_" + Integer.toHexString(command.hashCode()), active);
          Logger.recordOutput("CommandsAll/" + name, count > 0);
        };
    CommandScheduler.getInstance()
        .onCommandInitialize((Command command) -> logCommandFunction.accept(command, true));
    CommandScheduler.getInstance()
        .onCommandFinish((Command command) -> logCommandFunction.accept(command, false));
    CommandScheduler.getInstance()
        .onCommandInterrupt((Command command) -> logCommandFunction.accept(command, false));

    // Configure Driver Station for sim
    RoboRioSim.setTeamNumber(6328);
    if (Constants.getRobot() == RobotType.SIMBOT) {
      DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
      DriverStationSim.notifyNewData();
    }

    // Reset disabled timer
    disabledTimer.restart();

    // Set up auto logging for RobotState
    AutoLogOutputManager.addObject(RobotState.getInstance());

    // Start tracking of battery in management
    FinanceDepartment.getInstance().reset();

    // Instantiate RobotContainer
    robotContainer = new RobotContainer();
  }

  /** This function is called periodically during all modes. */
  @Override
  public void robotPeriodic() {

    // Reset logged tracer
    LoggedTracer.reset();

    // Main periodic functions
    VirtualSubsystem.runAllPeriodic();
    CommandScheduler.getInstance().run();
    LoggedTracer.record("Robot/Commands");
    VirtualSubsystem.runAllPeriodicAfterScheduler();
    FullSubsystem.runAllPeriodicAfterScheduler();
    LoggedTracer.record("Robot/AfterScheduler");

    // Reset tracking of battery
    if (DriverStation.isDisabled() && disabledTimer.hasElapsed(10.0)) {
      FinanceDepartment.getInstance().reset();
    }

    // Reset disabled timer
    if (DriverStation.isEnabled()) {
      disabledTimer.restart();
    }

    // Process throttle state
    Logger.recordOutput("Throttled", shouldThrottle());

    // Update RobotContainer dashboard outputs
    robotContainer.updateDashboardOutputs();

    // Log Mechanism3d data
    DarwinMechanism3d.getMeasured().log("Mechanism3d");

    // Update fuel sim
    if (Constants.getMode() == Mode.SIM) {
      robotContainer.updateFuelSim();
    }

    // Log fuel state
    fuelLoggingTimer.start();
    if (fuelLoggingTimer.advanceIfElapsed(0.05) && Constants.getMode() != Mode.REAL) {
      Logger.recordOutput(
          "ObjectDetection/FuelTranslations",
          ObjectDetection.getInstance().getFuelTranslations().stream()
              .map(
                  (translation) ->
                      new Translation3d(
                          translation.getX(),
                          translation.getY(),
                          FieldConstants.fuelDiameter / 2.0))
              .toArray(Translation3d[]::new));
    }

    // Log robots
    Logger.recordOutput(
        "ObjectDetection/RobotPoses",
        ObjectDetection.getInstance().getRobotTranslations().stream()
            .map(
                (translation) ->
                    new Pose3d(
                        new Translation3d(translation.getX(), translation.getY(), 0.0),
                        new Rotation3d(new Rotation2d(Timer.getTimestamp() * 5.0))))
            .toArray(Pose3d[]::new));

    // Log hub state
    Logger.recordOutput("HubShift/Official", HubShiftUtil.getOfficialShiftInfo());
    Logger.recordOutput("HubShift/Shifted", HubShiftUtil.getShiftedShiftInfo());

    // Log launching parameters
    var launchCalculator = LaunchCalculator.getInstance();
    Logger.recordOutput("LaunchCalculator/Parameters", launchCalculator.getParameters());
    Logger.recordOutput(
        "LaunchCalculator/FlywheelSpeedOffsetRadsPerSec",
        launchCalculator.getFlywheelSpeedOffsetRadsPerSec());
    SmartDashboard.putString(
        "Flywheel Speed Offset",
        Integer.toString((int) Math.round(launchCalculator.getFlywheelSpeedOffsetRadsPerSec())));

    // Clear launching parameters
    launchCalculator.clearLaunchingParameters();

    // Record cycle time
    LoggedTracer.record("Robot/Periodic");
  }

  /** Returns whether performance should be throttled to conserve power. */
  public static boolean shouldThrottle() {
    return disabledTimer.hasElapsed(5.0);
  }

  /** Whether to display alerts related to hardware faults. */
  public static boolean showHardwareAlerts() {
    return Constants.getMode() != Mode.SIM && Timer.getTimestamp() > 30.0;
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    autoStart = Timer.getTimestamp();
    autoMessagePrinted = false;
    autonomousCommand = robotContainer.getAutonomousCommand();

    if (autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(autonomousCommand);
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
