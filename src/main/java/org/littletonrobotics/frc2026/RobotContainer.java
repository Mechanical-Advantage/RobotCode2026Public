// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2026;

import choreo.Choreo;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.io.File;
import java.lang.reflect.Method;
import java.util.List;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import lombok.Getter;
import lombok.Setter;
import lombok.experimental.ExtensionMethod;
import org.littletonrobotics.frc2026.AutoSelector.AutoQuestion;
import org.littletonrobotics.frc2026.AutoSelector.AutoQuestionResponse;
import org.littletonrobotics.frc2026.Constants.Mode;
import org.littletonrobotics.frc2026.FieldConstants.AprilTagLayoutType;
import org.littletonrobotics.frc2026.commands.CompactingCommands;
import org.littletonrobotics.frc2026.commands.DriveCommands;
import org.littletonrobotics.frc2026.commands.auto.AutoBuilder;
import org.littletonrobotics.frc2026.subsystems.drive.Drive;
import org.littletonrobotics.frc2026.subsystems.drive.DriveConstants;
import org.littletonrobotics.frc2026.subsystems.drive.GyroIO;
import org.littletonrobotics.frc2026.subsystems.drive.ModuleIO;
import org.littletonrobotics.frc2026.subsystems.drive.ModuleIOSim;
import org.littletonrobotics.frc2026.subsystems.hopper.Hopper;
import org.littletonrobotics.frc2026.subsystems.hubcounter.HubCounter;
import org.littletonrobotics.frc2026.subsystems.kicker.Kicker;
import org.littletonrobotics.frc2026.subsystems.launcher.LaunchCalculator;
import org.littletonrobotics.frc2026.subsystems.launcher.flywheel.Flywheel;
import org.littletonrobotics.frc2026.subsystems.launcher.flywheel.FlywheelIO;
import org.littletonrobotics.frc2026.subsystems.launcher.hood.Hood;
import org.littletonrobotics.frc2026.subsystems.launcher.hood.HoodIO;
import org.littletonrobotics.frc2026.subsystems.leds.Leds;
import org.littletonrobotics.frc2026.subsystems.leds.LedsIO;
import org.littletonrobotics.frc2026.subsystems.leds.LedsIOHAL;
import org.littletonrobotics.frc2026.subsystems.rollers.RollerSystemIO;
import org.littletonrobotics.frc2026.subsystems.rollers.RollerSystemIOSim;
import org.littletonrobotics.frc2026.subsystems.slamtake.SlamIO;
import org.littletonrobotics.frc2026.subsystems.slamtake.SlamIOSim;
import org.littletonrobotics.frc2026.subsystems.slamtake.Slamtake;
import org.littletonrobotics.frc2026.subsystems.slamtake.Slamtake.IntakeGoal;
import org.littletonrobotics.frc2026.subsystems.slamtake.Slamtake.SlamGoal;
import org.littletonrobotics.frc2026.subsystems.vision.Vision;
import org.littletonrobotics.frc2026.subsystems.vision.VisionIO;
import org.littletonrobotics.frc2026.util.BeachedUtil;
import org.littletonrobotics.frc2026.util.ContinuousConditionalCommand;
import org.littletonrobotics.frc2026.util.FuelSim;
import org.littletonrobotics.frc2026.util.HubShiftUtil;
import org.littletonrobotics.frc2026.util.controllers.OverrideSwitches;
import org.littletonrobotics.frc2026.util.controllers.RazerWolverineController;
import org.littletonrobotics.frc2026.util.controllers.TriggerUtil;
import org.littletonrobotics.frc2026.util.geometry.AllianceFlipUtil;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

@ExtensionMethod({TriggerUtil.class})
public class RobotContainer {
  // Subsystems
  private Drive drive;
  private Slamtake slamtake;
  private Hopper hopper;
  private Kicker kicker;
  private Hood hood;
  private Flywheel flywheel;
  private Vision vision;
  private Leds leds;
  private HubCounter hubCounter = new HubCounter();

  // Controllers
  private final RazerWolverineController primary = new RazerWolverineController(0);
  private final CommandXboxController secondary = new CommandXboxController(1);
  private final OverrideSwitches overrides = new OverrideSwitches(5);

  // Driver overrides
  private final Trigger robotRelative = overrides.driverSwitch(1);
  private final Trigger coast = overrides.driverSwitch(2);
  private final Trigger lostAutoOverride = overrides.multiDirectionSwitchLeft();
  private final Trigger wonAutoOverride = overrides.multiDirectionSwitchRight();

  // Operator overrides
  private final Trigger disableAutoSpinup = overrides.operatorSwitch(0);
  private final Trigger ignoreHubState = overrides.operatorSwitch(1);
  private final Trigger noTiltCheck = overrides.operatorSwitch(2);
  private final Trigger sportModeOverride = overrides.operatorSwitch(3);

  // Alerts
  private final Alert primaryDisconnected =
      new Alert("Primary controller disconnected (port 0).", AlertType.kWarning);
  private final Alert secondaryDisconnected =
      new Alert("Secondary controller disconnected (port 1).", AlertType.kWarning);
  private final Alert overrideDisconnected =
      new Alert("Override controller disconnected (port 5).", AlertType.kInfo);
  private final Alert autoWinnerNotSet = new Alert("!!! AUTO WINNER NOT SET !!!", AlertType.kError);
  private final Alert aprilTagLayoutAlert = new Alert("", AlertType.kInfo);

  // Dashboard inputs and outputs
  private final AutoSelector autoSelector = new AutoSelector("Auto");
  private final LoggedDashboardChooser<AprilTagLayoutType> aprilTagLayoutChooser;
  private final LoggedNetworkNumber offsetTime =
      new LoggedNetworkNumber("/SmartDashboard/Auto/Offset Time?", 0.0);

  private boolean coastOverride = false;

  /** Keeps track of the number of balls in the hopper with the fuel sim. */
  public class SimFuelCount {
    @Getter private static final int capacity = 80;
    @Getter private static final double launchBPS = 16.0;

    @Setter @Getter private int fuelStored;

    public SimFuelCount(int fuelStored) {
      this.fuelStored = fuelStored;
    }
  }

  private FuelSim fuelSim;
  private SimFuelCount simFuelCount;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure fuel sim
    if (Constants.getMode() == Mode.SIM) {
      fuelSim = new FuelSim("FuelSim");
      simFuelCount = new SimFuelCount(8);
      ObjectDetection.setFuelSim(fuelSim);
      configureFuelSim();
    }

    // Instantiate subsystems
    if (Constants.getMode() != Constants.Mode.REPLAY) {
      // https://www.chiefdelphi.com/t/frc-6328-mechanical-advantage-2026-build-thread/509595/616
      switch (Constants.getRobot()) {
        case DARWIN:
          // Not implemented
          break;

        case ALPHABOT:
          // Not implemented
          break;

        case SIMBOT:
          drive =
              new Drive(
                  new GyroIO() {},
                  new GyroIO() {},
                  new ModuleIOSim(0),
                  new ModuleIOSim(1),
                  new ModuleIOSim(2),
                  new ModuleIOSim(3));
          slamtake =
              new Slamtake(
                  new SlamIOSim(),
                  new RollerSystemIOSim(DCMotor.getKrakenX60Foc(1), 1.0, 0.005, false));
          hopper =
              new Hopper(
                  new RollerSystemIOSim(DCMotor.getKrakenX60Foc(2), 4.0, 0.005, true),
                  Optional.of(simFuelCount));
          kicker =
              new Kicker(
                  new RollerSystemIO() {}, new RollerSystemIO() {}, Optional.of(simFuelCount));
          leds = new Leds(new LedsIOHAL());
          break;
      }
    }

    // No-op implementations for replay
    if (drive == null) {
      drive =
          new Drive(
              new GyroIO() {},
              new GyroIO() {},
              new ModuleIO() {},
              new ModuleIO() {},
              new ModuleIO() {},
              new ModuleIO() {});
    }
    if (slamtake == null) {
      slamtake = new Slamtake(new SlamIO() {}, new RollerSystemIO() {});
    }
    if (hopper == null) {
      hopper = new Hopper(new RollerSystemIO() {}, Optional.empty());
    }
    if (hood == null) {
      hood = new Hood(new HoodIO() {});
    }
    if (flywheel == null) {
      flywheel = new Flywheel(new FlywheelIO() {});
    }
    if (kicker == null) {
      kicker = new Kicker(new RollerSystemIO() {}, new RollerSystemIO() {}, Optional.empty());
    }
    if (vision == null) {
      switch (Constants.getRobot()) {
        case DARWIN ->
            vision =
                new Vision(
                    this::getSelectedAprilTagLayout,
                    new VisionIO() {},
                    new VisionIO() {},
                    new VisionIO() {},
                    new VisionIO() {});
        case ALPHABOT ->
            vision =
                new Vision(this::getSelectedAprilTagLayout, new VisionIO() {}, new VisionIO() {});
        default -> vision = new Vision(this::getSelectedAprilTagLayout);
      }
    }
    if (leds == null) {
      leds = new Leds(new LedsIO() {});
    }

    // Set up Choreo directory
    try {
      Method setChoreoDirMethod = Choreo.class.getDeclaredMethod("setChoreoDir", File.class);
      setChoreoDirMethod.setAccessible(true);
      setChoreoDirMethod.invoke(null, new File(Filesystem.getDeployDirectory(), "vts"));
    } catch (Exception e) {
      DriverStation.reportWarning("Failed to set Choreo directory.", false);
    }

    // Set up AprilTag layout type
    aprilTagLayoutChooser = new LoggedDashboardChooser<>("AprilTag Layout");
    aprilTagLayoutChooser.addDefaultOption("Official", FieldConstants.defaultAprilTagType);
    aprilTagLayoutChooser.addOption("Hub", AprilTagLayoutType.HUB);
    aprilTagLayoutChooser.addOption("Tower", AprilTagLayoutType.TOWER);
    aprilTagLayoutChooser.addOption("Outpost", AprilTagLayoutType.OUTPOST);
    aprilTagLayoutChooser.addOption("None", AprilTagLayoutType.NONE);

    // Set up overrides
    hood.setCoastOverride(() -> coastOverride);
    hopper.setCoastOverride(() -> coastOverride);
    slamtake.setCoastOverride(() -> coastOverride);
    kicker.setCoastOverride(() -> coastOverride);
    HubShiftUtil.setAllianceWinOverride(
        () -> {
          if (lostAutoOverride.getAsBoolean()) {
            return Optional.of(false);
          }
          if (wonAutoOverride.getAsBoolean()) {
            return Optional.of(true);
          }
          return Optional.empty();
        });

    // Configure the autos and button bindings
    configureAutos();
    configureButtonBindings();

    // Set default commands
    hood.setDefaultCommand(hood.runFixedCommand(() -> Hood.minAngle, () -> 0.0));
    flywheel.setDefaultCommand(
        new ContinuousConditionalCommand(
            flywheel.stopCommand(),
            flywheel.runFixedCommand(LaunchCalculator.idleSpeed, false),
            disableAutoSpinup));
  }

  private void configureAutos() {
    AutoBuilder autoBuilder =
        new AutoBuilder(
            drive,
            slamtake,
            hopper,
            kicker,
            hood,
            flywheel,
            autoSelector::getResponses,
            offsetTime);

    // Monopoly Salesman
    autoSelector.addRoutine(
        "Monopoly Salesman",
        List.of(
            new AutoQuestion(
                "Start Position?",
                List.of(AutoQuestionResponse.LEFT_TRENCH, AutoQuestionResponse.LEFT_BUMP)),
            new AutoQuestion("Post-Launch?", List.of(AutoQuestionResponse.NOTHING))),
        autoBuilder.monopolySalesman());

    // Timid Salesman
    autoSelector.addRoutine(
        "Timid Salesman",
        List.of(
            new AutoQuestion(
                "Start Position?",
                List.of(
                    AutoQuestionResponse.LEFT_TRENCH,
                    AutoQuestionResponse.LEFT_BUMP,
                    AutoQuestionResponse.CENTER,
                    AutoQuestionResponse.RIGHT_BUMP,
                    AutoQuestionResponse.RIGHT_TRENCH))),
        autoBuilder.timidSalesman());

    // Characterization
    autoSelector.addRoutine(
        "Combobulated Salesman",
        DriveCommands.feedforwardCharacterization(drive)
            .deadlineFor(
                flywheel.stopCommand(),
                Commands.run(() -> slamtake.setIntakeGoal(IntakeGoal.STOP), slamtake))
            .withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
    autoSelector.addRoutine(
        "Discombobulated Salesman",
        DriveCommands.wheelRadiusCharacterization(drive)
            .deadlineFor(
                flywheel.stopCommand(),
                Commands.run(() -> slamtake.setIntakeGoal(IntakeGoal.STOP), slamtake))
            .withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
  }

  /** Create the bindings between buttons and commands. */
  private void configureButtonBindings() {
    // Drive controls
    DoubleSupplier driverX = () -> -primary.getLeftY() - secondary.getLeftY();
    DoubleSupplier driverY = () -> -primary.getLeftX() - secondary.getLeftX();
    DoubleSupplier driverOmega = () -> -primary.getRightX() - secondary.getRightX();
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(drive, driverX, driverY, driverOmega, robotRelative));

    // ***** PRIMARY CONTROLLER *****

    Trigger hubActiveOrPassing =
        new Trigger(
            () ->
                HubShiftUtil.getShiftedShiftInfo().active()
                    || LaunchCalculator.getInstance().getParameters().passing());
    Trigger inLaunchingTolerance =
        new Trigger(
            () ->
                hood.atGoal()
                        && (LaunchCalculator.getInstance().getParameters().passing()
                            ? flywheel.withinTolerance(40.0)
                            : flywheel.atGoal())
                        && DriveCommands.atLaunchGoal()
                        && DriveCommands.atPitchAndRollTolerance()
                    || noTiltCheck.getAsBoolean());

    // Align and auto-launch
    primary
        .leftBumper()
        .whileTrue(DriveCommands.joystickDriveWhileLaunching(drive, driverX, driverY))
        .whileTrueContinuous(flywheel.runTrackTargetCommand())
        .whileTrueContinuous(hood.runTrackTargetCommand())
        .onFalse(Commands.runOnce(() -> slamtake.setSlamGoal(SlamGoal.DEPLOY)))
        .and(() -> LaunchCalculator.getInstance().getParameters().isValid())
        .and(() -> ignoreHubState.getAsBoolean() || hubActiveOrPassing.getAsBoolean())
        .and(inLaunchingTolerance.debounce(0.1, DebounceType.kFalling))
        .whileTrue(
            Commands.parallel(
                Commands.startEnd(
                    () -> hopper.setGoal(Hopper.Goal.LAUNCH),
                    () -> hopper.setGoal(Hopper.Goal.STOP),
                    hopper),
                Commands.startEnd(
                    () -> kicker.setGoal(Kicker.Goal.LAUNCH),
                    () -> kicker.setGoal(Kicker.Goal.STOP),
                    kicker),
                CompactingCommands.compact(slamtake)))
        .onFalse(
            Commands.startEnd(
                    () -> kicker.setGoal(Kicker.Goal.OUTTAKE),
                    () -> kicker.setGoal(Kicker.Goal.STOP),
                    kicker)
                .withTimeout(0.5));

    // Trying to launch in an inactive period
    primary
        .leftBumper()
        .and(() -> !LaunchCalculator.getInstance().getParameters().passing())
        .and(inLaunchingTolerance)
        .onTrue(
            Commands.runEnd(
                    () -> secondary.setRumble(RumbleType.kBothRumble, 1.0),
                    () -> secondary.setRumble(RumbleType.kBothRumble, 0.0))
                .withTimeout(.5));

    // Force launch (no tolerance checking)
    primary
        .rightBumper()
        .or(secondary.rightBumper())
        .and(() -> ignoreHubState.getAsBoolean() || hubActiveOrPassing.getAsBoolean())
        .whileTrue(
            Commands.parallel(
                    Commands.startEnd(
                        () -> hopper.setGoal(Hopper.Goal.LAUNCH),
                        () -> hopper.setGoal(Hopper.Goal.STOP),
                        hopper),
                    Commands.startEnd(
                        () -> kicker.setGoal(Kicker.Goal.LAUNCH),
                        () -> kicker.setGoal(Kicker.Goal.STOP),
                        kicker),
                    CompactingCommands.compact(slamtake))
                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming));

    // Outtake
    primary
        .b()
        .whileTrue(
            Commands.parallel(
                    Commands.startEnd(
                        () -> hopper.setGoal(Hopper.Goal.OUTTAKE),
                        () -> hopper.setGoal(Hopper.Goal.STOP),
                        hopper),
                    Commands.startEnd(
                        () -> kicker.setGoal(Kicker.Goal.OUTTAKE),
                        () -> kicker.setGoal(Kicker.Goal.STOP),
                        kicker),
                    Commands.startEnd(
                        () -> slamtake.setIntakeGoal(IntakeGoal.OUTTAKE),
                        () -> slamtake.setIntakeGoal(IntakeGoal.STOP)))
                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming));

    // Unjam
    primary
        .rightClaw()
        .whileTrue(
            Commands.parallel(
                    Commands.startEnd(
                        () -> hopper.setGoal(Hopper.Goal.OUTTAKE),
                        () -> hopper.setGoal(Hopper.Goal.STOP),
                        hopper),
                    Commands.startEnd(
                        () -> kicker.setGoal(Kicker.Goal.OUTTAKE),
                        () -> kicker.setGoal(Kicker.Goal.STOP),
                        kicker),
                    flywheel.runFixedCommand(() -> -50.0, false))
                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming));

    // Outpost preset
    primary
        .lowerRightPaddle()
        .whileTrue(
            flywheel
                .runFixedCommand(LaunchCalculator.outpostPreset.flywheelSpeed(), true)
                .alongWith(
                    hood.runFixedCommand(
                        () ->
                            Units.degreesToRadians(
                                LaunchCalculator.outpostPreset.hoodAngleDeg().get()),
                        () -> 0.0)));

    // Passing preset
    primary
        .leftClaw()
        .whileTrue(
            flywheel
                .runFixedCommand(LaunchCalculator.passingPreset.flywheelSpeed(), true)
                .alongWith(
                    hood.runFixedCommand(
                        () ->
                            Units.degreesToRadians(
                                LaunchCalculator.passingPreset.hoodAngleDeg().get()),
                        () -> 0.0)));

    // Tower preset
    primary
        .lowerLeftPaddle()
        .whileTrue(
            flywheel
                .runFixedCommand(LaunchCalculator.towerPreset.flywheelSpeed(), false)
                .alongWith(
                    hood.runFixedCommand(
                        () ->
                            Units.degreesToRadians(
                                LaunchCalculator.towerPreset.hoodAngleDeg().get()),
                        () -> 0.0)));

    // Trench preset
    primary
        .upperLeftPaddle()
        .whileTrue(
            flywheel
                .runFixedCommand(LaunchCalculator.trenchPreset.flywheelSpeed(), false)
                .alongWith(
                    hood.runFixedCommand(
                        () ->
                            Units.degreesToRadians(
                                LaunchCalculator.trenchPreset.hoodAngleDeg().get()),
                        () -> 0.0)));

    // Blearghh button
    primary
        .upperRightPaddle()
        .whileTrue(
            flywheel
                .runFixedCommand(LaunchCalculator.blearghhPreset.flywheelSpeed(), false)
                .alongWith(
                    hood.runFixedCommand(
                        () ->
                            Units.degreesToRadians(
                                LaunchCalculator.blearghhPreset.hoodAngleDeg().get()),
                        () -> 0.0)))
        .and(() -> flywheel.withinTolerance(40.0))
        .whileTrue(
            Commands.parallel(
                Commands.startEnd(
                    () -> hopper.setGoal(Hopper.Goal.BLEARGHH),
                    () -> hopper.setGoal(Hopper.Goal.STOP),
                    hopper),
                Commands.startEnd(
                    () -> kicker.setGoal(Kicker.Goal.BLEARGHH),
                    () -> kicker.setGoal(Kicker.Goal.STOP),
                    kicker)));

    // Retract intake
    primary
        .rightTrigger()
        .or(secondary.rightTrigger())
        .onTrue(Commands.runOnce(() -> slamtake.setSlamGoal(SlamGoal.RETRACT)))
        .onTrue(
            Commands.runEnd(
                    () -> slamtake.setIntakeGoal(IntakeGoal.INTAKE),
                    () -> slamtake.setIntakeGoal(IntakeGoal.STOP),
                    slamtake)
                .withTimeout(1.0));

    // Run intake
    primary
        .leftTrigger()
        .and(() -> DriverStation.isTeleopEnabled())
        .or(secondary.leftTrigger())
        .whileTrue(
            Commands.runEnd(
                    () -> slamtake.setIntakeGoal(IntakeGoal.INTAKE),
                    () -> slamtake.setIntakeGoal(IntakeGoal.STOP),
                    slamtake)
                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming))
        .whileTrue(Commands.run(() -> slamtake.setSlamGoal(SlamGoal.DEPLOY)));

    primary.a().whileTrue(DriveCommands.joystickDriveUnderTower(drive, driverX, driverY));

    // ***** SECONDARY CONTROLLER *****

    // Reset gyro
    secondary
        .start()
        .and(secondary.back())
        .onTrue(
            Commands.runOnce(
                    () ->
                        RobotState.getInstance()
                            .resetPose(
                                new Pose2d(
                                    RobotState.getInstance().getEstimatedPose().getTranslation(),
                                    AllianceFlipUtil.apply(Rotation2d.kZero))))
                .withName("ResetGyro")
                .ignoringDisable(true));

    // Systems check preset (min hood angle)
    secondary
        .a()
        .whileTrue(
            flywheel
                .runFixedCommand(LaunchCalculator.hoodMinPreset.flywheelSpeed(), false)
                .alongWith(
                    hood.runFixedCommand(
                        () ->
                            Units.degreesToRadians(
                                LaunchCalculator.hoodMinPreset.hoodAngleDeg().get()),
                        () -> 0.0)));

    // Systems check preset (max hood angle)
    secondary
        .y()
        .whileTrue(
            flywheel
                .runFixedCommand(LaunchCalculator.hoodMaxPreset.flywheelSpeed(), false)
                .alongWith(
                    hood.runFixedCommand(
                        () ->
                            Units.degreesToRadians(
                                LaunchCalculator.hoodMaxPreset.hoodAngleDeg().get()),
                        () -> 0.0)));

    // Hood and slam run zero commands
    secondary.x().onTrue(hood.zeroCommand().alongWith(slamtake.homeSlam()));

    // Manual zero commands (hood min, slam max)
    secondary.b().onTrue(hood.forceZeroCommand().alongWith(slamtake.zeroMaxSlam()));

    // Flywheel speed offset
    secondary
        .povUp()
        .whileTrue(
            Commands.runOnce(() -> LaunchCalculator.getInstance().incrementFlywheelSpeedOffset(1.0))
                .andThen(
                    Commands.waitSeconds(0.3),
                    Commands.repeatingSequence(
                        Commands.runOnce(
                            () -> LaunchCalculator.getInstance().incrementFlywheelSpeedOffset(1.0)),
                        Commands.waitSeconds(0.1)))
                .ignoringDisable(true));
    secondary
        .povDown()
        .whileTrue(
            Commands.runOnce(
                    () -> LaunchCalculator.getInstance().incrementFlywheelSpeedOffset(-1.0))
                .andThen(
                    Commands.waitSeconds(0.3),
                    Commands.repeatingSequence(
                        Commands.runOnce(
                            () ->
                                LaunchCalculator.getInstance().incrementFlywheelSpeedOffset(-1.0)),
                        Commands.waitSeconds(0.1)))
                .ignoringDisable(true));

    // Flywheel feedforward characterization test
    secondary
        .povRight()
        .whileTrue(flywheel.feedforwardCharacterizationCommand())
        .onFalse(
            Commands.runEnd(
                    () -> secondary.setRumble(RumbleType.kBothRumble, 1.0),
                    () -> secondary.setRumble(RumbleType.kBothRumble, 0.0))
                .withTimeout(.5)
                .onlyIf(() -> !(flywheel.isWithinTolerancekS() && flywheel.isWithinTolerancekV())));

    // Test flywheel spin-up
    secondary.leftBumper().whileTrue(flywheel.runFixedCommand(() -> 200.0, false));

    // ****** OVERRIDE SWITCHES *****

    // Coast override
    coast
        .onTrue(
            Commands.runOnce(
                    () -> {
                      if (DriverStation.isDisabled()) {
                        coastOverride = true;
                        leds.superstructureCoast = true;
                      }
                    })
                .withName("Superstructure Coast")
                .ignoringDisable(true))
        .onFalse(
            Commands.runOnce(
                    () -> {
                      coastOverride = false;
                      leds.superstructureCoast = false;
                    })
                .withName("Superstructure Uncoast")
                .ignoringDisable(true));

    // Hub counter override
    ignoreHubState
        .onTrue(
            Commands.runOnce(() -> hubCounter.setExternal(false))
                .withName("Enable External Hub Counter Control")
                .ignoringDisable(true))
        .onFalse(
            Commands.runOnce(() -> hubCounter.setExternal(true))
                .withName("Disable External Hub Counter Control")
                .ignoringDisable(true));
    BeachedUtil.getInstance().setIgnoreBeached(noTiltCheck);
    hubCounter.setExternal(!ignoreHubState.getAsBoolean());

    // Sport mode override
    sportModeOverride
        .onTrue(
            Commands.runOnce(
                    () -> {
                      flywheel.setSportMode(true);
                      hopper.setSportMode(true);
                    })
                .withName("Enable Sport Mode")
                .ignoringDisable(true))
        .onFalse(
            (Commands.runOnce(
                    () -> {
                      flywheel.setSportMode(false);
                      hopper.setSportMode(false);
                    })
                .withName("Disable Sport Mode")
                .ignoringDisable(true)));

    // ****** ALERTS ******

    // Warn formissing game data
    Timer teleopElapsedTimer = new Timer();
    RobotModeTriggers.teleop()
        .onTrue(
            Commands.runOnce(
                () -> {
                  teleopElapsedTimer.restart();
                }));
    RobotModeTriggers.teleop()
        .and(() -> !(DriverStation.getGameSpecificMessage().length() > 0))
        .and(() -> HubShiftUtil.getAllianceWinOverride().isEmpty())
        .and(() -> teleopElapsedTimer.hasElapsed(1.0))
        .whileTrue(
            Commands.runEnd(
                () -> {
                  primary.setRumble(RumbleType.kBothRumble, 1);
                  secondary.setRumble(RumbleType.kBothRumble, 1);
                },
                () -> {
                  primary.setRumble(RumbleType.kBothRumble, 0);
                  secondary.setRumble(RumbleType.kBothRumble, 0);
                }))
        .whileTrue(
            Commands.startEnd(
                () -> {
                  autoWinnerNotSet.set(true);
                  leds.autoWinnerNotSet = true;
                },
                () -> {
                  autoWinnerNotSet.set(false);
                  leds.autoWinnerNotSet = false;
                }));

    // End-of-shift warning
    for (int i = 1; i <= 5; i++) {
      double time = i;
      Trigger shiftAboutToEnd =
          new Trigger(() -> (HubShiftUtil.getShiftedShiftInfo().remainingTime() < time));
      shiftAboutToEnd
          .and(RobotModeTriggers.teleop())
          .and(ignoreHubState.negate())
          .onTrue(
              Commands.runEnd(
                      () -> primary.setRumble(RumbleType.kRightRumble, 1.0),
                      () -> primary.setRumble(RumbleType.kBothRumble, 0.0))
                  .withTimeout(0.25));
    }

    // Send tolerance information to LEDs
    inLaunchingTolerance.whileTrue(
        Commands.runEnd(
            () -> leds.inLaunchingTolerance = true, () -> leds.inLaunchingTolerance = false));

    // ****** ROBOT STATE *****

    // Automatically deploy intake on enable
    RobotModeTriggers.teleop()
        .onTrue(Commands.runOnce(() -> slamtake.setSlamGoal(SlamGoal.DEPLOY)));
    RobotModeTriggers.autonomous()
        .onTrue(
            Commands.waitSeconds(0.5)
                .andThen(Commands.runOnce(() -> slamtake.setSlamGoal(SlamGoal.DEPLOY))));

    // Automatically zero hood and intake
    RobotModeTriggers.teleop()
        .onTrue(
            hood.zeroCommand()
                .unless(hood::isZeroed)
                .alongWith(slamtake.homeSlam().unless(slamtake::isZeroed)));

    // Run the autonomous command for the hood during auto
    RobotModeTriggers.autonomous().whileTrue(hood.autonomousCommand());

    // Automatically run intake and flywheel in auto
    RobotModeTriggers.autonomous()
        .whileTrue(
            Commands.waitSeconds(1.0)
                .andThen(
                    Commands.runEnd(
                        () -> slamtake.setIntakeGoal(IntakeGoal.INTAKE),
                        () -> slamtake.setIntakeGoal(IntakeGoal.STOP),
                        slamtake)))
        .whileTrue(Commands.waitSeconds(1.5).andThen(flywheel.runTrackTargetCommand()));

    // Disable coast when enabling
    RobotModeTriggers.disabled()
        .onFalse(
            Commands.runOnce(
                    () -> {
                      coastOverride = false;
                      leds.superstructureCoast = false;
                    })
                .ignoringDisable(true));

    // Reset hub shift timer when enabling
    RobotModeTriggers.teleop().onTrue(Commands.runOnce(HubShiftUtil::initialize));
    RobotModeTriggers.autonomous().onTrue(Commands.runOnce(HubShiftUtil::initialize));
    RobotModeTriggers.disabled()
        .onTrue(Commands.runOnce(HubShiftUtil::initialize).ignoringDisable(true));
    RobotModeTriggers.autonomous().onTrue(Commands.runOnce(hubCounter::initialize));

    // Set initial fuel poses when starting auto
    RobotModeTriggers.autonomous()
        .onTrue(Commands.runOnce(() -> ObjectDetection.getInstance().initializeFuelPoses()));

    // Force zero hood when starting auto
    RobotModeTriggers.autonomous().onTrue(hood.forceZeroCommand());

    // Force zero intake when starting auto
    RobotModeTriggers.autonomous().onTrue(slamtake.zeroMaxSlam());
  }

  private void configureFuelSim() {
    fuelSim.registerRobot(
        DriveConstants.fullWidthX,
        DriveConstants.fullWidthY,
        Units.inchesToMeters(6.0),
        () -> RobotState.getInstance().getEstimatedPose(),
        () -> RobotState.getInstance().getFieldVelocity());

    fuelSim.registerIntake(
        DriveConstants.intakeNearX,
        DriveConstants.intakeFarX,
        -DriveConstants.fullApothemY,
        DriveConstants.fullApothemY,
        () ->
            slamtake.getSlamGoal().equals(Slamtake.SlamGoal.DEPLOY)
                && slamtake.getIntakeGoal().equals(Slamtake.IntakeGoal.INTAKE)
                && simFuelCount.getFuelStored() < SimFuelCount.capacity,
        () ->
            simFuelCount.setFuelStored(
                Math.min(simFuelCount.getFuelStored() + 1, SimFuelCount.capacity)));

    fuelSim.setSubticks(1);
    fuelSim.start();
    fuelSim.spawnStartingFuel();

    RobotModeTriggers.autonomous()
        .onTrue(
            Commands.runOnce(
                () -> {
                  fuelSim.clearFuel();
                  fuelSim.spawnStartingFuel();
                  simFuelCount.setFuelStored(8);
                }));
  }

  public void updateFuelSim() {
    if (fuelSim != null) {
      fuelSim.updateSim();
    }
  }

  /** Update dashboard outputs. */
  public void updateDashboardOutputs() {
    // Drive tolerance
    SmartDashboard.putBoolean("Drive At Goal", DriveCommands.atLaunchGoal());
    SmartDashboard.putBoolean("Drive Pitch & Roll OK", DriveCommands.atPitchAndRollTolerance());

    // Publish match time
    SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());

    // Update from HubShiftUtil
    SmartDashboard.putString(
        "Shifts/Remaining Shift Time",
        String.format("%.1f", Math.max(HubShiftUtil.getShiftedShiftInfo().remainingTime(), 0.0)));
    SmartDashboard.putBoolean("Shifts/Shift Active", HubShiftUtil.getShiftedShiftInfo().active());
    SmartDashboard.putString(
        "Shifts/Game State", HubShiftUtil.getShiftedShiftInfo().currentShift().toString());
    SmartDashboard.putBoolean(
        "Shifts/Active First?",
        DriverStation.getAlliance().orElse(Alliance.Blue) == HubShiftUtil.getFirstActiveAlliance());

    // Controller disconnected alerts
    primaryDisconnected.set(!DriverStation.isJoystickConnected(primary.getHID().getPort()));
    secondaryDisconnected.set(!DriverStation.isJoystickConnected(secondary.getHID().getPort()));
    overrideDisconnected.set(!overrides.isConnected());

    // AprilTag layout alert
    boolean aprilTagAlertActive = getSelectedAprilTagLayout() != FieldConstants.defaultAprilTagType;
    aprilTagLayoutAlert.set(aprilTagAlertActive);
    if (aprilTagAlertActive) {
      aprilTagLayoutAlert.setText(
          "Non-default AprilTag layout in use (" + getSelectedAprilTagLayout().toString() + ").");
    }
  }

  /** Returns the current AprilTag layout type. */
  public AprilTagLayoutType getSelectedAprilTagLayout() {
    return aprilTagLayoutChooser.get();
  }

  /** Returns the autonomous command for the Robot class. */
  public Command getAutonomousCommand() {
    return autoSelector.getCommand();
  }
}
