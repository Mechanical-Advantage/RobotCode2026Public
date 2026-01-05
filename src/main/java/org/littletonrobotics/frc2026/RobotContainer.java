// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2026;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import java.util.function.DoubleSupplier;
import lombok.experimental.ExtensionMethod;
import org.littletonrobotics.frc2026.FieldConstants.AprilTagLayoutType;
import org.littletonrobotics.frc2026.commands.DriveCommands;
import org.littletonrobotics.frc2026.subsystems.drive.Drive;
import org.littletonrobotics.frc2026.subsystems.drive.GyroIO;
import org.littletonrobotics.frc2026.subsystems.drive.ModuleIO;
import org.littletonrobotics.frc2026.subsystems.drive.ModuleIOSim;
import org.littletonrobotics.frc2026.subsystems.leds.Leds;
import org.littletonrobotics.frc2026.subsystems.leds.LedsIO;
import org.littletonrobotics.frc2026.subsystems.leds.LedsIOSim;
import org.littletonrobotics.frc2026.subsystems.vision.Vision;
import org.littletonrobotics.frc2026.util.controllers.OverrideSwitches;
import org.littletonrobotics.frc2026.util.controllers.RazerWolverineController;
import org.littletonrobotics.frc2026.util.controllers.TriggerUtil;
import org.littletonrobotics.frc2026.util.geometry.AllianceFlipUtil;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

@ExtensionMethod({TriggerUtil.class})
public class RobotContainer {
  // Subsystems
  private Drive drive;
  private Vision vision;
  private Leds leds;

  // Controller
  private final RazerWolverineController primary = new RazerWolverineController(0);
  private final CommandXboxController secondary = new CommandXboxController(1);
  private final OverrideSwitches overrides = new OverrideSwitches(5);

  private final Alert primaryDisconnected =
      new Alert("Primary controller disconnected (port 0).", AlertType.kWarning);
  private final Alert secondaryDisconnected =
      new Alert("Secondary controller disconnected (port 1).", AlertType.kWarning);
  private final Alert overrideDisconnected =
      new Alert("Override controller disconnected (port 5).", AlertType.kInfo);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    if (Constants.getMode() != Constants.Mode.REPLAY) {
      switch (Constants.robot) {
        case COMPBOT:
          // Not implemented
          break;

        case ALPHABOT:
          // Not implemented
          break;

        case SIMBOT:
          drive =
              new Drive(
                  new GyroIO() {},
                  new ModuleIOSim(),
                  new ModuleIOSim(),
                  new ModuleIOSim(),
                  new ModuleIOSim());
          leds = new Leds(new LedsIOSim());
          break;
      }
    }

    // No-op implementations for replay
    if (drive == null) {
      drive =
          new Drive(
              new GyroIO() {},
              new ModuleIO() {},
              new ModuleIO() {},
              new ModuleIO() {},
              new ModuleIO() {});
    }
    if (vision == null) {
      switch (Constants.robot) {
        case COMPBOT -> vision = new Vision(this::getSelectedAprilTagLayout);
        case ALPHABOT -> vision = new Vision(this::getSelectedAprilTagLayout);
        default -> vision = new Vision(this::getSelectedAprilTagLayout);
      }
    }
    if (leds == null) {
      leds = new Leds(new LedsIO() {});
    }

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices");
    autoChooser.addDefaultOption("Do Nothing", Commands.none());
    autoChooser.addDefaultOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));

    // Configure the button bindings
    configureButtonBindings();
  }

  /** Create the bindings between buttons and commands. */
  private void configureButtonBindings() {
    // Drive controls
    DoubleSupplier driverX = () -> -primary.getLeftY() - secondary.getLeftY();
    DoubleSupplier driverY = () -> -primary.getLeftX() - secondary.getLeftX();
    DoubleSupplier driverOmega = () -> -primary.getRightX() - secondary.getRightX();
    drive.setDefaultCommand(DriveCommands.joystickDrive(drive, driverX, driverY, driverOmega));

    // ***** PRIMARY CONTROLLER *****

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
  }

  /** Update dashboard outputs. */
  public void updateDashboardOutputs() {
    // Publish match time
    SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());

    // Controller disconnected alerts
    primaryDisconnected.set(!DriverStation.isJoystickConnected(primary.getHID().getPort()));
    secondaryDisconnected.set(!DriverStation.isJoystickConnected(secondary.getHID().getPort()));
    overrideDisconnected.set(!overrides.isConnected());
  }

  /** Returns the current AprilTag layout type. */
  public AprilTagLayoutType getSelectedAprilTagLayout() {
    return FieldConstants.defaultAprilTagType;
  }

  /** Returns the autonomous command for the Robot class. */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
