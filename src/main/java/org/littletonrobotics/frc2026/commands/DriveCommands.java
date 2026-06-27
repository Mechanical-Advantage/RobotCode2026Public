// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2026.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.LinkedList;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.frc2026.Constants;
import org.littletonrobotics.frc2026.FieldConstants;
import org.littletonrobotics.frc2026.RobotState;
import org.littletonrobotics.frc2026.subsystems.drive.Drive;
import org.littletonrobotics.frc2026.subsystems.drive.DriveConstants;
import org.littletonrobotics.frc2026.subsystems.launcher.LaunchCalculator;
import org.littletonrobotics.frc2026.subsystems.launcher.LauncherConstants;
import org.littletonrobotics.frc2026.util.LoggedTunableNumber;
import org.littletonrobotics.frc2026.util.geometry.AllianceFlipUtil;
import org.littletonrobotics.frc2026.util.geometry.GeomUtil;
import org.littletonrobotics.idun.IdunConstants;
import org.littletonrobotics.junction.Logger;

@IdunConstants
public class DriveCommands {
  public static final double deadband = 0.1;
  private static final double ffStartDelay = 2.0; // Secs
  private static final double ffRampRate = 0.1; // Volts/Sec
  private static final double wheelRadiusMaxVelocity = 0.25; // Rad/Sec
  private static final double wheelRadiusRampRate = 0.05; // Rad/Sec^2

  private static final LoggedTunableNumber driveTowerAngleKp =
      new LoggedTunableNumber("DriveCommands/Tower/kP", 6.0);
  private static final LoggedTunableNumber driveTowerAngleKd =
      new LoggedTunableNumber("DriveCommands/Tower/kD", 0.05);
  private static final LoggedTunableNumber driveLaunchKp =
      new LoggedTunableNumber("DriveCommands/Launching/kP", 8.0);
  private static final LoggedTunableNumber driveLaunchKd =
      new LoggedTunableNumber("DriveCommands/Launching/kD", 0.5);
  private static final LoggedTunableNumber driveYawLaunchToleranceDeg =
      new LoggedTunableNumber("DriveCommands/Launching/YawToleranceDeg", 10.0);
  private static final LoggedTunableNumber drivePitchLaunchToleranceDeg =
      new LoggedTunableNumber("DriveCommands/Launching/PitchToleranceDeg", 5.0);
  private static final LoggedTunableNumber driveRollLaunchToleranceDeg =
      new LoggedTunableNumber("DriveCommands/Launching/RollToleranceDeg", 5.0);
  private static final LoggedTunableNumber driveYawPassToleranceDeg =
      new LoggedTunableNumber("DriveCommands/Passing/YawToleranceDeg", 15.0);
  private static final LoggedTunableNumber drivePitchPassToleranceDeg =
      new LoggedTunableNumber("DriveCommands/Passing/PitchToleranceDeg", 5.0);
  private static final LoggedTunableNumber driveRollPassToleranceDeg =
      new LoggedTunableNumber("DriveCommands/Passing/RollToleranceDeg", 5.0);

  private static final LoggedTunableNumber lockMetersPerSecondThreshold =
      new LoggedTunableNumber("DriveCommands/Launching/LockMetersPerSecThreshold", 0.1);
  private static final LoggedTunableNumber lockOmegaRadsPerSecThreshold =
      new LoggedTunableNumber("DriveCommands/Launching/LockOmegaRadsPerSecThreshold", 0.15);

  private static final LoggedTunableNumber driveLaunchMaxPolarVelocityRadPerSec =
      new LoggedTunableNumber("DriveCommands/Launching/MaxPolarVelocityRadPerSec", 0.4);
  private static final LoggedTunableNumber driveLauncherCORMinErrorDeg =
      new LoggedTunableNumber("DriveCommands/Launching/DriveLauncherCORMinErrorDeg", 15.0);
  private static final LoggedTunableNumber driveLauncherCORMaxErrorDeg =
      new LoggedTunableNumber("DriveCommands/Launching/DriveLauncherCORMaxErrorDeg", 30.0);

  private DriveCommands() {}

  public static Translation2d getLinearVelocityFromJoysticks(double x, double y) {
    // Apply deadband
    double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), deadband);
    Rotation2d linearDirection = new Rotation2d(x, y);

    // Square magnitude for more precise control
    linearMagnitude = linearMagnitude * linearMagnitude;

    // Return new linear velocity
    return new Pose2d(Translation2d.kZero, linearDirection)
        .transformBy(new Transform2d(linearMagnitude, 0.0, Rotation2d.kZero))
        .getTranslation();
  }

  public static double getOmegaFromJoysticks(double driverOmega) {
    double omega = MathUtil.applyDeadband(driverOmega, deadband);
    return omega * omega * Math.signum(omega);
  }

  public static ChassisSpeeds getSpeedsFromJoysticks(
      double driverX, double driverY, double driverOmega) {
    // Get linear velocity
    Translation2d linearVelocity =
        getLinearVelocityFromJoysticks(driverX, driverY).times(DriveConstants.maxLinearSpeed);

    // Calculate angular velocity
    double omega = getOmegaFromJoysticks(driverOmega);

    return new ChassisSpeeds(
        linearVelocity.getX(), linearVelocity.getY(), omega * DriveConstants.maxAngularSpeed);
  }

  /**
   * Field or robot relative drive command using two joysticks (controlling linear and angular
   * velocities).
   */
  public static Command joystickDrive(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier,
      BooleanSupplier robotRelative) {
    return Commands.run(
        () -> {
          ChassisSpeeds speeds =
              getSpeedsFromJoysticks(
                  xSupplier.getAsDouble(), ySupplier.getAsDouble(), omegaSupplier.getAsDouble());
          drive.runVelocity(
              robotRelative.getAsBoolean()
                  ? speeds
                  : ChassisSpeeds.fromFieldRelativeSpeeds(
                      speeds,
                      DriverStation.getAlliance().isPresent()
                              && DriverStation.getAlliance().get() == Alliance.Red
                          ? RobotState.getInstance().getRotation().plus(Rotation2d.kPi)
                          : RobotState.getInstance().getRotation()));
        },
        drive);
  }

  public static Command joystickDriveUnderTower(
      Drive drive, DoubleSupplier xSupplier, DoubleSupplier ySupplier) {

    return Commands.run(
        () -> {
          Rotation2d nearestLockedRotation =
              RobotState.getInstance().getRotation().getRadians() > 0
                  ? Rotation2d.kCCW_90deg
                  : Rotation2d.kCW_90deg;
          var omegaOutput =
              nearestLockedRotation.minus(RobotState.getInstance().getRotation()).getRadians()
                      * driveTowerAngleKp.get()
                  - RobotState.getInstance().getFieldVelocity().omegaRadiansPerSecond
                      * driveTowerAngleKd.get();

          // Calculate speeds
          Translation2d fieldRelativeLinearVelocity =
              getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble())
                  .times(DriveConstants.maxLinearSpeed);
          if (AllianceFlipUtil.shouldFlip()) {
            fieldRelativeLinearVelocity = fieldRelativeLinearVelocity.times(-1.0);
          }
          ChassisSpeeds speeds =
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  fieldRelativeLinearVelocity.getX(),
                  fieldRelativeLinearVelocity.getY(),
                  omegaOutput,
                  RobotState.getInstance().getRotation());
          drive.runVelocity(speeds);
          RobotState.getInstance()
              .setRobotSetpointVelocity(
                  ChassisSpeeds.discretize(
                      new ChassisSpeeds(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, 0.0),
                      Constants.loopPeriodSecs));
        },
        drive);
  }

  public static boolean atPitchAndRollTolerance() {
    var passing = LaunchCalculator.getInstance().getParameters().passing();
    var rotation3d =
        RobotState.getInstance().getEstimatedRotation3dAtTimestamp(Timer.getTimestamp());
    return rotation3d.isEmpty()
        || (Math.abs(rotation3d.get().getX())
                <= Units.degreesToRadians(
                    passing ? driveRollPassToleranceDeg.get() : driveRollLaunchToleranceDeg.get())
            && Math.abs(rotation3d.get().getY())
                <= Units.degreesToRadians(
                    passing
                        ? drivePitchPassToleranceDeg.get()
                        : drivePitchLaunchToleranceDeg.get()));
  }

  public static boolean atLaunchGoal() {
    var passing = LaunchCalculator.getInstance().getParameters().passing();
    return DriverStation.isEnabled()
        && Math.abs(
                RobotState.getInstance()
                    .getRotation()
                    .minus(LaunchCalculator.getInstance().getParameters().driveAngle())
                    .getRadians())
            <= Units.degreesToRadians(
                passing ? driveYawPassToleranceDeg.get() : driveYawLaunchToleranceDeg.get());
  }

  public static Command joystickDriveWhileLaunching(
      Drive drive, DoubleSupplier xSupplier, DoubleSupplier ySupplier) {
    // Create command
    return Commands.run(
        () -> {
          // Run PID controller
          final var parameters = LaunchCalculator.getInstance().getParameters();
          double omegaOutput =
              parameters.driveVelocity()
                  + (parameters
                          .driveAngle()
                          .minus(RobotState.getInstance().getRotation())
                          .getRadians()
                      * driveLaunchKp.get())
                  + ((parameters.driveVelocity()
                          - RobotState.getInstance().getRobotVelocity().omegaRadiansPerSecond)
                      * driveLaunchKd.get());

          // Calculate speeds
          Translation2d fieldRelativeLinearVelocity =
              getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble())
                  .times(DriveConstants.maxLinearSpeed);
          if (AllianceFlipUtil.shouldFlip()) {
            fieldRelativeLinearVelocity = fieldRelativeLinearVelocity.times(-1.0);
          }

          // Only limit if launching, not passing
          if (!LaunchCalculator.getInstance().getParameters().passing()) {
            // Calculate max linear velocity magnitude based on the max polar velocity
            double maxLinearVelocityMagnitude = Double.POSITIVE_INFINITY;
            double robotAngle =
                Math.abs(
                    AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint.toTranslation2d())
                        .minus(RobotState.getInstance().getEstimatedPose().getTranslation())
                        .getAngle()
                        .minus(fieldRelativeLinearVelocity.getAngle())
                        .getRadians());
            double robotHubDistance =
                LaunchCalculator.getInstance().getParameters().distanceNoLookahead();
            double hubAngle =
                driveLaunchMaxPolarVelocityRadPerSec.get()
                    * LaunchCalculator.getInstance().getNaiveTOF(robotHubDistance);
            double lookaheadAngle = Math.PI - robotAngle - hubAngle;

            // Calculate limit if triangle is valid (otherwise no limit)
            if (lookaheadAngle > 0) {
              double robotLookaheadDistance =
                  robotHubDistance * Math.sin(hubAngle) / Math.sin(lookaheadAngle);
              maxLinearVelocityMagnitude =
                  robotLookaheadDistance
                      / LaunchCalculator.getInstance().getNaiveTOF(robotHubDistance);
            }

            // Apply limit to velocity
            if (fieldRelativeLinearVelocity.getNorm() > maxLinearVelocityMagnitude) {
              fieldRelativeLinearVelocity =
                  fieldRelativeLinearVelocity.times(
                      maxLinearVelocityMagnitude / fieldRelativeLinearVelocity.getNorm());
            }
          }

          // Apply chassis speeds
          double corScalar =
              MathUtil.clamp(
                  (Math.abs(
                              parameters
                                  .driveAngle()
                                  .minus(RobotState.getInstance().getRotation())
                                  .getDegrees())
                          - driveLauncherCORMinErrorDeg.get())
                      / (driveLauncherCORMaxErrorDeg.get() - driveLauncherCORMinErrorDeg.get()),
                  0.0,
                  1.0);
          Translation2d launcherToRobot =
              LauncherConstants.robotToLauncher.getTranslation().toTranslation2d().unaryMinus();
          ChassisSpeeds fieldRelativeSpeedsWithOffset =
              GeomUtil.transformVelocity(
                  new ChassisSpeeds(
                      fieldRelativeLinearVelocity.getX(),
                      fieldRelativeLinearVelocity.getY(),
                      omegaOutput),
                  launcherToRobot.times(1.0 - corScalar),
                  RobotState.getInstance().getRotation());

          // Apply O-lock
          boolean oLock =
              Math.hypot(
                          fieldRelativeSpeedsWithOffset.vxMetersPerSecond,
                          fieldRelativeSpeedsWithOffset.vyMetersPerSecond)
                      < lockMetersPerSecondThreshold.get()
                  && Math.abs(fieldRelativeSpeedsWithOffset.omegaRadiansPerSecond)
                      < lockOmegaRadsPerSecThreshold.get();
          Logger.recordOutput("DriveCommands/Launching/OLock", oLock);
          if (oLock) {
            drive.stopWithO();
          } else {
            drive.runVelocity(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                    fieldRelativeSpeedsWithOffset, RobotState.getInstance().getRotation()));
          }

          // Override robot setpoint speeds published by drive. We run our calculations using the
          // speeds that will ultimately be applied once we are using the full robot-to-launcher
          // transform. This prevents the setpoint from changing due to the shifting COR of the
          // robot.
          ChassisSpeeds fieldRelativeSpeedsWithFullOffset =
              GeomUtil.transformVelocity(
                  new ChassisSpeeds(
                      fieldRelativeLinearVelocity.getX(),
                      fieldRelativeLinearVelocity.getY(),
                      omegaOutput),
                  launcherToRobot,
                  RobotState.getInstance().getRotation());
          RobotState.getInstance()
              .setRobotSetpointVelocity(
                  ChassisSpeeds.discretize(
                      ChassisSpeeds.fromFieldRelativeSpeeds(
                          fieldRelativeSpeedsWithFullOffset,
                          RobotState.getInstance().getRotation()),
                      Constants.loopPeriodSecs));

          // Log data
          Logger.recordOutput(
              "DriveCommands/Launching/SetpointPose",
              new Pose2d(
                  RobotState.getInstance().getEstimatedPose().getTranslation(),
                  parameters.driveAngle()));
          Logger.recordOutput("DriveCommands/Launching/AtGoalTolerance", atLaunchGoal());
          Logger.recordOutput(
              "DriveCommands/Launching/AtPitchAndRollTolerance", atPitchAndRollTolerance());
          Logger.recordOutput(
              "DriveCommands/Launching/ErrorPosition",
              parameters.driveAngle().minus(RobotState.getInstance().getRotation()));
          Logger.recordOutput(
              "DriveCommands/Launching/ErrorVelocityRadPerSec",
              parameters.driveVelocity()
                  - RobotState.getInstance().getRobotVelocity().omegaRadiansPerSecond);
          Logger.recordOutput(
              "DriveCommands/Launching/MeasuredPosition", RobotState.getInstance().getRotation());
          Logger.recordOutput(
              "DriveCommands/Launching/MeasuredVelocityRadPerSec",
              RobotState.getInstance().getRobotVelocity().omegaRadiansPerSecond);
          Logger.recordOutput("DriveCommands/Launching/SetpointPosition", parameters.driveAngle());
          Logger.recordOutput(
              "DriveCommands/Launching/SetpointVelocityRadPerSec", parameters.driveVelocity());
        },
        drive);
  }

  /**
   * Measures the velocity feedforward constants for the drive motors.
   *
   * <p>This command should only be used in voltage control mode.
   */
  public static Command feedforwardCharacterization(Drive drive) {
    List<Double> velocitySamples = new LinkedList<>();
    List<Double> voltageSamples = new LinkedList<>();
    Timer timer = new Timer();

    return Commands.sequence(
        // Reset data
        Commands.runOnce(
            () -> {
              velocitySamples.clear();
              voltageSamples.clear();
            }),

        // Allow modules to orient
        Commands.run(
                () -> {
                  drive.runCharacterization(0.0);
                },
                drive)
            .withTimeout(ffStartDelay),

        // Start timer
        Commands.runOnce(timer::restart),

        // Accelerate and gather data
        Commands.run(
                () -> {
                  double voltage = timer.get() * ffRampRate;
                  drive.runCharacterization(voltage);
                  velocitySamples.add(drive.getFFCharacterizationVelocity());
                  voltageSamples.add(voltage);
                },
                drive)

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
                  double kS = (sumY * sumX2 - sumX * sumXY) / (n * sumX2 - sumX * sumX);
                  double kV = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);

                  NumberFormat formatter = new DecimalFormat("#0.00000");
                  System.out.println("********** Drive FF Characterization Results **********");
                  System.out.println("\tkS: " + formatter.format(kS));
                  System.out.println("\tkV: " + formatter.format(kV));
                }));
  }

  /** Measures the robot's wheel radius by spinning in a circle. */
  public static Command wheelRadiusCharacterization(Drive drive) {
    SlewRateLimiter limiter = new SlewRateLimiter(wheelRadiusRampRate);
    WheelRadiusCharacterizationState state = new WheelRadiusCharacterizationState();

    return Commands.parallel(
        // Drive control sequence
        Commands.sequence(
            // Reset acceleration limiter
            Commands.runOnce(() -> limiter.reset(0.0)),

            // Turn in place, accelerating up to full speed
            Commands.run(
                () -> {
                  double speed = limiter.calculate(wheelRadiusMaxVelocity);
                  drive.runVelocity(new ChassisSpeeds(0.0, 0.0, speed));
                },
                drive)),

        // Measurement sequence
        Commands.sequence(
            // Wait for modules to fully orient before starting measurement
            Commands.waitSeconds(1.0),

            // Record starting measurement
            Commands.runOnce(
                () -> {
                  state.positions = drive.getWheelRadiusCharacterizationPositions();
                  state.lastAngle = RobotState.getInstance().getRotation();
                  state.gyroDelta = 0.0;
                }),

            // Update gyro delta
            Commands.run(
                    () -> {
                      var rotation = RobotState.getInstance().getRotation();
                      state.gyroDelta += Math.abs(rotation.minus(state.lastAngle).getRadians());
                      state.lastAngle = rotation;

                      double[] positions = drive.getWheelRadiusCharacterizationPositions();
                      double wheelDelta = 0.0;
                      for (int i = 0; i < 4; i++) {
                        wheelDelta += Math.abs(positions[i] - state.positions[i]) / 4.0;
                      }
                      double wheelRadius =
                          (state.gyroDelta * DriveConstants.driveBaseRadius) / wheelDelta;

                      Logger.recordOutput("Drive/WheelDelta", wheelDelta);
                      Logger.recordOutput("Drive/WheelRadius", wheelRadius);
                    })

                // When cancelled, calculate and print results
                .finallyDo(
                    () -> {
                      double[] positions = drive.getWheelRadiusCharacterizationPositions();
                      double wheelDelta = 0.0;
                      for (int i = 0; i < 4; i++) {
                        wheelDelta += Math.abs(positions[i] - state.positions[i]) / 4.0;
                      }
                      double wheelRadius =
                          (state.gyroDelta * DriveConstants.driveBaseRadius) / wheelDelta;

                      NumberFormat formatter = new DecimalFormat("#0.00000000");
                      System.out.println(
                          "********** Wheel Radius Characterization Results **********");
                      System.out.println(
                          "\tWheel Delta: " + formatter.format(wheelDelta) + " radians");
                      System.out.println(
                          "\tGyro Delta: " + formatter.format(state.gyroDelta) + " radians");
                      System.out.println(
                          "\tWheel Radius: "
                              + formatter.format(wheelRadius)
                              + " meters, "
                              + formatter.format(Units.metersToInches(wheelRadius))
                              + " inches");
                    })));
  }

  private static class WheelRadiusCharacterizationState {
    double[] positions = new double[4];
    Rotation2d lastAngle = Rotation2d.kZero;
    double gyroDelta = 0.0;
  }
}
