// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2026.commands.auto;

import static org.littletonrobotics.frc2026.commands.auto.AutoCommands.*;

import choreo.Choreo;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import lombok.RequiredArgsConstructor;
import org.littletonrobotics.frc2026.AutoFieldConstants;
import org.littletonrobotics.frc2026.AutoFieldConstants.*;
import org.littletonrobotics.frc2026.AutoSelector.AutoQuestionResponse;
import org.littletonrobotics.frc2026.FieldConstants;
import org.littletonrobotics.frc2026.FieldConstants.LinesVertical;
import org.littletonrobotics.frc2026.TrenchBounds;
import org.littletonrobotics.frc2026.salesman.SalesmanCommands;
import org.littletonrobotics.frc2026.salesman.SalesmanSolver;
import org.littletonrobotics.frc2026.subsystems.drive.Drive;
import org.littletonrobotics.frc2026.subsystems.drive.DriveConstants;
import org.littletonrobotics.frc2026.subsystems.hopper.Hopper;
import org.littletonrobotics.frc2026.subsystems.kicker.Kicker;
import org.littletonrobotics.frc2026.subsystems.launcher.LaunchCalculator;
import org.littletonrobotics.frc2026.subsystems.launcher.flywheel.Flywheel;
import org.littletonrobotics.frc2026.subsystems.launcher.hood.Hood;
import org.littletonrobotics.frc2026.subsystems.slamtake.Slamtake;
import org.littletonrobotics.frc2026.subsystems.slamtake.Slamtake.IntakeGoal;
import org.littletonrobotics.frc2026.subsystems.slamtake.Slamtake.SlamGoal;
import org.littletonrobotics.frc2026.util.ContinuousConditionalCommand;
import org.littletonrobotics.frc2026.util.SuppliedWaitCommand;
import org.littletonrobotics.frc2026.util.geometry.Bounds;

@RequiredArgsConstructor
@SuppressWarnings("unused")
public class AutoBuilder {
  private final Drive drive;
  private final Slamtake slamtake;
  private final Hopper hopper;
  private final Kicker kicker;
  private final Hood hood;
  private final Flywheel flywheel;
  private final SalesmanSolver salesmanSolver;

  private final Supplier<List<AutoQuestionResponse>> responses;
  private final DoubleSupplier offsetTime;

  public static final double neutralZoneIntakeTimeFirst = 2.5;
  public static final double neutralZoneIntakeTimeOther = 3.0;
  public static final double launchTime = 2.5;
  public static final double depotLaunchTime = 1.5;
  public static final double autoDuration = 21.0;

  private Supplier<Bounds> boundsSupplier(
      int boundsQuestionIndex, Supplier<AutoQuestionResponse> returnSide) {
    return () -> {
      switch (responses.get().get(boundsQuestionIndex)) {
        case DYNAMIC_ETHICAL:
          return getDynamicBounds(returnSide, true);
        case DYNAMIC_UNETHICAL:
          return getDynamicBounds(returnSide, false);
        case LEFT_CLOSE:
          return SalesmanSolver.leftBounds;
        case RIGHT_CLOSE:
          return SalesmanSolver.rightBounds;
        case FULL_CLOSE:
        default:
          return SalesmanSolver.defaultBounds;
      }
    };
  }

  static Map<AutoQuestionResponse, Double> coastTargetTimesThirdBump = new HashMap<>();
  static Map<AutoQuestionResponse, Double> coastTargetTimesThirdTrench = new HashMap<>();
  static Map<AutoQuestionResponse, Double> coastTargetTimesTilde = new HashMap<>();

  static {
    AutoQuestionResponse[] coastTargets = {
      AutoQuestionResponse.NONE,
      AutoQuestionResponse.HUB,
      AutoQuestionResponse.BUMP,
      AutoQuestionResponse.TRENCH
    };
    for (AutoQuestionResponse target : coastTargets) {
      Optional<Trajectory<SwerveSample>> trajectoryOptionalThirdBump =
          target.equals(AutoQuestionResponse.NONE)
              ? Choreo.loadTrajectory("NonPath")
              : Choreo.loadTrajectory("launchLeftBumpToFar%sKachow".formatted(target.getName()));
      Optional<Trajectory<SwerveSample>> trajectoryOptionalThirdTrench =
          target.equals(AutoQuestionResponse.NONE)
              ? Choreo.loadTrajectory("NonPath")
              : Choreo.loadTrajectory("launchLeftTrenchToFar%sKachow".formatted(target.getName()));
      Optional<Trajectory<SwerveSample>> trajectoryOptionalTilde =
          Choreo.loadTrajectory(
              "launchLeftBumpTildeSweepTo%s%sKachow"
                  .formatted(
                      target.equals(AutoQuestionResponse.NONE) ? "" : "Far", target.getName()));
      coastTargetTimesThirdBump.put(target, trajectoryOptionalThirdBump.get().getTotalTime());
      coastTargetTimesThirdTrench.put(target, trajectoryOptionalThirdTrench.get().getTotalTime());
      coastTargetTimesTilde.put(target, trajectoryOptionalTilde.get().getTotalTime());
    }
  }

  // MARK: Semple
  public Command sempleSalesman() {
    Supplier<AutoQuestionResponse> startPosition = () -> responses.get().get(0);
    Supplier<AutoQuestionResponse> firstSweep = () -> responses.get().get(1);
    Supplier<AutoQuestionResponse> secondSweep = () -> responses.get().get(2);
    Supplier<AutoQuestionResponse> thirdSweep = () -> AutoQuestionResponse.SALESMAN;
    Supplier<AutoQuestionResponse> returnResponse = () -> responses.get().get(3);
    final int boundsIndex = 4;

    BooleanSupplier shouldAim =
        () ->
            yCrossed(FieldConstants.LinesHorizontal.leftTrenchOpenEnd, false)
                    && yCrossed(FieldConstants.LinesHorizontal.rightTrenchOpenStart, true)
                ? xCrossed(
                    FieldConstants.LinesVertical.starting - DriveConstants.fullApothemX - 0.2,
                    false)
                : xCrossed(TrenchBounds.Hood.blueLeftTrench.minX(), false);

    return Commands.sequence(
            // Reset pose
            resetStartingPose(startPosition),

            // Wait if offset
            new SuppliedWaitCommand(offsetTime),

            // Sweep through neutral zone
            firstNeutralZoneSweep(
                drive,
                salesmanSolver,
                startPosition,
                firstSweep,
                boundsSupplier(boundsIndex, returnResponse),
                neutralZoneIntakeTimeFirst,
                neutralZoneIntakeTimeFirst - 1.0),

            // Return to alliance zone launch pose for first volley
            returnToDeterminedLaunchPose(drive, returnResponse)
                .raceWith(
                    Commands.waitUntil(
                        () ->
                            xCrossed(
                                FieldConstants.LinesVertical.starting - DriveConstants.fullApothemX,
                                false))),

            // Launch and complete second sweep
            lotmThroughNeutralZoneSweep(
                drive,
                salesmanSolver,
                secondSweep,
                returnResponse,
                boundsSupplier(boundsIndex, returnResponse),
                shouldAim,
                neutralZoneIntakeTimeOther,
                neutralZoneIntakeTimeOther - 1.0),

            // Return to alliance zone launch pose for second volley
            returnToDeterminedLaunchPose(drive, returnResponse)
                .raceWith(
                    Commands.waitUntil(
                        () ->
                            xCrossed(
                                FieldConstants.LinesVertical.starting - DriveConstants.fullApothemX,
                                false))),

            // Launch and complete attempt third sweep
            lotmThroughNeutralZoneSweep(
                drive,
                salesmanSolver,
                thirdSweep,
                returnResponse,
                boundsSupplier(boundsIndex, returnResponse),
                shouldAim,
                neutralZoneIntakeTimeOther,
                neutralZoneIntakeTimeOther - 1.0),

            // (Try to) Return to alliance zone launch pose for third volley
            returnToDeterminedLaunchPose(drive, returnResponse)
                .raceWith(
                    Commands.waitUntil(
                        () ->
                            xCrossed(
                                FieldConstants.LinesVertical.starting - DriveConstants.fullApothemX,
                                false))))
        .deadlineFor(
            new ContinuousConditionalCommand(
                indexMindfully(
                    hopper,
                    kicker,
                    flywheel,
                    slamtake,
                    () -> AutoCommands.withinLaunchingTolerance(Rotation2d.fromDegrees(3.0))),
                Commands.sequence(
                    Commands.waitUntil(
                        () -> xCrossed(FieldConstants.LeftTrench.center.getX(), true)),
                    Commands.run(
                        () -> {
                          slamtake.setSlamGoal(SlamGoal.DEPLOY);
                          slamtake.setIntakeGoal(IntakeGoal.INTAKE);
                        }),
                    Commands.idle()),
                () -> shouldAim.getAsBoolean()));
  }

  // MARK: Kachow
  public Command kachowSalesman() {
    Supplier<AutoQuestionResponse> firstSweep = () -> responses.get().get(1);
    Supplier<AutoQuestionResponse> secondSweep = () -> responses.get().get(2);
    Supplier<AutoQuestionResponse> returnResponse = () -> responses.get().get(3);
    Supplier<AutoQuestionResponse> coastTarget = () -> responses.get().get(4);
    final int boundsIndex = 5;
    Supplier<AutoQuestionResponse> startPosition =
        () -> {
          if (firstSweep.get().equals(AutoQuestionResponse.FLIGHTLESS)) {
            if (responses.get().get(0).equals(AutoQuestionResponse.LEFT_TRENCH)) {
              return AutoQuestionResponse.LEFT_TRENCH_OFFSET;
            } else if (responses.get().get(0).equals(AutoQuestionResponse.RIGHT_TRENCH)) {
              return AutoQuestionResponse.RIGHT_TRENCH_OFFSET;
            }
          }
          return responses.get().get(0);
        };

    DoubleSupplier earliestLeaveForCoast =
        () ->
            autoDuration
                - (secondSweep.get().equals(AutoQuestionResponse.TILDE)
                    ? coastTargetTimesTilde.get(coastTarget.get())
                    : coastTargetTimesThirdBump.get(coastTarget.get()));
    Timer autoTimer = new Timer();

    return Commands.sequence(
        // Reset timer
        Commands.runOnce(autoTimer::restart),

        // Reset pose
        resetStartingPose(startPosition),

        // Wait if offset
        new SuppliedWaitCommand(offsetTime),

        // Sweep through neutral zone
        firstNeutralZoneSweep(
            drive,
            salesmanSolver,
            startPosition,
            firstSweep,
            boundsSupplier(boundsIndex, returnResponse),
            neutralZoneIntakeTimeFirst,
            neutralZoneIntakeTimeFirst - 1.0),

        // Return to alliance zone launch pose for first volley
        returnToDeterminedLaunchPose(drive, returnResponse)
            .raceWith(
                Commands.waitUntil(
                    () ->
                        xCrossed(
                            FieldConstants.LinesVertical.starting
                                - DriveConstants.fullBaseRadius
                                + 0.1,
                            false))),

        // Shoot from static location
        returnLaunchAndIndexMindfully(
            drive,
            hopper,
            kicker,
            flywheel,
            slamtake,
            false,
            () ->
                isLeftSide(returnResponse).getAsBoolean()
                    ? AutoQuestionResponse.LEFT_BUMP
                    : AutoQuestionResponse.RIGHT_BUMP,
            () ->
                secondSweep.get().equals(AutoQuestionResponse.TILDE)
                    ? autoTimer.get() >= earliestLeaveForCoast.getAsDouble()
                    : true),

        // Re-deploy slamtake before entering trench
        Commands.runOnce(
            () -> {
              slamtake.setSlamGoal(SlamGoal.DEPLOY);
              slamtake.setIntakeGoal(IntakeGoal.INTAKE);
            }),

        // Launch and complete second sweep
        Commands.either(
            // If taking a third bot trajectory, set up coast
            Commands.sequence(
                    Commands.waitUntil(
                        () -> xCrossed(FieldConstants.LinesVertical.neutralZoneNear, true)),
                    Commands.runOnce(() -> drive.setCoastRequest(Drive.CoastRequest.ALWAYS_COAST)),
                    Commands.idle())
                .deadlineFor(
                    Commands.select(
                        Map.of(
                            AutoQuestionResponse.HUB,
                            followTrajectory(
                                "launchLeftBumpTildeSweepToFarHubKachow",
                                drive,
                                false,
                                () -> !isLeftSide(returnResponse).getAsBoolean()),
                            AutoQuestionResponse.BUMP,
                            followTrajectory(
                                "launchLeftBumpTildeSweepToFarBumpKachow",
                                drive,
                                false,
                                () -> !isLeftSide(returnResponse).getAsBoolean()),
                            AutoQuestionResponse.TRENCH,
                            followTrajectory(
                                "launchLeftBumpTildeSweepToFarTrenchKachow",
                                drive,
                                false,
                                () -> !isLeftSide(returnResponse).getAsBoolean()),
                            AutoQuestionResponse.NONE,
                            followTrajectory(
                                "launchLeftBumpTildeSweepToNoneKachow",
                                drive,
                                false,
                                () -> !isLeftSide(returnResponse).getAsBoolean())),
                        coastTarget)),

            // If taking normal second sweep
            launchThroughNeutralZoneSweep(
                drive,
                salesmanSolver,
                secondSweep,
                returnResponse,
                boundsSupplier(boundsIndex, returnResponse),
                neutralZoneIntakeTimeOther,
                neutralZoneIntakeTimeOther - 1.0),
            () -> secondSweep.get().equals(AutoQuestionResponse.TILDE)),

        // Return to alliance zone launch pose for second volley
        returnToDeterminedLaunchPose(drive, returnResponse)
            .raceWith(
                Commands.waitUntil(
                    () ->
                        xCrossed(
                            FieldConstants.LinesVertical.starting
                                - DriveConstants.fullBaseRadius
                                + 0.1,
                            false))),

        // Shoot from static location
        returnLaunchAndIndexMindfully(
            drive,
            hopper,
            kicker,
            flywheel,
            slamtake,
            false,
            () ->
                isLeftSide(returnResponse).getAsBoolean()
                    ? AutoQuestionResponse.LEFT_BUMP
                    : AutoQuestionResponse.RIGHT_BUMP,
            () -> autoTimer.get() >= earliestLeaveForCoast.getAsDouble()),

        // Re-deploy slamtake before entering trench
        Commands.runOnce(
            () -> {
              slamtake.setSlamGoal(SlamGoal.DEPLOY);
              slamtake.setIntakeGoal(IntakeGoal.INTAKE);
            }),

        // Coast to opponent hub
        driveToCoastTarget(
                drive,
                coastTarget,
                () -> false,
                () -> !isLeftSide(returnResponse).getAsBoolean(),
                true)
            .deadlineFor(
                Commands.sequence(
                    Commands.waitUntil(
                        () -> xCrossed(FieldConstants.LinesVertical.neutralZoneNear, true)),
                    Commands.runOnce(
                        () -> drive.setCoastRequest(Drive.CoastRequest.ALWAYS_COAST)))));
  }

  // MARK: Benevolent
  public Command benevolentSalesman() {
    Supplier<AutoQuestionResponse> startPosition = () -> responses.get().get(0);
    Supplier<AutoQuestionResponse> sweepMode = () -> responses.get().get(1);
    int boundsIndex = 2;
    Supplier<AutoQuestionResponse> passingMode = () -> responses.get().get(3);

    return Commands.sequence(
        // Reset pose
        resetStartingPose(startPosition),

        // Wait if offset
        new SuppliedWaitCommand(offsetTime),

        // Sweep through neutral zone
        firstNeutralZoneSweep(
            drive,
            salesmanSolver,
            startPosition,
            sweepMode,
            boundsSupplier(boundsIndex, startPosition),
            2.5,
            1.5),

        // Passing cycles
        Commands.repeatingSequence(
            // Pass back to alliance zone
            passingCommand(drive, hopper, kicker, flywheel, slamtake, passingMode, launchTime),
            // Employ Salesman
            SalesmanCommands.autoIntake(
                    drive, salesmanSolver, boundsSupplier(boundsIndex, startPosition))
                .raceWith(Commands.waitSeconds(neutralZoneIntakeTimeOther))));
  }

  // MARK: Mellonomics
  public Command mellonomicsSalesman() {
    Supplier<AutoQuestionResponse> startPosition = () -> responses.get().get(0);
    Supplier<AutoQuestionResponse> firstSweep = () -> responses.get().get(1);
    Supplier<AutoQuestionResponse> secondSweep = () -> responses.get().get(2);
    Supplier<AutoQuestionResponse> returnResponse = () -> responses.get().get(3);
    int boundsIndex = 4;
    Supplier<AutoQuestionResponse> passingMode = () -> responses.get().get(5);

    BooleanSupplier shouldAim =
        () ->
            yCrossed(FieldConstants.LinesHorizontal.leftTrenchOpenEnd, false)
                    && yCrossed(FieldConstants.LinesHorizontal.rightTrenchOpenStart, true)
                ? xCrossed(
                    FieldConstants.LinesVertical.starting - DriveConstants.fullApothemX, false)
                : xCrossed(Trench.leftEntry.getX(), false);

    return Commands.sequence(
        // Reset pose
        resetStartingPose(startPosition),

        // Wait if offset
        new SuppliedWaitCommand(offsetTime),

        // Sweep through neutral zone
        firstNeutralZoneSweep(
            drive,
            salesmanSolver,
            startPosition,
            firstSweep,
            boundsSupplier(boundsIndex, returnResponse),
            2.5,
            1.5),

        // Pass back to alliance zone
        passingCommand(drive, hopper, kicker, flywheel, slamtake, passingMode, launchTime),

        // Employ salesman
        SalesmanCommands.autoIntake(
                drive, salesmanSolver, boundsSupplier(boundsIndex, returnResponse))
            .withTimeout(neutralZoneIntakeTimeOther),

        // Return to alliance zone launch pose for first volley
        returnToDeterminedLaunchPose(drive, returnResponse)
            .raceWith(
                Commands.waitUntil(
                    () ->
                        xCrossed(
                            FieldConstants.LinesVertical.starting - DriveConstants.fullApothemX,
                            false))),

        // Launch and complete second sweep
        lotmThroughNeutralZoneSweep(
                drive,
                salesmanSolver,
                secondSweep,
                returnResponse,
                boundsSupplier(boundsIndex, returnResponse),
                shouldAim,
                neutralZoneIntakeTimeOther,
                neutralZoneIntakeTimeOther - 1.0)
            .deadlineFor(
                new ContinuousConditionalCommand(
                    indexMindfully(
                        hopper,
                        kicker,
                        flywheel,
                        slamtake,
                        () -> AutoCommands.withinLaunchingTolerance(Rotation2d.fromDegrees(8.0))),
                    Commands.sequence(
                        Commands.waitUntil(() -> xCrossed(LinesVertical.neutralZoneNear, true)),
                        Commands.run(
                            () -> {
                              slamtake.setSlamGoal(SlamGoal.DEPLOY);
                              slamtake.setIntakeGoal(IntakeGoal.INTAKE);
                            }),
                        Commands.idle()),
                    () -> shouldAim.getAsBoolean())),

        // Return to alliance zone launch pose to attempt second volley
        returnToDeterminedLaunchPose(drive, returnResponse)
            .raceWith(
                Commands.waitUntil(
                    () ->
                        xCrossed(
                            FieldConstants.LinesVertical.starting - DriveConstants.fullApothemX,
                            false))));
  }

  // MARK: Substantial
  public Command substantialSalesman() {
    Supplier<AutoQuestionResponse> startPosition = () -> responses.get().get(0);
    Supplier<AutoQuestionResponse> sweepMode = () -> responses.get().get(1);
    Supplier<AutoQuestionResponse> returnResponse = () -> responses.get().get(2);
    Supplier<AutoQuestionResponse> coastTarget = () -> responses.get().get(3);
    int boundsIndex = 4;

    BooleanSupplier isBump = () -> returnResponse.get().equals(AutoQuestionResponse.LEFT_BUMP);

    DoubleSupplier earliestLeaveForCoast =
        () ->
            autoDuration
                - (isBump.getAsBoolean()
                    ? coastTarget.get().equals(AutoQuestionResponse.NONE) ? 0.0 : 1.5
                    : coastTargetTimesThirdTrench.get(coastTarget.get()));
    Timer autoTimer = new Timer();

    return Commands.sequence(
        // Reset timer
        Commands.runOnce(autoTimer::restart),

        // Reset pose
        resetStartingPose(startPosition),

        // Wait if offset
        new SuppliedWaitCommand(offsetTime),

        // Sweep through neutral zone
        firstNeutralZoneSweep(
            drive,
            salesmanSolver,
            startPosition,
            sweepMode,
            boundsSupplier(boundsIndex, () -> AutoQuestionResponse.LEFT_BUMP),
            neutralZoneIntakeTimeFirst,
            neutralZoneIntakeTimeFirst - 1.0),

        // Return to alliance zone launch pose for first volley
        returnToDeterminedLaunchPose(drive, () -> AutoQuestionResponse.LEFT_BUMP)
            .raceWith(
                Commands.waitUntil(
                    () ->
                        xCrossed(
                            FieldConstants.LinesVertical.starting
                                - DriveConstants.fullBaseRadius
                                + 0.1,
                            false))),

        // Shoot from static location
        returnLaunchAndIndexMindfully(
            drive,
            hopper,
            kicker,
            flywheel,
            slamtake,
            false,
            () ->
                isLeftSide(returnResponse).getAsBoolean()
                    ? AutoQuestionResponse.LEFT_BUMP
                    : AutoQuestionResponse.RIGHT_BUMP,
            () -> true),

        // Re-deploy slamtake before intaking from depot
        Commands.runOnce(
            () -> {
              slamtake.setSlamGoal(SlamGoal.DEPLOY);
              slamtake.setIntakeGoal(IntakeGoal.INTAKE);
            }),

        // Follow intake and drive-up trajectory
        Commands.either(
            followTrajectory("launchLeftBumpThroughDepotToLaunchLeftBump", drive, false),
            followTrajectory("launchLeftBumpThroughDepotToLaunchLeftTrench", drive, false),
            isBump),

        // Shoot from static location
        Commands.either(
            returnLaunchAndIndexMindfully(
                drive,
                hopper,
                kicker,
                flywheel,
                slamtake,
                true,
                returnResponse,
                () -> autoTimer.get() >= earliestLeaveForCoast.getAsDouble()),
            returnLaunchAndIndexMindfully(
                drive,
                hopper,
                kicker,
                flywheel,
                slamtake,
                false,
                returnResponse,
                () -> autoTimer.get() >= earliestLeaveForCoast.getAsDouble()),
            () -> autoTimer.get() >= earliestLeaveForCoast.getAsDouble() - launchTime),

        // Coast target
        Commands.sequence(
                Commands.waitUntil(
                    () -> xCrossed(FieldConstants.LinesVertical.neutralZoneNear, true)),
                Commands.runOnce(() -> drive.setCoastRequest(Drive.CoastRequest.ALWAYS_COAST)),
                Commands.idle())
            .deadlineFor(
                driveToCoastTarget(drive, coastTarget, isBump, () -> false, false),
                Commands.sequence(
                    Commands.waitUntil(
                        () ->
                            xCrossed(
                                FieldConstants.LinesVertical.starting - DriveConstants.fullApothemX,
                                true)),
                    Commands.runOnce(
                        () -> {
                          slamtake.setSlamGoal(SlamGoal.DEPLOY);
                          slamtake.setIntakeGoal(IntakeGoal.INTAKE);
                        }))));
  }

  // MARK: Monopoly
  public Command monopolySalesman() {
    Supplier<AutoQuestionResponse> startPosition = () -> responses.get().get(0);
    Supplier<AutoQuestionResponse> postLaunch = () -> responses.get().get(1);

    return Commands.sequence(
        // Drive to closest intaking position
        Commands.select(
            Map.of(
                AutoQuestionResponse.LEFT_TRENCH,
                followTrajectory("trenchLeftStartThroughDepot", drive, true),
                AutoQuestionResponse.LEFT_BUMP,
                followTrajectory("bumpLeftInnerThroughDepot", drive, true)),
            startPosition),

        // Drive to and launch from launch pose
        AutoCommands.driveToPose(drive, () -> Launch.leftTower)
            .raceWith(
                Commands.sequence(
                    AutoCommands.waitUntilWithinTolerance(
                        Launch.leftTower, 0.1, Rotation2d.fromDegrees(5)),
                    index(hopper, kicker, flywheel, slamtake))),

        // Initiate chosen end behavior
        Commands.select(
            Map.of(
                AutoQuestionResponse.NOTHING,
                Commands.none(),
                AutoQuestionResponse.SALESMAN,
                salesmanCycle(
                    drive,
                    salesmanSolver,
                    hopper,
                    kicker,
                    flywheel,
                    slamtake,
                    () ->
                        isLeftSide(postLaunch).getAsBoolean()
                            ? AutoQuestionResponse.LEFT_BUMP
                            : AutoQuestionResponse.RIGHT_BUMP,
                    boundsSupplier(2, () -> responses.get().get(0)),
                    true,
                    false)),
            () -> responses.get().get(1)));
  }

  // MARK: Timid
  public Command timidSalesman() {
    Supplier<Pose2d> target =
        () -> {
          Translation2d offset = new Translation2d();
          switch (responses.get().get(0)) {
            case CENTER:
            case LEFT_BUMP:
            case RIGHT_BUMP:
              offset = new Translation2d(-1, 0);
              break;
            case LEFT_TRENCH:
              offset = new Translation2d(-1, -0.5);
              break;
            case RIGHT_TRENCH:
              offset = new Translation2d(-1, 0.5);
              break;
            default:
              break;
          }

          return LaunchCalculator.getStationaryAimedPose(
              responses.get().get(0).getTranslation().plus(offset), true);
        };

    return Commands.sequence(
        // Reset pose
        Commands.select(
            Map.of(
                AutoQuestionResponse.LEFT_TRENCH,
                resetPose(
                    new Pose2d(
                        AutoFieldConstants.Trench.leftStart.minus(
                            new Translation2d(DriveConstants.fullWidthX, 0.0)),
                        Rotation2d.kPi)),
                AutoQuestionResponse.LEFT_BUMP,
                resetPose(new Pose2d(AutoFieldConstants.Bump.leftInner, Rotation2d.kPi)),
                AutoQuestionResponse.CENTER,
                resetPose(new Pose2d(AutoFieldConstants.Hub.centerStart, Rotation2d.kPi)),
                AutoQuestionResponse.RIGHT_BUMP,
                resetPose(
                    new Pose2d(
                        AutoFieldConstants.Bump.rightInner.minus(
                            new Translation2d(DriveConstants.fullWidthX, 0.0)),
                        Rotation2d.kPi)),
                AutoQuestionResponse.RIGHT_TRENCH,
                resetPose(new Pose2d(AutoFieldConstants.Trench.rightStart, Rotation2d.kPi))),
            () -> responses.get().get(0)),

        // Drive backwards a little bit and launch
        Commands.parallel(
            AutoCommands.driveToPose(drive, target),
            Commands.sequence(
                waitUntilWithinTolerance(target, 0.1, Rotation2d.fromDegrees(5)),
                index(hopper, kicker, flywheel, slamtake))));
  }

  // MARK: Drive Forward 1m
  public Command driveForward1mSalesman() {
    return followTrajectory("DriveForward1m", drive, true);
  }
}
