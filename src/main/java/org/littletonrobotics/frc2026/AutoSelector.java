// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2026;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.ArrayList;
import java.util.List;
import lombok.Getter;
import org.littletonrobotics.frc2026.AutoFieldConstants.*;
import org.littletonrobotics.frc2026.util.SwitchableChooser;
import org.littletonrobotics.frc2026.util.VirtualSubsystem;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class AutoSelector extends VirtualSubsystem {
  private static final int maxQuestions = 6;
  private static final AutoRoutine defaultRoutine =
      new AutoRoutine("Posthumous Salesman", List.of(), Commands.none());

  private final LoggedDashboardChooser<AutoRoutine> routineChooser;
  private final List<StringPublisher> questionPublishers;
  private final List<SwitchableChooser> questionChoosers;

  private AutoRoutine lastRoutine;
  private List<AutoQuestionResponse> lastResponses;
  private boolean hasResponses = false;

  public AutoSelector(String key) {
    routineChooser = new LoggedDashboardChooser<>(key + "/Routine");
    routineChooser.addDefaultOption(defaultRoutine.name(), defaultRoutine);
    lastRoutine = defaultRoutine;
    lastResponses = List.of();

    // Publish questions and choosers
    questionPublishers = new ArrayList<>();
    questionChoosers = new ArrayList<>();
    for (int i = 0; i < maxQuestions; i++) {
      var publisher =
          NetworkTableInstance.getDefault()
              .getStringTopic("/SmartDashboard/" + key + "/Question #" + Integer.toString(i + 1))
              .publish();
      publisher.set("NA");
      questionPublishers.add(publisher);
      questionChoosers.add(
          new SwitchableChooser(key + "/Question #" + Integer.toString(i + 1) + " Chooser"));
    }
  }

  /** Registers a new auto routine that can be selected. */
  public void addRoutine(String name, Command command) {
    addRoutine(name, List.of(), command);
  }

  /** Registers a new auto routine that can be selected. */
  public void addRoutine(String name, List<AutoQuestion> questions, Command command) {
    if (questions.size() > maxQuestions) {
      throw new RuntimeException(
          "Auto routine contained more than "
              + Integer.toString(maxQuestions)
              + " questions: "
              + name);
    }
    routineChooser.addOption(name, new AutoRoutine(name, questions, command));
  }

  /** Returns the selected auto command. */
  public Command getCommand() {
    return lastRoutine.command();
  }

  /** Returns the name of the selected routine. */
  public String getSelectedName() {
    return lastRoutine.name();
  }

  /** Returns the selected question responses. */
  public List<AutoQuestionResponse> getResponses() {
    return lastResponses;
  }

  public void periodic() {
    // Skip updates when actively running in auto
    if (DriverStation.isAutonomousEnabled() && hasResponses) {
      return;
    }

    // Update the list of questions
    var selectedRoutine = routineChooser.get();
    if (selectedRoutine == null) {
      return;
    }
    if (!selectedRoutine.equals(lastRoutine)) {
      var questions = selectedRoutine.questions();
      for (int i = 0; i < maxQuestions; i++) {
        if (i < questions.size()) {
          questionPublishers.get(i).set(questions.get(i).question());
          questionChoosers
              .get(i)
              .setOptions(
                  questions.get(i).responses().stream()
                      .map((AutoQuestionResponse response) -> response.toString())
                      .toArray(String[]::new));
        } else {
          questionPublishers.get(i).set("");
          questionChoosers.get(i).setOptions(new String[] {});
        }
      }
    }

    // Update the routine and responses
    lastRoutine = selectedRoutine;
    lastResponses = new ArrayList<>();
    for (int i = 0; i < lastRoutine.questions().size(); i++) {
      String responseString = questionChoosers.get(i).get();
      lastResponses.add(
          responseString == null
              ? lastRoutine.questions().get(i).responses().get(0)
              : AutoQuestionResponse.valueOf(responseString));
    }

    // Mark that we have successfully read the responses at least once
    hasResponses = true;
  }

  public void periodicAfterScheduler() {}

  /** A customizable auto routine associated with a single command. */
  private static final record AutoRoutine(
      String name, List<AutoQuestion> questions, Command command) {}

  /** A question to ask for customizing an auto routine. */
  public static record AutoQuestion(String question, List<AutoQuestionResponse> responses) {}

  /** Responses to auto routine questions. */
  public static enum AutoQuestionResponse {
    LEFT_TRENCH(Trench.leftStart),
    LEFT_TRENCH_OFFSET(Trench.leftStartOffset),
    LEFT_BUMP(Bump.leftInner),
    CENTER(Hub.centerStart),
    RIGHT_BUMP(Bump.rightInner),
    RIGHT_TRENCH(Trench.rightStart),
    RIGHT_TRENCH_OFFSET(Trench.rightStartOffset),
    HUB("Hub"),
    BUMP("Bump"),
    TRENCH("Trench"),
    LEFT(),
    RIGHT(),
    LEFT_CLOSE(),
    RIGHT_CLOSE(),
    LEFT_NO_TRENCH(),
    RIGHT_NO_TRENCH(),
    CLOSEST(),
    SALESMAN(),
    SALESMAN_TURN(),
    CONSERVATIVE(),
    NEUTRAL(),
    FLIGHTLESS(),
    DAVIS(),
    DAVIS_FRIENDSHIP(),
    TILDE(),
    SALESMAN_FROM_BEHIND(),
    SALESMAN_FROM_BEHIND_FRIENDSHIP(),
    CORIOLIS(),
    DYNAMIC_ETHICAL(),
    DYNAMIC_UNETHICAL(),
    NOTHING(),
    YES(),
    NO(),
    BOTH(),
    NONE("None"),
    FULL_CLOSE();

    @Getter private final Translation2d translation;
    @Getter private final String name;

    AutoQuestionResponse(Translation2d translation) {
      this.translation = translation;
      name = null;
    }

    AutoQuestionResponse(String name) {
      this.translation = null;
      this.name = name;
    }

    AutoQuestionResponse() {
      translation = null;
      name = null;
    }
  }
}
