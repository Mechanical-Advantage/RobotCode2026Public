// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2026.subsystems.leds;

import static org.littletonrobotics.frc2026.subsystems.leds.LedConstants.*;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import java.lang.reflect.Field;
import java.util.List;
import java.util.Optional;
import org.littletonrobotics.frc2026.subsystems.leds.LedsIO.LedsIOOutputs;
import org.littletonrobotics.frc2026.util.LoggedTracer;
import org.littletonrobotics.frc2026.util.VirtualSubsystem;
import org.littletonrobotics.junction.Logger;

public class Leds extends VirtualSubsystem {
  private static Leds global;

  public static Leds getGlobal() {
    return global;
  }

  // Robot state tracking
  public boolean lowBatteryAlert = false;
  public boolean superstructureCoast = false;
  private boolean estopped = false;
  private Optional<Alliance> alliance = Optional.empty();
  private Color disabledColor = Color.kGold;
  private Color secondaryDisabledColor = Color.kDarkBlue;

  private final LedsIO io;
  private final LedsIOInputsAutoLogged inputs = new LedsIOInputsAutoLogged();
  private final LedsIOOutputs outputs = new LedsIOOutputs();

  private final AddressableLEDBuffer buffer = new AddressableLEDBuffer(length);
  private Field bufferField = null;

  // Constants
  private static final Section fullSection = new Section(0, length);

  public Leds(LedsIO io) {
    this.io = io;
    global = this;

    try {
      bufferField = AddressableLEDBuffer.class.getDeclaredField("m_buffer");
      bufferField.setAccessible(true);
    } catch (NoSuchFieldException | SecurityException e) {
      e.printStackTrace();
    }
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("LEDs", inputs);

    // Update alliance color
    if (DriverStation.isFMSAttached()) {
      alliance = DriverStation.getAlliance();
      disabledColor =
          alliance
              .map(alliance -> alliance == Alliance.Blue ? Color.kBlue : Color.kRed)
              .orElse(disabledColor);
      secondaryDisabledColor = alliance.isPresent() ? Color.kBlack : secondaryDisabledColor;
    }

    // Update estop state
    if (DriverStation.isEStopped()) {
      estopped = true;
    }

    // Select LED mode
    solid(fullSection, Color.kBlack); // Default to off

    // Update pattern
    if (estopped) {
      solid(fullSection, Color.kRed);
    } else if (DriverStation.isDisabled()) {
      if (superstructureCoast) {
        // Superstructure coast
        solid(fullSection, Color.kWhite);
      } else if (lowBatteryAlert) {
        // Low battery
        strobe(fullSection, Color.kOrangeRed, Color.kBlack, strobeDuration);
      } else if (prideLeds) {
        // Pride stripes
        stripes(
            fullSection,
            List.of(
                Color.kBlack,
                Color.kRed,
                Color.kOrangeRed,
                Color.kYellow,
                Color.kGreen,
                Color.kBlue,
                Color.kPurple,
                Color.kBlack,
                new Color(0.15, 0.3, 1.0),
                Color.kDeepPink,
                Color.kWhite,
                Color.kDeepPink,
                new Color(0.15, 0.3, 1.0)),
            3,
            5.0);
      } else {
        // Default pattern
        wave(
            fullSection,
            disabledColor,
            secondaryDisabledColor,
            waveDisabledCycleLength,
            waveDisabledDuration);
      }
    } else if (DriverStation.isAutonomous()) {
      wave(fullSection, Color.kGold, Color.kDarkBlue, waveFastCycleLength, waveFastDuration);
    } else {
      // Not implemented
    }

    // Override with loading animation
    if (Timer.getTimestamp() < 30.0) {
      breath(fullSection, Color.kBlack, Color.kWhite, startupBreathDuration, Timer.getTimestamp());
    }

    // Send to buffer
    if (bufferField == null) return;
    try {
      outputs.buffer = (byte[]) bufferField.get(buffer);

    } catch (IllegalArgumentException | IllegalAccessException e) {
      e.printStackTrace();
    }

    // Record cycle time
    LoggedTracer.record("LEDs");
  }

  @Override
  public void periodicAfterScheduler() {
    io.applyOutputs(outputs);
  }

  private Color solid(Section section, Color color) {
    if (color != null) {
      for (int i = section.start(); i < section.end(); i++) {
        setLED(i, color);
      }
    }
    return color;
  }

  private Color strobe(Section section, Color c1, Color c2, double duration) {
    boolean c1On = ((Timer.getTimestamp() % duration) / duration) > 0.5;
    if ((c1On && c1 == null) || (!c1On && c2 == null)) return null;
    return solid(section, c1On ? c1 : c2);
  }

  private Color breath(Section section, Color c1, Color c2, double duration, double timestamp) {
    Color color = breathCalculate(section, c1, c2, duration, timestamp);
    solid(section, color);
    return color;
  }

  private Color breathCalculate(
      Section section, Color c1, Color c2, double duration, double timestamp) {
    double x = ((timestamp % duration) / duration) * 2.0 * Math.PI;
    double ratio = (Math.sin(x) + 1.0) / 2.0;
    double red = (c1.red * (1 - ratio)) + (c2.red * ratio);
    double green = (c1.green * (1 - ratio)) + (c2.green * ratio);
    double blue = (c1.blue * (1 - ratio)) + (c2.blue * ratio);
    var color = new Color(red, green, blue);
    return color;
  }

  @SuppressWarnings("unused")
  private void rainbow(Section section, double cycleLength, double duration) {
    double x = (1 - ((Timer.getTimestamp() / duration) % 1.0)) * 180.0;
    double xDiffPerLed = 180.0 / cycleLength;
    for (int i = section.end() - 1; i >= section.start(); i--) {
      x += xDiffPerLed;
      x %= 180.0;
      setHSV(i, (int) x, 255, 255);
    }
  }

  private void wave(Section section, Color c1, Color c2, double cycleLength, double duration) {
    double x = (1 - ((Timer.getTimestamp() % duration) / duration)) * 2.0 * Math.PI;
    double xDiffPerLed = (2.0 * Math.PI) / cycleLength;
    x += xDiffPerLed * (length - section.end());
    for (int i = section.end() - 1; i >= section.start(); i--) {
      x += xDiffPerLed;
      double ratio = (Math.pow(Math.sin(x), waveExponent) + 1.0) / 2.0;
      if (Double.isNaN(ratio)) {
        ratio = (-Math.pow(Math.sin(x + Math.PI), waveExponent) + 1.0) / 2.0;
      }
      if (Double.isNaN(ratio)) {
        ratio = 0.5;
      }
      double red = (c1.red * (1 - ratio)) + (c2.red * ratio);
      double green = (c1.green * (1 - ratio)) + (c2.green * ratio);
      double blue = (c1.blue * (1 - ratio)) + (c2.blue * ratio);
      setLED(i, new Color(red, green, blue));
    }
  }

  private void stripes(Section section, List<Color> colors, int stripeLength, double duration) {
    int offset = (int) (Timer.getTimestamp() % duration / duration * stripeLength * colors.size());
    for (int i = section.end() - 1; i >= section.start(); i--) {
      int colorIndex =
          (int) (Math.floor((double) (i - offset) / stripeLength) + colors.size()) % colors.size();
      colorIndex = colors.size() - 1 - colorIndex;
      setLED(i, colors.get(colorIndex));
    }
  }

  private void setHSV(int index, int h, int s, int v) {
    setLED(index, Color.fromHSV(h, s, v));
  }

  private void setLED(int index, Color color) {
    buffer.setLED(index, color);
  }

  private record Section(int start, int end) {}
}
