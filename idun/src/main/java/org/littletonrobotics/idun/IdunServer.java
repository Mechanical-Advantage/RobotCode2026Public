// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.idun;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.hal.DriverStationJNI;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.MatchType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.net.SocketException;
import java.nio.ByteBuffer;
import java.util.Random;
import java.util.concurrent.locks.Condition;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import org.littletonrobotics.idun.IdunComms.IOData;
import org.littletonrobotics.idun.IdunComms.JoystickOutputs;
import org.littletonrobotics.idun.IdunComms.RobotInputs;
import org.littletonrobotics.idun.IdunComms.RobotOutputs;
import org.littletonrobotics.junction.Logger;

public class IdunServer {
  private static RobotInputs inputs = RobotInputs.getDefaultInstance();
  private static RobotOutputs.Builder outputsBuilder = RobotOutputs.newBuilder();
  private static long launchUID = new Random().nextLong();

  private static DatagramSocket socket;
  private static InetAddress clientAddress;
  private static int clientPort;
  private static boolean lastConnected = false;

  private static double lastMessageTimestamp = -1.0;
  private static double lastBuildUIDWarning = -1.0;
  private static int incomingBytes = 0;
  private static int outgoingBytes = 0;
  private static Timer bandwidthTimer = new Timer();

  private static double lastPeriodic = -1.0;
  private static MedianFilter loopCycleFrequencyFilter = new MedianFilter(10);

  private static final Lock inputsLock = new ReentrantLock();
  private static final Condition newInputsCondition = inputsLock.newCondition();

  /** Start waiting for the client. */
  public static void start() {
    if (socket != null) return;

    try {
      socket = new DatagramSocket(IdunNetworkConstants.serverPort);
    } catch (SocketException e) {
      e.printStackTrace();
    }

    if (socket != null) {
      var serverThread =
          new Thread(
              () -> {
                System.out.println(
                    "[IdunServer] Socket ready on port "
                        + Integer.toString(IdunNetworkConstants.serverPort)
                        + ", waiting for client...");
                byte[] buffer = new byte[IdunNetworkConstants.maxPacketSizeBytes];
                DatagramPacket packet = new DatagramPacket(buffer, buffer.length);
                while (true) {
                  try {
                    // Wait for a packet
                    socket.receive(packet);

                    // A new client is sending data
                    if (clientAddress == null
                        || !clientAddress.equals(packet.getAddress())
                        || clientPort != packet.getPort()) {
                      System.out.println(
                          "[IdunServer] Connected to client "
                              + packet.getAddress().getHostAddress()
                              + ":"
                              + packet.getPort());
                      bandwidthTimer.reset();
                    }

                    // Store client address to send responses
                    clientAddress = packet.getAddress();
                    clientPort = packet.getPort();

                    // Parse message from packet
                    var newInputs =
                        RobotInputs.parseFrom(
                            ByteBuffer.wrap(packet.getData(), 0, packet.getLength()));

                    // Check build UID
                    if (IdunPlatform.isRobot && newInputs.getBuildUID() != IdunBuildConstants.uid) {
                      if (Timer.getFPGATimestamp() - lastBuildUIDWarning > 1.0) {
                        System.out.println(
                            "[IdunServer] Server and client UIDs do not match, ignoring message (!!!)");
                        lastBuildUIDWarning = Timer.getFPGATimestamp();
                      }
                      continue;
                    }

                    // Update inputs
                    inputsLock.lock();
                    try {
                      inputs = newInputs;
                      lastMessageTimestamp = Timer.getFPGATimestamp();
                      incomingBytes += packet.getLength();
                      newInputsCondition.signalAll(); // Wake up the waiting thread
                    } finally {
                      inputsLock.unlock();
                    }
                  } catch (IOException e) {
                    clientAddress = null;
                    System.out.println(
                        "[IdunServer] Error receiving packet, waiting for reconnection...");
                    e.printStackTrace();
                  }
                }
              });

      serverThread.setName("IdunServer");
      serverThread.setDaemon(true);
      serverThread.start();
      bandwidthTimer.start();
    }
  }

  /**
   * Blocks until the next set of inputs is received or the timeout is reached.
   *
   * @param timeoutUs The maximum time to wait in microseconds.
   * @return True if the inputs were received, or false if the timeout was reached.
   */
  public static boolean waitForInputs(long timeoutUs) {
    long timeoutNanos = (long) (timeoutUs * 1000L);
    inputsLock.lock();
    try {
      // Loop until the next message is received, since awaitNanos can exit early due to "spurious
      // wakeups". This code structure is recommended by the Javadoc comment for awaitNanos.
      double initialTimestamp = lastMessageTimestamp;
      while (lastMessageTimestamp == initialTimestamp) {
        if (timeoutNanos <= 0L) {
          return false; // Timeout expired but no message received
        }
        timeoutNanos = newInputsCondition.awaitNanos(timeoutNanos);
      }
      return true; // New message was received
    } catch (InterruptedException e) {
      Thread.currentThread().interrupt();
      return false;
    } finally {
      inputsLock.unlock();
    }
  }

  /** Returns whether the server is currently connected to the client. */
  public static boolean isConnected() {
    return clientAddress != null
        && Timer.getFPGATimestamp() - lastMessageTimestamp
            < (double) IdunNetworkConstants.timeoutMs / 1000.0;
  }

  /** Reads the latest set of inputs for a subsystem, or null if the name is not found. */
  public static IOData getSubsystemInputs(String name) {
    inputsLock.lock();
    try {
      return inputs.getSubsystemOrDefault(name, null);
    } finally {
      inputsLock.unlock();
    }
  }

  /** Writes a new set of outputs for a subsystem. */
  public static void setSubsystemOutputs(String name, IOData data) {
    outputsBuilder.putSubsystem(name, data);
  }

  /** Sets the offset that can be added to the gyro to get the robot rotation. */
  public static void setGyroOffset(Rotation2d offset) {
    outputsBuilder.setGyroOffset(offset.getRadians());
  }

  /** Requests that the robot code on the client exit and restart. */
  public static void requestClientCodeRestart() {
    outputsBuilder.setRestartFlag(true);
  }

  /** Returns whether the RIO is browned out. */
  public static boolean isRoboRioBrownedOut() {
    if (!isConnected()) return false;

    inputsLock.lock();
    try {
      return inputs.getRio().getBrownedOut();
    } finally {
      inputsLock.unlock();
    }
  }

  /** Process incoming messages from the server. */
  public static void processIncoming() {
    // Measure loop cycle frequency
    double now = Timer.getFPGATimestamp();
    if (lastPeriodic > 0.0) {
      Logger.recordOutput(
          "Idun/LoopCycleFrequency",
          loopCycleFrequencyFilter.calculate(1.0 / (now - lastPeriodic)));
    }
    lastPeriodic = now;

    // Read inputs
    inputsLock.lock();
    try {
      var rioInputs = inputs.getRio();
      var dsInputs = inputs.getDs();
      var tracerInputs = inputs.getTracer();

      // Copy sequence number to outputs
      outputsBuilder.setSeqnum(inputs.getSeqnum());

      // Update sim state
      RoboRioSim.setVInVoltage(rioInputs.getBatteryVoltage());
      RoboRioSim.setVInCurrent(rioInputs.getBatteryCurrent());
      DriverStationSim.setAllianceStationId(
          switch (dsInputs.getAllianceStation()) {
            case DriverStationJNI.kRed1AllianceStation -> AllianceStationID.Red1;
            case DriverStationJNI.kRed2AllianceStation -> AllianceStationID.Red2;
            case DriverStationJNI.kRed3AllianceStation -> AllianceStationID.Red3;
            case DriverStationJNI.kBlue1AllianceStation -> AllianceStationID.Blue1;
            case DriverStationJNI.kBlue2AllianceStation -> AllianceStationID.Blue2;
            case DriverStationJNI.kBlue3AllianceStation -> AllianceStationID.Blue3;
            default -> AllianceStationID.Unknown;
          });
      DriverStationSim.setEventName(dsInputs.getEventName());
      DriverStationSim.setGameSpecificMessage(dsInputs.getGameSpecificMessage());
      DriverStationSim.setMatchNumber(dsInputs.getMatchNumber());
      DriverStationSim.setReplayNumber(dsInputs.getReplayNumber());
      DriverStationSim.setMatchType(
          switch (dsInputs.getMatchType()) {
            case 1 -> MatchType.Practice;
            case 2 -> MatchType.Qualification;
            case 3 -> MatchType.Elimination;
            default -> MatchType.None;
          });
      DriverStationSim.setMatchTime(dsInputs.getMatchTime());

      var controlWord = dsInputs.getControlWord();
      boolean dsAttached =
          controlWord.getDsAttached()
              && isConnected(); // Disconnected RIO is equivalent to not receiving DS packets
      DriverStationSim.setEnabled(controlWord.getEnabled());
      DriverStationSim.setAutonomous(controlWord.getAutonomous());
      DriverStationSim.setTest(controlWord.getTest());
      DriverStationSim.setEStop(controlWord.getEstop());
      DriverStationSim.setFmsAttached(controlWord.getFmsAttached());
      DriverStationSim.setDsAttached(dsAttached);

      for (int id = 0; id < dsInputs.getJoystickCount(); id++) {
        var joystick = dsInputs.getJoystick(id);
        DriverStationSim.setJoystickName(id, joystick.getName());
        DriverStationSim.setJoystickType(id, joystick.getType());
        DriverStationSim.setJoystickIsXbox(id, joystick.getIsXbox());
        DriverStationSim.setJoystickButtonCount(id, joystick.getButtonCount());
        DriverStationSim.setJoystickButtons(id, joystick.getButtons());
        DriverStationSim.setJoystickPOVCount(id, joystick.getPovCount());
        for (int i = 0; i < joystick.getPovValueCount(); i++) {
          DriverStationSim.setJoystickPOV(id, i, joystick.getPovValue(i));
        }
        DriverStationSim.setJoystickAxisCount(id, joystick.getAxisCount());
        for (int i = 0; i < joystick.getAxisTypeCount(); i++) {
          DriverStationSim.setJoystickAxisType(id, i, joystick.getAxisType(i));
        }
        for (int i = 0; i < joystick.getAxisValueCount(); i++) {
          DriverStationSim.setJoystickAxis(id, i, joystick.getAxisValue(i));
        }
      }

      if (dsAttached) {
        DriverStationSim.notifyNewData();
      }

      // Log RIO inputs
      Logger.recordOutput("Idun/roboRIO/SystemActive", rioInputs.getSystemActive());
      Logger.recordOutput("Idun/roboRIO/BrownedOut", rioInputs.getBrownedOut());
      Logger.recordOutput("Idun/roboRIO/CommsDisableCount", rioInputs.getCommsDisableCount());
      Logger.recordOutput("Idun/roboRIO/RSLState", rioInputs.getRslState());
      Logger.recordOutput("Idun/roboRIO/BatteryVoltage", rioInputs.getBatteryVoltage());
      Logger.recordOutput("Idun/roboRIO/BatteryCurrent", rioInputs.getBatteryCurrent());

      Logger.recordOutput("Idun/roboRIO/3v3Rail/Voltage", rioInputs.getRail3V3().getVoltage());
      Logger.recordOutput("Idun/roboRIO/3v3Rail/Current", rioInputs.getRail3V3().getCurrent());
      Logger.recordOutput("Idun/roboRIO/3v3Rail/Active", rioInputs.getRail3V3().getActive());
      Logger.recordOutput(
          "Idun/roboRIO/3v3Rail/CurrentFaults", rioInputs.getRail3V3().getCurrentFaults());

      Logger.recordOutput("Idun/roboRIO/5vRail/Voltage", rioInputs.getRail5V().getVoltage());
      Logger.recordOutput("Idun/roboRIO/5vRail/Current", rioInputs.getRail5V().getCurrent());
      Logger.recordOutput("Idun/roboRIO/5vRail/Active", rioInputs.getRail5V().getActive());
      Logger.recordOutput(
          "Idun/roboRIO/5vRail/CurrentFaults", rioInputs.getRail5V().getCurrentFaults());

      Logger.recordOutput("Idun/roboRIO/6vRail/Voltage", rioInputs.getRail6V().getVoltage());
      Logger.recordOutput("Idun/roboRIO/6vRail/Current", rioInputs.getRail6V().getCurrent());
      Logger.recordOutput("Idun/roboRIO/6vRail/Active", rioInputs.getRail6V().getActive());
      Logger.recordOutput(
          "Idun/roboRIO/6vRail/CurrentFaults", rioInputs.getRail6V().getCurrentFaults());

      rioInputs
          .getCanStatusMap()
          .forEach(
              (name, status) -> {
                Logger.recordOutput(
                    "Idun/roboRIO/CAN/" + name + "/Utilization", status.getUtilization());
                Logger.recordOutput("Idun/roboRIO/CAN/" + name + "/OffCount", status.getOffCount());
                Logger.recordOutput(
                    "Idun/roboRIO/CAN/" + name + "/TxFullCount", status.getTxFullCount());
                Logger.recordOutput(
                    "Idun/roboRIO/CAN/" + name + "/ReceiveErrorCount",
                    status.getReceiveErrorCount());
                Logger.recordOutput(
                    "Idun/roboRIO/CAN/" + name + "/TransmitErrorCount",
                    status.getTransmitErrorCount());
              });
      Logger.recordOutput("Idun/roboRIO/EpochTimeMicros", rioInputs.getEpochTimeMicros());

      // Log other built-in inputs
      Logger.recordOutput("Idun/SequenceNum", inputs.getSeqnum());
      Logger.recordOutput("Idun/BuildUIDs/MacMini", IdunBuildConstants.uid);
      Logger.recordOutput("Idun/BuildUIDs/roboRIO", inputs.getBuildUID());
      Logger.recordOutput("Idun/LaunchUIDs/MacMini", launchUID);
      Logger.recordOutput("Idun/LaunchUIDs/roboRIO", inputs.getLaunchUID());
      Logger.recordOutput("Idun/RIODrive", inputs.getIsLocalDrive());
      Logger.recordOutput("Idun/roboRIO/InputCycleMS", tracerInputs.getInputCycleMs());
      Logger.recordOutput("Idun/roboRIO/Tracer/DSAlertMS", tracerInputs.getDsAlertMs());
      Logger.recordOutput(
          "Idun/roboRIO/Tracer/DisconnectedStopMS", tracerInputs.getDisconnectedStopMs());
      Logger.recordOutput(
          "Idun/roboRIO/Tracer/PhoenixRefreshMS", tracerInputs.getPhoenixRefreshMs());
      Logger.recordOutput(
          "Idun/roboRIO/Tracer/SubsystemInputsMS", tracerInputs.getSubsystemInputsMs());
      Logger.recordOutput("Idun/roboRIO/Tracer/LocalDriveMS", tracerInputs.getLocalDriveMs());
      Logger.recordOutput("Idun/roboRIO/Tracer/BuiltInInputsMS", tracerInputs.getBuiltInInputsMs());
      Logger.recordOutput("Idun/roboRIO/Tracer/SerializeMS", tracerInputs.getSerializeMs());
      Logger.recordOutput("Idun/roboRIO/Tracer/TransmitMS", tracerInputs.getTransmitMs());
      Logger.recordOutput("Idun/roboRIO/OutputCycleMS", tracerInputs.getOutputCycleMs());

      // Log connection status
      Logger.recordOutput("Idun/Connected", isConnected());
      if (bandwidthTimer.advanceIfElapsed(1)) {
        Logger.recordOutput("Idun/Bandwidth/InputsMbps", incomingBytes * 8.0 / 1.0e6);
        Logger.recordOutput("Idun/Bandwidth/OutputsMbps", outgoingBytes * 8.0 / 1.0e6);
        incomingBytes = 0;
        outgoingBytes = 0;
      }
    } finally {
      inputsLock.unlock();
    }
  }

  /** Send outgoing messages to the server. */
  public static void processOutputs() {
    if (!isConnected()) {
      if (lastConnected) {
        System.out.println("[IdunServer] Client timed out, waiting for reconnection...");
      }
      lastConnected = false;
      return;
    }
    lastConnected = true;

    // Set build and launch UIDs
    outputsBuilder.setBuildUID(IdunBuildConstants.uid);
    outputsBuilder.setLaunchUID(launchUID);

    // Update joystick outputs
    outputsBuilder.clearJoystick();
    for (int id = 0; id < DriverStation.kJoystickPorts; id++) {
      var joystickBuilder = JoystickOutputs.newBuilder();
      joystickBuilder.setOutputs(DriverStationSim.getJoystickOutputs(id));
      joystickBuilder.setLeftRumble(DriverStationSim.getJoystickRumble(id, 0));
      joystickBuilder.setRightRumble(DriverStationSim.getJoystickRumble(id, 1));
      outputsBuilder.addJoystick(joystickBuilder.build());
    }

    // Send outgoing messages
    try {
      byte[] payload = outputsBuilder.build().toByteArray();
      DatagramPacket packet =
          new DatagramPacket(payload, payload.length, clientAddress, clientPort);
      socket.send(packet);
      outgoingBytes += payload.length;
    } catch (IOException e) {
      e.printStackTrace();
    }
  }

  private IdunServer() {}
}
