# Project Idun

Project Idun is the core framework powering the robot code for 6328's 2026 robot "Darwin". Under Idun, the primary Java robot code runs on the Mac mini while "real" C++ IO implementations run on the roboRIO. A primary objective is to provide an in-season development experience that is as close to "normal" as possible. This objective is supported by the robot code build system, which automatically handles data plumbing in Java and C++ using an annotation processor and provides standard commands (deploy, simulate, etc.) that are fully functional within the split project structure.

*This page provides a technical breakdown of the components of Idun. For a high-level overview, please see [this post](https://www.chiefdelphi.com/t/frc-6328-mechanical-advantage-2026-build-thread/509595/616) on our 2026 build thread instead.*

> [!IMPORTANT]
> This software is provided for educational purposes only. As with all 6328 robot code, Idun was designed only for use within our team and **does not come with any promise of support or long-term maintenance**. For the 2027 FRC season, [Systemcore](https://community.firstinspires.org/introducing-the-future-mobile-robot-controller) provides vastly improved performance compared to the roboRIO without the complexity and risks of Idun.

## 🛜 Networking

Idun is based on a custom network protocol using Protobuf and UDP, with a server implementation in Java and a client implementation in C++.

* **Protobuf**: The [`IdunComms`](src/main/protobuf/IdunComms.proto) file contains the Protobuf definitions for Idun, which includes input and output message types. During each cycle, a single `RobotInputs` message is sent from the roboRIO to the Mac mini and a single `RobotOutputs` message is sent in response.

* **Server**: The [`IdunServer`](src/main/java/org/littletonrobotics/idun/IdunServer.java) class implements the Idun protocol on the Mac mini and provides APIs that are used internally by generated IO implementations for reading inputs and writing outputs.

* **Client**: The [`IdunClient`](src/main/include/idun/IdunClient.h) class implements the Idun protocol on the roboRIO. This class manages the Idun watchdog, which is used by the C++ project to disable outputs if communication is lost between the roboRIO and Mac mini. Outputs are applied from a separate thread. Some built-in outputs (e.g. joystick rumble) are handled directly by this class.

* **Code Restart**: Since the Driver Station's built-in "Restart Robot Code" button only affects the roboRIO code, the Mac mini code publishes a dashboard button which causes both robot programs to restart. When clicked, `IdunServer` sends a restart flag via Protobuf. The `IdunClient` on the roboRIO catches this and executes an OS-level shell command to forcefully shut down and restart the C++ process, ensuring both devices reboot in sync.

* **Packet Loss Tracking**: Every Protobuf packet exchanged contains a sequential `seqnum`. The `IdunClient` actively tracks these sequence numbers to detect dropped UDP packets, calculating and logging a live packet loss ratio metric to monitor network health during a match.

* **Stateless Outputs**: Every IO interface ([example](../src/main/java/org/littletonrobotics/frc2026/subsystems/launcher/flywheel/FlywheelIO.java)) includes a single `applyOutputs` method which sends the full state of the subsystem outputs. This architecture creates a stateless relationship between the subsystem and IO implementation, making the distributed system robust to disconnects or crashes.

## ⚙️ Annotation Processor

Idun's [annotation processor](src/main/processor/org/littletonrobotics/idun/) implements three key features:

* **IdunIO**: This annotation is used on the Java IO interface to generate the corresponding C++ and Java code for Idun. For example, the annotation on `FlywheelIO` generates a `FlywheelIOIdun` implementation for the Java program and a `FlywheelIO` class for the C++ program. These generated classes automatically serialize/deserialize data into Protobuf objects and read/write data from the `IdunServer` and `IdunClient` classes.

* **IdunConstants**: This annotation is used on a Java class containing constants to generate a corresponding C++ header file. For example, adding `@IdunConstants` to [`DriveConstants`](../src/main/java/org/littletonrobotics/frc2026/subsystems/drive/DriveConstants.java) makes the same constant values available in the C++ project without duplicating the definitions.

* **Build Constants**: The annotation processor generates a unique 64-bit build ID for both the Java and C++ programs. The `IdunServer` and `IdunClient` verify that these IDs match when communicating, ensuring that the Mac mini and roboRIO are always running the exact same version of the code.

## ✉️ Deployment

Idun integrates deeply with the GradleRIO build system to support seamless deployment, including a Java environment optimized for macOS.

* **GradleRIO Extensions**: Idun includes a set of custom GradleRIO extensions in the [`org.littletonrobotics.gradlerio`](gradlerio/src/main/java/org/littletonrobotics/gradlerio) package. These extend GradleRIO to support deploying to the Mac mini via SSH (including native macOS dependencies) and managing the Java program's lifecycle via a [launchd service](../macmini/org.littletonrobotics.robot.plist).

* **Multi-Target Deploy**: The [`build.gradle`](../build.gradle) defines two deployment targets: `macmini` for the Java project and `roborio` for the C++ project. Running a standard deploy command deploys to both targets in parallel. Deploying to the Mac mini is supported on all platforms, so students developing on Windows/Linux can still deploy the appropriate macOS/Athena native dependencies to each device.

* **Vendordeps**: Vendor libraries are isolated between the Java and C++ projects, ensuring that each device can only access the libraries that make sense in context (e.g. Phoenix is only used in C++ and AdvantageKit is only used in Java). GradleRIO's built-in vendordep management (via the "vendordeps" folder) is used for the C++ project, and dependencies for the Java project (including native dependencies) are handled manually in `build.gradle`.

* **Simulation**: The Java project operates independently in simulation without needing to build or run the C++ project, using separate IO Java implementations (e.g. `FlywheelIOSim`) similar to a standard AdvantageKit project. The C++ project can be manually launched in simulation when testing the Idun layer itself.

* **Java Optimizations**: The Mac mini runs a Java 25 JRE for access to the latest optimizations, so the Java program runs with flags optimizing the garbage collector (including generational ZGC and compact object headers).

* **Log Server**: WPILOG files for the Java project are stored on the Mac mini's internal SSD and accessible via a custom [Python server](../macmini/log_server.py), since AdvantageScope's built-in log downloading feature ([docs](https://docs.advantagescope.org/overview/log-files/#downloading-from-the-robot)) is only compatible with the roboRIO.

## 🤖 C++ Project

The roboRIO C++ program acts as the thin client for the system, primarily handling hardware IO, network diagnostics, and fail-safe local driving.

* **Profiling**: The roboRIO precisely profiles the latency of every step in the communication loop—from polling hardware inputs to serialization, transmission, and output application. These metrics are packed into a `TracerTimes` struct and shipped to the Mac mini every cycle, allowing network latency and CPU spikes on both devices to be tracked with sub-millisecond precision.

* **Logging**: [`IdunLogManager`](src/main/include/idun/IdunLogManager.h) acts as a centralized data logger for the roboRIO, similar to WPILib's `DataLogManager`. It logs DS info, input/output sequence numbers, packet loss ratio, ping success, and occasionally samples full Protobuf payloads. While the Mac mini logs are the "primary" logs used in almost all cases, the roboRIO logs provide additional data when the Idun layer itself requires debugging.

* **Process UIDs**: The build UID and launch UID are logged for both the Mac mini and roboRIO via [`IdunLogger`](src/main/include/idun/IdunLogger.h). This is useful to verify that both devices are running matching versions of the code and to see if one device crashed and rebooted (since the launch UID is regenerated on every launch). All of these UIDs are logged by *both* the roboRIO and Mac mini, giving us a complete picture of what both devices see throughout any communication failure.

* **Ping**: The roboRIO's [`IdunPing`](src/main/include/idun/IdunPing.h) class periodically sends ping packets to the Mac mini when disabled to monitor network health and connection stability. This is used to differentiate between failures of the Mac mini power/OS and the Java robot program itself.

* **Local Drive**: "Local drive" is a safety/backup feature that allows the roboRIO to be driven locally by flipping an override switch on the operator console. This stops the C++ program from applying subsystem outputs from the Mac mini and calculates swerve drive outputs in [`LocalDrive`](../src/main/cpp/LocalDrive.cpp) based on the joysticks and gyro.

* **Gyro Offset**: The gyro offset is calculated in the Java [`Drive`](../src/main/java/org/littletonrobotics/frc2026/subsystems/drive/Drive.java) subsystem and published to the `IdunServer`. This allows the C++ local drive to use the same field-relative rotation as the main Java code, even if the Mac mini loses connection later in the match.

* **DS Integration**: With a typical WPILib project, the Driver Station displays "No Robot Code" until the code is running (this status is also communicated to the FMS). Idun delays this signal until the connection to the Mac mini is established, even if the C++ robot program is already initialized. This makes it clear to the drive team and field staff when the robot is ready to enable. In case of issues, enabling "local drive" overrides this behavior and immediately signals that the robot code is ready.

## ☕️ Java Project

The Mac mini Java program acts as the central brain of the robot, executing the main control loop and high-level logic.

* **Simulation Layer Bridging**: [`IdunServer.java`](src/main/java/org/littletonrobotics/idun/IdunServer.java) directly bridges the incoming roboRIO telemetry into WPILib's simulation API (e.g. `DriverStationSim` and `RoboRioSim`). This means that when the Java code on the Mac mini queries standard WPILib methods, they return the live values exactly as if the program were running on the roboRIO.

* **Loop Cycle Timing**: [`IdunRobot`](src/main/java/org/littletonrobotics/idun/IdunRobot.java) extends `IterativeRobotBase` and supports multiple timing modes. In `SYNC` mode, it waits for inputs from the roboRIO (`IdunServer.waitForInputs()`), guaranteeing that the Mac mini processes each cycle from the roboRIO with minimal latency. `FIXED` mode is used when disconnected from the roboRIO, and `FAST` mode is used for AdvantageKit replay.

* **FullSubsystem**: [`FullSubsystem`](../src/main/java/org/littletonrobotics/frc2026/util/FullSubsystem.java) extends WPILib's `SubsystemBase` by providing a `periodicAfterScheduler()` callback. This allows the Java program to calculate all logic and states during the command scheduler, then publish outputs to the C++ program simultaneously at the end of the loop cycle.

* **Thread Time Constraint Policy**: The robot program uses a native Mach kernel feature in macOS ([`MachThreading`](../src/main/java/org/littletonrobotics/frc2026/util/darwin/MachThreading.java)) that allows the JVM's main robot loop thread to request real-time priority from the OS scheduler, vastly improving loop cycle times on the Mac mini.

* **WPILOGXZ**: A custom AdvantageKit data log writer ([`WPILOGXZWriter`](../src/main/java/org/littletonrobotics/frc2026/util/logging/WPILOGXZWriter.java)) uses XZ (LZMA2) compression to reduce the file size of logs by ~5x, making it far more practical to download/manage log files that include 200 Hz data. The output format (`.wpilogxz`) is natively supported by AdvantageScope. This class also uses POSIX `F_FULLFSYNC` to ensure data is written to the physical drive, minimizing the risk of log corruption on sudden power loss.

* **macOS Power Management**: The [`DarwinPowerMode`](../src/main/java/org/littletonrobotics/frc2026/util/darwin/DarwinPowerMode.java) class enables and disables the Mac mini's [low power mode](https://support.apple.com/en-us/101613) via a `pmset` command to minimize energy usage when disabled. For energy logging, the [`DarwinPowerMonitor`](../src/main/java/org/littletonrobotics/frc2026/util/darwin/DarwinPowerMonitor.java) class interfaces with the third-party [SAP Power Monitor](https://github.com/SAP/power-monitoring-tool-for-macos) application to record the device power draw in watts.
