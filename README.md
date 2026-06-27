# RobotCode2026 (Public)

[![License](https://img.shields.io/badge/License-MIT-blue)](https://github.com/Mechanical-Advantage/RobotCode2026Public/blob/main/LICENSE) ![Last Commit](https://img.shields.io/github/last-commit/Mechanical-Advantage/RobotCode2026Public?color=yellow)

### [Darwin Technical Binder](https://drive.google.com/file/d/1jdVGivUltPaQ3_wGDXRlkQZBLpTBDL2V/view?usp=sharing)

This project contains robot code for 6328's 2026 robot "Darwin". See 6328's Open Alliance [build thread](https://www.chiefdelphi.com/t/frc-6328-mechanical-advantage-2026-build-thread/509595) on Chief Delphi for details, or browse 6328's robot code from previous games: [2025](https://github.com/Mechanical-Advantage/RobotCode2025Public), [2024](https://github.com/Mechanical-Advantage/RobotCode2024Public), [2023](https://github.com/Mechanical-Advantage/RobotCode2023), [2022](https://github.com/Mechanical-Advantage/RobotCode2022).

This repository is updated daily with the latest stable code from 6328's development repository. Please contact software@team6328.org with any questions.

![Banner](/banner.jpg)

## Setup

Before cloning the repository on Windows, please follow the steps below. These steps can be skipped on macOS and Linux.

1. Enable "Developer Mode" in System Settings.
2. Run "git config --global core.symlinks true"
3. Clone the repository as normal

After cloning, follow the steps below to set up the project for the first time:

- [ ] Install Python 3.9+ and ensure it is available on the PATH.
- [ ] Install the recommended VSCode extensions when prompted.
- [ ] Copy the JSON files from the `sim` folder to the root directory of the robot project.
- [ ] Configure the "Custom Assets Folder" in AdvantageScope ([docs](https://docs.advantagescope.org/more-features/custom-assets/)) to the `ascope_assets` folder.
- [ ] Set the "Robot Address" in AdvantageScope ([docs](https://docs.advantagescope.org/overview/live-sources/)) to `10.63.28.10` and use the "NetworkTables 4 (AdvantageKit)" live mode.

## Gradle Tasks

Below is a list of helpful Gradle tasks. Note that deploying to only one device is not recommended, as Idun requires that both components were built together when running on the real robot.

```bash
./gradlew build # Build all code (Java and C++)

./gradlew deploy # Deploy to Mac mini and roboRIO in parallel
./gradlew deploymacmini # Deploy to the Mac mini
./gradlew deployroborio # Deploy to the roboRIO

./gradlew simulateJava # Simulate Java Mac mini project
./gradlew simulateNative # Simulate C++ roboRIO project (not available by default)
```
