# Phoenix 6 CAN Frames

Used as a reference for optimization of status signal frequencies. Based only on the public Phoenix API with no reverse engineering.

- API 25.4.0
- TalonFX 25.2.1.0
- Pigeon 25.5.0.0
- CANcoder 25.5.1.0

## Talon FX on CAN 2.0

- Velocity, Position, Acceleration

- RotorVelocity, RotorPosition

- StatorCurrent, SupplyCurrent, SupplyVoltage, DeviceTemp, ProcessorTemp, AncillaryDeviceTemp

- ControlMode, MotionMagicIsRunning, RobotEnable, DeviceEnabled, ClosedLoopIntegratedOutput, ClosedLoopFeedForward

- MotorVoltage, ForwardLimit, ReverseLimit, AppliedRotorPolarity, DutyCycle, TorqueCurrent, BridgeOutput

- MotorOutputStatus, ClosedLoopReferenceSlope

- ClosedLoopSlot, ConnectedMotor, ClosedLoopProportionalOutput, ClosedLoopDerivativeOutput, ClosedLoopOutput

- ClosedLoopReference, ClosedLoopError

- FaultField, StickyFaultField

- VersionMajor, VersionMinor, VersionBugfix, VersionBuild, Version, IsProLicensed

- MotorKT, MotorKV, MotorStallCurrent

- DifferentialDifferenceVelocity, DifferentialDifferencePosition

- DifferentialAverageVelocity, DifferentialAveragePosition

- DifferentialControlMode, DifferentialOutput, DifferentialClosedLoopIntegratedOutput, DifferentialClosedLoopFeedForward

- DifferentialClosedLoopSlot, DifferentialClosedLoopProportionalOutput, DifferentialClosedLoopDerivativeOutput, DifferentialClosedLoopOutput

- DifferentialClosedLoopReferenceSlope

- DifferentialClosedLoopReference, DifferentialClosedLoopError

## Talon FX on CAN FD

- MotorVoltage, ForwardLimit, ReverseLimit, AppliedRotorPolarity, DutyCycle, TorqueCurrent, StatorCurrent, SupplyCurrent, SupplyVoltage, DeviceTemp, ProcessorTemp, RotorVelocity, RotorPosition, Velocity, Position, Acceleration, ControlMode, MotionMagicIsRunning, RobotEnable, DeviceEnable, ClosedLoopSlot, MotorOutputStatus, BridgeOutput, AncillaryDeviceTemp, ConnectedMotor, ClosedLoopProportionalOutput, ClosedLoopIntegratedOutput, ClosedLoopFeedForward, ClosedLoopDerivativeOutput, ClosedLoopOutput, ClosedLoopReference, ClosedLoopReferenceSlope, ClosedLoopError

- DifferentialControlMode, DifferentialAverageVelocity, DifferentialAveragePosition, DifferentialDifferenceVelocity, DifferentialDifferencePosition, DifferentialClosedLoopSlot, DifferentialOutput, DifferentialClosedLoopProportionalOutput, DifferentialClosedLoopIntegratedOutput, DifferentialClosedLoopFeedForward, DifferentialClosedLoopDerivativeOutput, DifferentialClosedLoopOutput, DifferentialClosedLoopReference, DifferentialClosedLoopReferenceSlope, DifferentialClosedLoopError

- FaultField, StickyFaultField

- VersionMajor, VersionMinor, VersionBugfix, VersionBuild, Version, IsProLicensed

- MotorKT, MotorKV, MotorStallCurrent

## Pigeon 2 on CAN FD

- Yaw, Pitch, Roll, QuatW, QuatX, QuatY, QuatZ, GravityVectorX, GravityVectorY, GravityVectorZ, Temperature, NoMotionEnabled, NoMotionCount, TemperatureCompensationDisabled, UpTime, AccumGyroX, AccumGyroY, AccumGyroZ, AngularVelocityXWorld, AngularVelocityYWorld, AngularVelocityZWorld, AccelerationX, AccelerationY, AccelerationZ, SupplyVoltage

- AngularVelocityXDevice, AngularVelocityYDevice, AngularVelocityZDevice

- MagneticFieldX, MagneticFieldY, MagneticFieldZ, RawMagneticFieldX, RawMagneticFieldY, RawMagneticFieldZ

- FaultField, StickyFaultField

- VersionMajor, VersionMinor, VersionBugfix, VersionBuild, Version, IsProLicensed

## CANcoder on CAN FD

- Velocity, Position, AbsolutePosition, UnfilteredVelocity, PositionSinceBoot, SupplyVoltage, MagnetHealth

- FaultField, StickyFaultField

- VersionMajor, VersionMinor, VersionBugfix, VersionBuild, Version, IsProLicensed
