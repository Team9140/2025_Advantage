package org.team9140.frc2025.subsystems.superstructure.elevator;

import org.littletonrobotics.junction.AutoLog;

// Heavily inspired by Team Mechanical Advantage
// https://github.com/Mechanical-Advantage/RobotCode2025Public/blob/main/src/main/java/org/littletonrobotics/frc2025/subsystems/superstructure/elevator
public interface ElevatorIO {
  @AutoLog
  class ElevatorIOInputs {
    public ElevatorIOData data =
        new ElevatorIOData(false, false, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  }

  record ElevatorIOData(
      boolean motorConnected,
      boolean followerConnected,
      double positionRad,
      double velocityRadPerSec,
      double appliedVolts,
      double torqueCurrentAmps,
      double supplyCurrentAmps,
      double tempCelsius,
      double followerAppliedVolts,
      double followerTorqueCurrentAmps,
      double followerSupplyCurrentAmps,
      double followerTempCelsius) {}

  default void updateInputs(ElevatorIOInputs inputs) {}

  default void runOpenLoop(double output) {}

  default void runVolts(double volts) {}

  default void stop() {}

  /** Run elevator output shaft to positionRad with additional feedforward output */
  default void runPosition(double positionRad, double feedforward) {}

  default void setPID(double kP, double kI, double kD) {}

  default void setBrakeMode(boolean enabled) {}
}
